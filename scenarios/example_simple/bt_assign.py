import json
import math
import re
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple, List, Set

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8, String, Float64

from modules.base_bt_nodes import BTNodeList, Node, Status


CUSTOM_ACTION_NODES = ["AssignTarget"]
BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)


@dataclass
class Lease:
    agent_id: str
    bid: float
    expires_at: float


class AssignTarget(Node):
    """
    AssignTarget (Decentralized lease-based assignment)
    ---------------------------------------------------
    Fire_UGV: "continuous lease" 방식으로 최대한 단순/안정화
      - 가장 가까운 Fire 1개만 선택
      - 다른 agent의 유효 lease가 더 좋으면 차선책 선택
      - 선택된 Fire는 최소 홀드타임 동안 유지 (스위칭 억제)
      - Fire가 모두 없으면 SUCCESS 반환 (MoveToBase로 넘어가게)

    Target(UAV/Rescue_UGV)은 기존 동작 유지 가능(지금은 문제 없다 했으니),
    필요하면 이후에 Target도 같은 lease 방식으로 단순화 가능.
    """

    BID_TOPIC = "/assign_target/bid"
    CLAIM_TOPIC = "/assign_target/claim"

    # Target status
    UNCHECKED = 0
    CHECKED = 1
    COMPLETED = 2

    # ---- Tuning (안정성 우선) ----
    DISCOVER_PERIOD = 0.30

    # Fire_UGV reassignment control
    REEVAL_PERIOD = 0.85 # 0.25          # 너무 잦으면 빙빙 돎, 너무 길면 반응 느림
    MIN_HOLD_TIME = 10.0 # 1.2           # 최소 유지 시간 (스위칭 억제)
    SWITCH_MARGIN = 30.0          # 새 bid가 이만큼 더 좋아야 갈아탐

    # Lease
    CLAIM_TTL = 2.0 # 1.6
    RENEW_MARGIN = 0.55           # 남은 TTL 이 값보다 작으면 갱신

    # Cooldown (락에 걸렸거나 뺏긴 Fire를 잠깐 스킵)
    COOLDOWN_SEC = 2.5 # 0.8

    def __init__(self, name: str, agent):
        super().__init__(name)
        self.type = "Action"
        self.ros = agent.ros_bridge
        self.agent_id = self._infer_agent_id(agent)

        self._pub_bid = self.ros.node.create_publisher(String, self.BID_TOPIC, 10)
        self._pub_claim = self.ros.node.create_publisher(String, self.CLAIM_TOPIC, 10)
        self.ros.node.create_subscription(String, self.BID_TOPIC, self._on_bid, 10)
        self.ros.node.create_subscription(String, self.CLAIM_TOPIC, self._on_claim, 10)

        # World caches
        self._my_pose: Optional[PoseStamped] = None
        self._poses_fire: Dict[str, PoseStamped] = {}
        self._fire_radius: Dict[str, float] = {}

        # Target caches (지금은 유지용)
        self._poses_target: Dict[str, PoseStamped] = {}
        self._status_target: Dict[str, int] = {}

        self._subscribed_topics: Set[str] = set()

        # tables
        self._bids_table: Dict[str, Dict[str, float]] = {}
        self._leases_table: Dict[str, Lease] = {}

        # current assignment
        self._current_object_id: Optional[str] = None
        self._current_bid: float = -1e18
        self._last_switch_time: float = 0.0

        # timing
        self._last_discover_time = 0.0
        self._last_reeval_time = 0.0
        self._cooldown_until: Dict[str, float] = {}

        # subscribe my pose
        self._ensure_sub(PoseStamped, f"/{self.agent_id}/pose_world", self._on_my_pose)
        print("[AssignTarget] inferred agent_id =", self.agent_id, "ros_namespace =", getattr(agent, "ros_namespace", None))

    # ---------------- ROS callbacks ----------------

    def _on_my_pose(self, msg: PoseStamped):
        self._my_pose = msg

    def _on_bid(self, msg: String):
        try:
            data = json.loads(msg.data)
            obj_id = data["target"]
            bidder = data["agent"]
            bid_value = float(data["bid"])
        except Exception:
            return
        self._bids_table.setdefault(obj_id, {})[bidder] = bid_value

    def _on_claim(self, msg: String):
        try:
            data = json.loads(msg.data)
            obj_id = data["target"]
            owner = data["agent"]
            bid_value = float(data["bid"])
            expires_at = float(data["expires_at"])
        except Exception:
            return

        cur = self._leases_table.get(obj_id)
        if cur is None or self._claim_is_better(owner, bid_value, expires_at, cur):
            self._leases_table[obj_id] = Lease(agent_id=owner, bid=bid_value, expires_at=expires_at)

    # ---------------- BT tick ----------------

    async def run(self, agent, blackboard):
        now = time.time()

        # (1) discover topics
        if now - self._last_discover_time >= self.DISCOVER_PERIOD:
            self._last_discover_time = now
            self._discover_world_topics()

        # (2) need my pose
        if self._my_pose is None:
            blackboard["assigned_target"] = None
            self.status = Status.SUCCESS # Status.RUNNING
            return self.status

        # (3) clear invalid current (fire suppressed etc.)
        self._validate_current_assignment(agent, now)

        # (4) Fire_UGV: if no active fires -> SUCCESS (중요!)
        if agent.type == "Fire_UGV":
            if not self._poses_fire:
                blackboard["assigned_target"] = None
                self._current_object_id = None
                self._current_bid = -1e18
                self.status = Status.SUCCESS
                return self.status

        # (5) renew lease if needed
        self._renew_claim_if_needed(now)

        # (6) decide/re-evaluate periodically
        if agent.type == "Fire_UGV":
            if now - self._last_reeval_time >= self.REEVAL_PERIOD:
                self._last_reeval_time = now
                self._fire_ugv_pick_and_claim(now)

        elif agent.type in ("UAV", "Rescue_UGV"):
            if now - self._last_reeval_time >= self.REEVAL_PERIOD:
                self._last_reeval_time = now
                self._target_pick_and_claim(agent, now)

        # (7) publish blackboard
        blackboard["assigned_target"] = self._current_object_id
        self.status = Status.SUCCESS # Status.RUNNING
        return self.status

    # ============================================================
    # Fire_UGV simplified assignment
    # ============================================================

    def _fire_ugv_pick_and_claim(self, now: float):
        """
        1) 모든 fire에 대해 내 bid(=-dist^2) 계산
        2) (쿨다운/타 로봇의 더 좋은 유효 lease)인 fire는 스킵
        3) 남는 것 중 최고 bid 선택
        4) 현재 타겟 유지 정책:
           - 현재 fire가 여전히 유효하고, 새 후보가 margin 만큼 더 좋지 않으면 유지
           - MIN_HOLD_TIME 안에는 절대 스위치 안 함
        5) 선택된 fire에 대해 claim publish (continuous lease)
        """
        best_id, best_bid = self._best_available_fire_for_me(now)
        if best_id is None:
            # 모든 Fire가 다른 agent에게 점유된 상태 → MoveToBase로 넘어가게
            if self._current_object_id and self._current_object_id.startswith("Fire_"):
                print(f"[AssignTarget] {self.agent_id} no available fire, releasing current")
                self._current_object_id = None
                self._current_bid = -1e18
            return

        # current 유지 판단
        if self._current_object_id and self._current_object_id.startswith("Fire_"):
            cur_id = self._current_object_id
            cur_bid = self._compute_bid_for_fire(cur_id)
            # 현재도 여전히 "내가 가져갈 수 있는 fire"이면 유지 성향
            if cur_bid is not None:
                held_long_enough = (now - self._last_switch_time) >= self.MIN_HOLD_TIME
                # 스위치 조건: 홀드타임 만족 + 새 후보가 충분히 더 좋음
                if (best_id != cur_id) and held_long_enough and (best_bid > cur_bid + self.SWITCH_MARGIN):
                    self._switch_to_fire(best_id, best_bid, now)
                else:
                    # 유지 (현재 fire claim 갱신용 publish)
                    self._publish_bid(cur_id, cur_bid)
                    self._publish_claim(cur_id, cur_bid, now)
                    self._current_bid = cur_bid
                    return

        # current 없으면 바로 채택
        self._switch_to_fire(best_id, best_bid, now)

    def _switch_to_fire(self, fire_id: str, bid: float, now: float):
        self._current_object_id = fire_id
        self._current_bid = bid
        self._last_switch_time = now
        self._publish_bid(fire_id, bid)
        self._publish_claim(fire_id, bid, now)
        print(f"[AssignTarget] {self.agent_id} ASSIGNED -> {fire_id} (bid={bid:.2f})")

    def _best_available_fire_for_me(self, now: float) -> Tuple[Optional[str], float]:
        best_id = None
        best_bid = -1e18

        for fire_id, pose in self._poses_fire.items():
            if self._cooldown_until.get(fire_id, 0.0) > now:
                continue

            bid = self._compute_bid_for_fire(fire_id)
            if bid is None:
                continue

            # 다른 로봇이 더 좋은 유효 lease를 들고 있으면 스킵
            lease = self._leases_table.get(fire_id)
            if lease and lease.expires_at > now and lease.agent_id != self.agent_id:
                if not self._is_bid_better(self.agent_id, bid, lease.agent_id, lease.bid):
                    continue

            if self._is_bid_better(self.agent_id, bid, best_id or self.agent_id, best_bid):
                best_id, best_bid = fire_id, bid

        return best_id, best_bid

    def _compute_bid_for_fire(self, fire_id: str) -> Optional[float]:
        if self._my_pose is None:
            return None
        pose = self._poses_fire.get(fire_id)
        if pose is None:
            return None
        ax = self._my_pose.pose.position.x
        ay = self._my_pose.pose.position.y
        fx = pose.pose.position.x
        fy = pose.pose.position.y
        dx = fx - ax
        dy = fy - ay
        return -(dx * dx + dy * dy)

    def _validate_current_assignment(self, agent, now: float):
        """
        - Fire가 suppressed 되면(=publisher 0 / cache에서 제거) current 해제
        - current가 다른 agent에게 확실히 뺏기면 cooldown 걸고 해제
        """
        cur = self._current_object_id
        if not cur:
            return

        # Fire suppressed or missing
        if cur.startswith("Fire_"):
            if cur not in self._poses_fire:
                self._current_object_id = None
                self._current_bid = -1e18
                return

            # MIN_HOLD_TIME 체크: 최소 유지 시간 동안은 뺏기지 않음
            if (now - self._last_switch_time) < self.MIN_HOLD_TIME:
                return

            # 확실히 뺏긴 경우(유효 lease가 타인이고 내가 못 이기면)
            lease = self._leases_table.get(cur)
            if lease and lease.expires_at > now and lease.agent_id != self.agent_id:
                my_bid = self._compute_bid_for_fire(cur)
                if my_bid is None:
                    return
                if not self._is_bid_better(self.agent_id, my_bid, lease.agent_id, lease.bid):
                    self._cooldown_until[cur] = now + self.COOLDOWN_SEC
                    self._current_object_id = None
                    self._current_bid = -1e18

    def _target_pick_and_claim(self, agent, now: float):
        """
        UAV: UNCHECKED target만
        Rescue_UGV: CHECKED target만
        """
        # UAV는 CHECKED 이상이면 release해서 Rescue가 먹게
        if agent.type == "UAV" and self._current_object_id and self._current_object_id.startswith("Target_"):
            st = self._status_target.get(self._current_object_id, self.UNCHECKED)
            if st != self.UNCHECKED:
                print(f"[AssignTarget] {self.agent_id} releasing Target (status={st}, not UNCHECKED)")
                self._publish_release_claim(self._current_object_id)
                self._leases_table.pop(self._current_object_id, None)
                self._current_object_id = None
                self._current_bid = -1e18

        # Rescue_UGV는 COMPLETED가 되면 release
        if agent.type == "Rescue_UGV" and self._current_object_id and self._current_object_id.startswith("Target_"):
            st = self._status_target.get(self._current_object_id, self.UNCHECKED)
            if st != self.CHECKED:
                print(f"[AssignTarget] {self.agent_id} releasing Target (status={st}, not CHECKED)")
                self._publish_release_claim(self._current_object_id)
                self._leases_table.pop(self._current_object_id, None)
                self._current_object_id = None
                self._current_bid = -1e18

        desired = self.UNCHECKED if agent.type == "UAV" else self.CHECKED
        best_id, best_bid = self._best_available_target_for_me(now, desired_status=desired)
        if best_id is None:
            return

        # 현재 타겟 유지(스위칭 억제)
        if self._current_object_id and self._current_object_id.startswith("Target_"):
            cur_id = self._current_object_id
            cur_bid = self._compute_bid_for_target(cur_id)
            if cur_bid is not None:
                held_long_enough = (now - self._last_switch_time) >= self.MIN_HOLD_TIME
                if (best_id != cur_id) and held_long_enough and (best_bid > cur_bid + self.SWITCH_MARGIN):
                    self._switch_to_target(best_id, best_bid, now)
                else:
                    self._publish_bid(cur_id, cur_bid)
                    self._publish_claim(cur_id, cur_bid, now)
                    self._current_bid = cur_bid
                    return

        self._switch_to_target(best_id, best_bid, now)


    def _switch_to_target(self, target_id: str, bid: float, now: float):
        self._current_object_id = target_id
        self._current_bid = bid
        self._last_switch_time = now
        self._publish_bid(target_id, bid)
        self._publish_claim(target_id, bid, now)
        print(f"[AssignTarget] {self.agent_id} ASSIGNED -> {target_id} (bid={bid:.2f})")


    def _best_available_target_for_me(self, now: float, desired_status: int) -> Tuple[Optional[str], float]:
        best_id = None
        best_bid = -1e18

        for tid, pose in self._poses_target.items():
            st = self._status_target.get(tid, self.UNCHECKED)
            if st == self.COMPLETED:
                continue
            if st != desired_status:
                continue
            if self._cooldown_until.get(tid, 0.0) > now:
                continue

            bid = self._compute_bid_for_target(tid)
            if bid is None:
                continue

            lease = self._leases_table.get(tid)
            if lease and lease.expires_at > now and lease.agent_id != self.agent_id:
                # (특수처리) CHECKED target을 UAV가 잡고 있는 lease는 lock으로 취급 안 함
                if st == self.CHECKED and str(lease.agent_id).startswith("UAV"):
                    pass
                else:
                    if not self._is_bid_better(self.agent_id, bid, lease.agent_id, lease.bid):
                        continue

            if self._is_bid_better(self.agent_id, bid, best_id or self.agent_id, best_bid):
                best_id, best_bid = tid, bid

        return best_id, best_bid


    def _compute_bid_for_target(self, target_id: str) -> Optional[float]:
        if self._my_pose is None:
            return None
        pose = self._poses_target.get(target_id)
        if pose is None:
            return None
        ax = self._my_pose.pose.position.x
        ay = self._my_pose.pose.position.y
        tx = pose.pose.position.x
        ty = pose.pose.position.y
        dx = tx - ax
        dy = ty - ay
        return -(dx * dx + dy * dy)


    def _publish_release_claim(self, obj_id: str):
        msg = String()
        msg.data = json.dumps({
            "agent": self.agent_id,
            "target": obj_id,
            "bid": -1e18,
            "expires_at": 0.0
        })
        self._pub_claim.publish(msg)

    # ============================================================
    # Lease/bid publish & renew
    # ============================================================

    def _publish_bid(self, obj_id: str, bid: float):
        msg = String()
        msg.data = json.dumps({"agent": self.agent_id, "target": obj_id, "bid": float(bid)})
        self._pub_bid.publish(msg)

    def _publish_claim(self, obj_id: str, bid: float, now: float):
        expires_at = now + self.CLAIM_TTL
        msg = String()
        msg.data = json.dumps({
            "agent": self.agent_id,
            "target": obj_id,
            "bid": float(bid),
            "expires_at": float(expires_at)
        })
        self._pub_claim.publish(msg)
        self._leases_table[obj_id] = Lease(agent_id=self.agent_id, bid=float(bid), expires_at=float(expires_at))

    def _renew_claim_if_needed(self, now: float):
        if self._current_object_id is None:
            return
        lease = self._leases_table.get(self._current_object_id)
        if not lease:
            return
        if lease.agent_id != self.agent_id:
            return
        if (lease.expires_at - now) < self.RENEW_MARGIN:
            # 최신 bid로 갱신 (특히 이동하면서 bid가 바뀌므로)
            if self._current_object_id.startswith("Fire_"):
                bid = self._compute_bid_for_fire(self._current_object_id)
                if bid is None:
                    bid = lease.bid
            else:
                bid = lease.bid
            self._publish_bid(self._current_object_id, bid)
            self._publish_claim(self._current_object_id, bid, now)

    # ============================================================
    # Deterministic compare
    # ============================================================

    def _is_bid_better(self, agent_a: str, bid_a: float, agent_b: str, bid_b: float) -> bool:
        if bid_a > bid_b + 1e-9:
            return True
        if abs(bid_a - bid_b) <= 1e-9 and agent_a < agent_b:
            return True
        return False

    def _claim_is_better(self, agent_id: str, bid: float, expires_at: float, cur: Lease) -> bool:
        now = time.time()
        if cur.expires_at <= now:
            return True
        if self._is_bid_better(agent_id, bid, cur.agent_id, cur.bid):
            return True
        if abs(bid - cur.bid) <= 1e-9 and agent_id == cur.agent_id and expires_at > cur.expires_at:
            return True
        return False

    # ============================================================
    # Discovery (Fire 중심 유지 + Target은 기존 캐시 유지)
    # ============================================================

    def _discover_world_topics(self):
        try:
            topics = self.ros.node.get_topic_names_and_types()
        except Exception:
            return

        fire_pose_regex = re.compile(r"^/world/fire/(Fire_\d+)/pose$")
        fire_radius_regex = re.compile(r"^/world/fire/(Fire_\d+)/radius$")

        target_pose_regex = re.compile(r"^/world/target/(Target_\d+)/pose$")
        target_status_regex = re.compile(r"^/world/target/(Target_\d+)/status$")

        # Target: publisher 없는 건 제거
        active_target_ids: Set[str] = set()
        # Fire: publisher 없는 건 제거
        active_fire_ids: Set[str] = set()

        for topic_name, _types in topics:
            m = target_pose_regex.match(topic_name)
            if m:
                target_id = m.group(1)
                pubs = self.ros.node.get_publishers_info_by_topic(topic_name)
                if len(pubs) > 0:
                    active_target_ids.add(target_id)
                # 기존 subscribe 로직 유지

        for topic_name, _types in topics:
            m = fire_pose_regex.match(topic_name)
            if m:
                fire_id = m.group(1)
                pubs = self.ros.node.get_publishers_info_by_topic(topic_name)
                if len(pubs) > 0:
                    active_fire_ids.add(fire_id)
                if topic_name not in self._subscribed_topics:
                    self._ensure_sub(
                        PoseStamped,
                        topic_name,
                        lambda msg, k=fire_id: self._poses_fire.__setitem__(k, msg)
                    )
                continue

            m = fire_radius_regex.match(topic_name)
            if m:
                fire_id = m.group(1)
                if topic_name not in self._subscribed_topics:
                    self._ensure_sub(
                        Float64,
                        topic_name,
                        lambda msg, k=fire_id: self._fire_radius.__setitem__(k, float(msg.data))
                    )
                continue

            # Target (유지)
            m = target_pose_regex.match(topic_name)
            if m:
                target_id = m.group(1)
                if topic_name not in self._subscribed_topics:
                    self._ensure_sub(
                        PoseStamped,
                        topic_name,
                        lambda msg, k=target_id: self._poses_target.__setitem__(k, msg)
                    )
                continue

            m = target_status_regex.match(topic_name)
            if m:
                target_id = m.group(1)
                if topic_name not in self._subscribed_topics:
                    self._ensure_sub(
                        UInt8,
                        topic_name,
                        lambda msg, k=target_id: self._status_target.__setitem__(k, int(msg.data))
                    )
                continue

        # publisher 없는 fire 제거 (suppressed)
        for fire_id in list(self._poses_fire.keys()):
            if fire_id not in active_fire_ids:
                # 혹시 topic 목록에 잠깐 안 보이는 케이스 대비해서 publisher 재확인
                topic = f"/world/fire/{fire_id}/pose"
                pubs = self.ros.node.get_publishers_info_by_topic(topic)
                if len(pubs) == 0:
                    self._poses_fire.pop(fire_id, None)
                    self._fire_radius.pop(fire_id, None)
                    self._leases_table.pop(fire_id, None)
                    if self._current_object_id == fire_id:
                        self._current_object_id = None
                        self._current_bid = -1e18

        for target_id in list(self._poses_target.keys()):
            if target_id not in active_target_ids:
                self._poses_target.pop(target_id, None)
                self._status_target.pop(target_id, None)
                self._leases_table.pop(target_id, None)
                if self._current_object_id == target_id:
                    self._current_object_id = None
                    self._current_bid = -1e18

    def _ensure_sub(self, msg_type, topic: str, callback):
        if topic in self._subscribed_topics:
            return
        self.ros.node.create_subscription(msg_type, topic, callback, 10)
        self._subscribed_topics.add(topic)

    # ============================================================
    # Agent identity
    # ============================================================

    def _infer_agent_id(self, agent) -> str:
        namespace = getattr(agent, "ros_namespace", "") or ""
        if isinstance(namespace, str):
            cleaned = namespace.strip().strip("/")
            if cleaned:
                return cleaned

        agent_type = getattr(agent, "type", "") or ""
        if agent_type in ("Rescue_UGV", "Fire_UGV", "UAV"):
            return f"{agent_type}_1"
        return "agent_1"