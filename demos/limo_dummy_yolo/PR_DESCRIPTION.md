제목: Add standalone RGB-D human dummy perception topics for BT

기존 더미 인식 데모는 시뮬레이터 소스에 의존해 인식 기능만 기존 BT에 연결하기 어려웠습니다. YOLO 추론과 깊이 좌표 계산을 독립 `human_yolo` ROS 패키지로 분리하고, 모델과 설정·의존성·실행 문서를 포함합니다.

- `/human/detected` (`Bool`), `/human/position_valid` (`Bool`), `/human/position` (`PointStamped`) 및 프레임별 상세 JSON을 제공합니다.
- 좌표는 RGB 촬영 시각과 카메라 광학 frame_id를 유지하며, 깊이가 없거나 오래된 경우 유효 좌표를 발행하지 않습니다. 카메라 입력 중단 시 Bool 상태를 만료시킵니다.
- main 공장 BT 호환 출력 `/perception/is_person` (`Bool`)과 `/perception/person_pose` (`PoseStamped`)도 제공합니다.
- 시뮬레이터·맵·로봇 모델·별도 이동/접근/회전 제어·BT 실행기·학습 데이터는 포함하지 않습니다.
- 기존 시뮬레이터/카메라를 실행하고 인식 노드를 별도로 시작합니다. BT 구독 타입, 좌표 단위/축, 시간 제한과 실패 처리를 README에 명시합니다.

검증: 기하·동기화 및 ROS 토픽 계약 테스트 12개, 독립 패키지 colcon 빌드. 폴더를 별도로 복사해 빌드·설치한 뒤 실제 `best.pt`의 감지 결과와 ROS `PointStamped` 출력을 검증했습니다(정렬된 1.2m 시험 깊이 입력에서 z=1.2m 확인). 영상 입력은 검증에만 사용했으며 PR에 포함하지 않습니다.

리뷰 범위는 `demos/limo_dummy_yolo/`입니다. 로컬의 다른 BT 시나리오나 시뮬레이터 작업은 이 PR에 추가하지 않습니다. main 시뮬레이터의 launch를 자동 변경하는 PR이 아니며, 실제 입력 토픽과 RGB-D 정렬은 기존 환경에서 설정해야 합니다.

새 main(699d2f6) + 공개 시뮬레이터 main(68d4fc0) + 본 패키지만 별도 빌드해 기본 월드의 RGB-D 연결을 확인했습니다. 검증용 더미를 추가한 별도 월드 사본에서 77/77 프레임 탐지, 75개 유효 좌표 출력, main 실제 `IsPerson` 노드의 SUCCESS를 확인했습니다. 기본 월드 자체에는 더미가 없습니다. 공장 월드/launch는 공개 시뮬레이터 main에 없어 공장 전체 주행·map 좌표 기록까지 검증했다고 주장하지 않습니다.
