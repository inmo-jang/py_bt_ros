"""Aligned RGB-D projection; no simulator, motion, BT or TF dependencies."""
import math
import numpy as np


def depth_point(depth, bbox, info, min_depth=.15, max_depth=3.):
    """Return a visible foreground surface point in optical-frame metres."""
    if depth.encoding not in ('16UC1', '32FC1'):
        return None
    if (depth.width, depth.height) != (info.width, info.height):
        return None
    try:
        x0, y0, x1, y1 = map(float, bbox)
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        if not all(math.isfinite(v) for v in [x0,y0,x1,y1,fx,fy,cx,cy]):
            return None
        if x1-x0 < 16 or y1-y0 < 16 or fx <= 0 or fy <= 0:
            return None
        left, right = max(0,int(x0+.15*(x1-x0))), min(depth.width,int(x0+.85*(x1-x0)))
        top, bottom = max(0,int(y0+.35*(y1-y0))), min(depth.height,int(y0+.65*(y1-y0)))
        if right <= left or bottom <= top:
            return None
        dtype = np.dtype(('>' if depth.is_bigendian else '<') + ('u2' if depth.encoding=='16UC1' else 'f4'))
        if depth.step < depth.width*dtype.itemsize or depth.step % dtype.itemsize:
            return None
        if len(depth.data) != depth.height*depth.step:
            return None
        a = np.frombuffer(depth.data,dtype).reshape(depth.height,depth.step//dtype.itemsize)
        patch = a[top:bottom,left:right].astype(float)
        if depth.encoding == '16UC1':
            patch *= .001
        valid = np.isfinite(patch) & (patch>=min_depth) & (patch<=max_depth)
        if valid.sum() < 25 or valid.mean() < .4:
            return None
        # Narrow legs leave background between them; choose a foreground cluster.
        z = float(np.quantile(patch[valid],.20))
        near = valid & (abs(patch-z)<.10)
        if near.sum() < 25 or near.mean() < .10:
            return None
        z = float(np.median(patch[near]))
        yy, xx = np.where(near)
        u, v = float(np.median(xx)+left), float(np.median(yy)+top)
        return {'x':(u-cx)*z/fx,'y':(v-cy)*z/fy,'z':z}
    except (ValueError,TypeError,IndexError,OverflowError):
        return None


def seconds(stamp):
    return stamp.sec + stamp.nanosec/1e9


def perception_result(rgb, detections, depths, info, target_class='rescue_dummy',
                      confidence=.5, sync_tolerance=.10, min_depth=.15, max_depth=3.):
    """Filter class; associate nearest aligned depth; select highest-confidence valid point."""
    targets=[]
    for detection in detections:
        try:
            score=float(detection['confidence'])
            if detection['class_name']==target_class and math.isfinite(score) and score>=confidence:
                bbox=list(map(float,detection['xyxy']))
                if len(bbox)!=4 or not all(math.isfinite(v) for v in bbox):continue
                targets.append(dict(detection,confidence=score,xyxy=bbox))
        except (KeyError,TypeError,ValueError):
            continue
    aligned = [d for d in depths if d.header.frame_id==rgb.header.frame_id and
               abs(seconds(d.header.stamp)-seconds(rgb.header.stamp))<=sync_tolerance]
    calibrated = (info is not None and bool(rgb.header.frame_id) and
                  info.header.frame_id==rgb.header.frame_id and
                  (info.width,info.height)==(rgb.width,rgb.height))
    if aligned and calibrated:
        depth=min(aligned,key=lambda d:abs(seconds(d.header.stamp)-seconds(rgb.header.stamp)))
        for target in targets:
            point=depth_point(depth,target['xyxy'],info,min_depth,max_depth)
            if point is not None:
                target['position_camera']=point
    valid=[(i,t) for i,t in enumerate(targets) if 'position_camera' in t]
    selected=max(valid,key=lambda entry:entry[1]['confidence'])[0] if valid else None
    payload={'stamp':{'sec':rgb.header.stamp.sec,'nanosec':rgb.header.stamp.nanosec},
             'frame_id':rgb.header.frame_id,'detected':bool(targets),
             'position_valid':selected is not None,'selected_index':selected,'detections':targets}
    point=targets[selected]['position_camera'] if selected is not None else None
    return payload,point
