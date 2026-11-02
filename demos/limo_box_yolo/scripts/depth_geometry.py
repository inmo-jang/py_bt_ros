"""RGB-aligned depth sampling and rigid point transforms, independent of YOLO."""
import math
import numpy as np


def sample_box_depth(depth, bbox, info):
    if depth.encoding not in ('16UC1','32FC1'):return None
    if depth.width!=info.width or depth.height!=info.height:return None
    x0,y0,x1,y1=map(float,bbox)
    if x1-x0<16 or y1-y0<16:return None
    # Use the central patch; borders can include floor/background or another object.
    left=max(0,int(x0+.35*(x1-x0)));right=min(depth.width,int(x0+.65*(x1-x0)))
    top=max(0,int(y0+.35*(y1-y0)));bottom=min(depth.height,int(y0+.65*(y1-y0)))
    if right<=left or bottom<=top:return None
    dtype=np.dtype(('>' if depth.is_bigendian else '<')+('u2' if depth.encoding=='16UC1' else 'f4'))
    bpp=dtype.itemsize
    a=np.frombuffer(depth.data,dtype).reshape(depth.height,depth.step//bpp)[:,:depth.width]
    patch=a[top:bottom,left:right].astype(float)
    if depth.encoding=='16UC1':patch*=.001
    valid=np.isfinite(patch)&(patch>=.15)&(patch<=3.)
    if valid.sum()<25 or valid.mean()<.4:return None
    z=float(np.median(patch[valid]));near=valid&(abs(patch-z)<.05)
    yy,xx=np.where(near)
    if not len(xx):return None
    u=float(np.median(xx)+left);v=float(np.median(yy)+top)
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    if fx<=0 or fy<=0:return None
    return {'x':(u-cx)*z/fx,'y':(v-cy)*z/fy,'z':z,'depth_m':z,'valid_fraction':float(valid.mean())}


def transform_xyz(transform, xyz):
    q=transform.rotation;t=transform.translation
    v=np.asarray(xyz,dtype=float);r=np.array([q.x,q.y,q.z]);norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
    if not math.isfinite(norm) or norm<1e-8:raise ValueError('Invalid transform quaternion')
    # q * v * q^-1; normalized formula handles minor TF quaternion round-off.
    rotated=v+(2/norm)*(q.w*np.cross(r,v)+np.cross(r,np.cross(r,v)))
    return (rotated+np.array([t.x,t.y,t.z])).tolist()
