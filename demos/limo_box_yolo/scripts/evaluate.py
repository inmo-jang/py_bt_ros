"""Evaluate thresholded detection on held-out camera poses, including negatives."""
import json
from pathlib import Path
import numpy as np
import torch
from ultralytics import YOLO
ROOT=Path(__file__).resolve().parents[1];torch.set_num_threads(4)
model=YOLO(str(ROOT/'models/best.pt'))
files=sorted((ROOT/'dataset/images/test').glob('*.jpg'))
results=model.predict([str(f) for f in files],imgsz=320,conf=.5,device='cpu',verbose=False,stream=True)
tp=fp=fn=positives=negative_fp=0;rows=[]
for f,r in zip(files,results):
 label=(ROOT/'dataset/labels/test'/f.with_suffix('.txt').name).read_text().splitlines();matched=False
 if label:
  positives+=1;_,x,y,w,h=map(float,label[0].split());gt=np.array([(x-w/2)*640,(y-h/2)*480,(x+w/2)*640,(y+h/2)*480])
  for b in r.boxes:
   p=b.xyxy[0].cpu().numpy();inter=np.prod(np.maximum(0,np.minimum(gt[2:],p[2:])-np.maximum(gt[:2],p[:2])));union=np.prod(gt[2:]-gt[:2])+np.prod(p[2:]-p[:2])-inter
   if not matched and inter/max(union,1)>=.5:matched=True;tp+=1
   else:fp+=1
  if not matched:fn+=1
 else:
  fp+=len(r.boxes);negative_fp+=bool(len(r.boxes))
 rows.append(dict(image=f.name,detections=len(r.boxes),matched=matched,confidence=[float(b.conf.item()) for b in r.boxes]))
 if f.name in ['test_0000.jpg','test_0007.jpg','test_0019.jpg','test_0030.jpg','test_0040.jpg','test_0069.jpg']:
  out=ROOT/'reports/evidence';out.mkdir(exist_ok=True);r.save(filename=str(out/f.name))
report={'confidence_threshold':.5,'iou_threshold':.5,'positive_images':positives,'negative_images':len(files)-positives,'true_positive_boxes':tp,'false_positive_boxes':fp,'false_negative_boxes':fn,'precision':tp/max(tp+fp,1),'recall':tp/max(tp+fn,1),'negative_images_with_false_detection':negative_fp,'images':rows}
(ROOT/'reports/threshold_test.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='images'},indent=2))
