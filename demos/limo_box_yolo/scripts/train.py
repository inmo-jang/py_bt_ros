from pathlib import Path
import json,torch,shutil,yaml
from ultralytics import YOLO
ROOT=Path(__file__).resolve().parents[1]
config=yaml.safe_load((ROOT/'data.yaml').read_text())
config['path']=str(ROOT/'dataset')
DATA=ROOT/'data.local.yaml';DATA.write_text(yaml.safe_dump(config,sort_keys=False))
torch.set_num_threads(4)
model=YOLO(str(ROOT/'models/yolo11n.pt'))
model.train(data=str(DATA),epochs=18,imgsz=320,batch=16,
 device='cpu',workers=0,freeze=10,optimizer='AdamW',lr0=.002,
 project=str(ROOT/'runs'),name='box_demo',exist_ok=True,
 seed=41,patience=6,plots=True,cache=False,amp=False,
 degrees=8,translate=.1,scale=.3,fliplr=.5,mosaic=.5,close_mosaic=5)
model=YOLO(str(ROOT/'runs/box_demo/weights/best.pt'))
metrics=model.val(data=str(DATA),split='test',imgsz=320,device='cpu',workers=0,plots=True,project=str(ROOT/'runs'),name='heldout_test')
(ROOT/'reports/test_metrics.json').write_text(json.dumps(metrics.results_dict,indent=2))

shutil.copy2(ROOT/'runs/box_demo/weights/best.pt',ROOT/'models/best.pt')
