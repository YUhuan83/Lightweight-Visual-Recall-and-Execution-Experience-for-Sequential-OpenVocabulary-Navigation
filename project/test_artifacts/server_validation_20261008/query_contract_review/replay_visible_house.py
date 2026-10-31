import cv2,json,pathlib,numpy as np,torch
from omegaconf import OmegaConf
from torchvision import transforms
from explorfm import ExploRFMInference
from visual_navigation.utils.local_models import model_paths
from visual_navigation.utils.object_search_utils import localize_query
r=pathlib.Path('/home/ly/town_memory_validation_20261008');out=r/'trial3_visible_house_replay';out.mkdir(exist_ok=True)
cfg=OmegaConf.load('/home/ly/wildos_ab_20261008_B_fix/wildos_port/repro_check/src/nebula2-wildos/visual_navigation/configs/beamng_ablation.yaml')
ck,ver,ad=model_paths(cfg)
model=ExploRFMInference(frontier_ckpt=ck/cfg.frontier_ckpt,traversability_ckpt=ck/cfg.traversability_ckpt,model_version=ver,adaptor_version=cfg.adaptor_version,adaptor_ckpt_path=ad,use_naclip=cfg.use_naclip,use_summary_for_spatial=cfg.use_summary_for_spatial,radio_dim=cfg.radio_dim,static_scale_factor=cfg.static_scale_factor,model_precision=cfg.model_precision)
text=model.forward_on_text(['green residential house'])
c=cv2.VideoCapture(str(r/'B_green_house_station_trial3/front_camera.mp4'));results=[]
for t in [78,80,82,85]:
 c.set(cv2.CAP_PROP_POS_MSEC,t*1000);ok,bgr=c.read();assert ok
 cv2.imwrite(str(out/f'{t}s.png'),bgr)
 rgb=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
 with torch.inference_mode():
  _,_,feat=model.forward(transforms.ToTensor()(rgb).unsqueeze(0))
  if model.model_precision.is_fp16():feat=feat.half()
  norm=feat/feat.norm(dim=1,keepdim=True)
  sims=torch.einsum('qc,bchw->bqhw',text.to(dtype=norm.dtype),norm).float().cpu().numpy()[0,0]
  _,mask=localize_query(text,feat,rgb.shape[:2],False,float(cfg.object_search_config.mask_threshold),minimum_component_cells=5)
 raw=(sims>float(cfg.object_search_config.mask_threshold)).astype(np.uint8)
 n,labels,stats,cent=cv2.connectedComponentsWithStats(raw,8)
 d=dict(video_second=t,feature_shape=list(sims.shape),max_similarity=float(sims.max()),threshold=float(cfg.object_search_config.mask_threshold),raw_positive_cells=int(raw.sum()),component_sizes=stats[1:,cv2.CC_STAT_AREA].tolist(),filtered_pixels=int(mask.sum()),top_feature_xy=[int(np.argmax(sims)%sims.shape[1]),int(np.argmax(sims)//sims.shape[1])])
 results.append(d);print(json.dumps(d),flush=True)
 heat=cv2.applyColorMap(cv2.resize(np.clip(sims/0.15,0,1)*255,(bgr.shape[1],bgr.shape[0])).astype(np.uint8),cv2.COLORMAP_JET)
 cv2.imwrite(str(out/f'{t}s_similarity.jpg'),cv2.addWeighted(bgr,0.6,heat,0.4,0))
(out/'results.json').write_text(json.dumps(results,indent=2));c.release()
