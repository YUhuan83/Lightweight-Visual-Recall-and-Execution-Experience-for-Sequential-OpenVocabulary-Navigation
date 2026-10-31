"""Replay saved camera frames with the deployed local image/text model."""
import json
import sys
from pathlib import Path
import cv2
import numpy as np
import torch
from omegaconf import OmegaConf
from explorfm import ExploRFMInference
from visual_navigation.utils.local_models import model_paths
from visual_navigation.utils.object_search_utils import localize_query

config = OmegaConf.load(sys.argv[1])
root, version, adaptor = model_paths(config)
model = ExploRFMInference(root/config.frontier_ckpt, root/config.traversability_ckpt,
    model_version=version, adaptor_version=config.adaptor_version,
    adaptor_ckpt_path=adaptor, use_naclip=config.use_naclip,
    use_summary_for_spatial=config.use_summary_for_spatial,
    radio_dim=config.radio_dim, static_scale_factor=config.static_scale_factor,
    model_precision=config.model_precision)
labels = ['traffic light', 'stop sign', 'gas station sign', 'tree',
          'street lamp', 'road sign', 'building', 'sky']
text = model.forward_on_text(labels)
results = []
for filename in sys.argv[2:]:
    rgb = cv2.cvtColor(cv2.imread(filename), cv2.COLOR_BGR2RGB)
    with torch.inference_mode():
        _, _, spatial = model.forward_on_numpy(rgb)
        spatial = spatial/spatial.norm(dim=1, keepdim=True).clamp_min(1e-6)
        similarities = torch.einsum('qc,bchw->bqhw', text.to(spatial.dtype), spatial).float().cpu().numpy()[0]
    target = similarities[0]
    negative = similarities[1:].max(axis=0)
    positive = target > config.object_search_config.mask_threshold
    _, checked_mask = localize_query(
        text_feats=text[:1], spatial_feats=spatial,
        orig_img_shape=rgb.shape[:2],
        pixel_level_seg=config.object_search_config.pixel_level_seg,
        mask_threshold=config.object_search_config.mask_threshold,
        negative_text_feats=text[1:], minimum_competitive_cells=5)
    result = {'file':filename, 'feature_shape': list(target.shape), 'labels':labels,
              'maxima':similarities.max(axis=(1,2)).tolist(),
              'target_cells_above_thresholds':{str(t):int((target>t).sum()) for t in (.03,.05,.08,.10,.15,.20)},
              'target_wins_above_thresholds':{str(t):int(((target>t)&(target>negative)).sum()) for t in (.03,.05,.08,.10,.15,.20)},
              'deployed_competitive_mask_pixels':int(checked_mask.sum()),
              'strongest_competitor_by_cell':int((similarities[1:].max(axis=0)>target).sum())}
    np.savez(str(Path(filename).with_suffix(''))+'_semantic.npz', similarities=similarities)
    results.append(result)
print('SEMANTIC_AUDIT='+json.dumps(results), flush=True)
