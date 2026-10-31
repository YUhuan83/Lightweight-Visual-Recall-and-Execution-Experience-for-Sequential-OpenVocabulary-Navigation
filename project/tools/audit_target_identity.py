"""Compare query/class scores on unmodified saved RGB frames; never drive a vehicle."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
import torch
from omegaconf import OmegaConf
from explorfm import ExploRFMInference
from visual_navigation.utils.local_models import model_paths
from visual_navigation.utils.object_search_utils import localize_query

p = argparse.ArgumentParser()
p.add_argument('config')
p.add_argument('cases', help='JSON with labels and frames, optionally pixel ROIs')
p.add_argument('output')
args = p.parse_args()
config = OmegaConf.load(args.config)
cases = json.loads(Path(args.cases).read_text())
root, version, adaptor = model_paths(config)
model = ExploRFMInference(root/config.frontier_ckpt, root/config.traversability_ckpt,
    model_version=version, adaptor_version=config.adaptor_version,
    adaptor_ckpt_path=adaptor, use_naclip=config.use_naclip,
    use_summary_for_spatial=config.use_summary_for_spatial,
    radio_dim=config.radio_dim, static_scale_factor=config.static_scale_factor,
    model_precision=config.model_precision)
labels = cases['labels']
text = model.forward_on_text(labels)
out = Path(args.output)
out.mkdir(parents=True, exist_ok=True)
results = []
for case in cases['frames']:
    rgb = cv2.cvtColor(cv2.imread(case['path']), cv2.COLOR_BGR2RGB)
    with torch.inference_mode():
        _, _, spatial = model.forward_on_numpy(rgb)
        spatial = spatial/spatial.norm(dim=1, keepdim=True).clamp_min(1e-6)
        scores = torch.einsum('qc,bchw->bqhw', text.to(spatial.dtype), spatial).float().cpu().numpy()[0]
        masks = {}
        for name, minimum in [('legacy', 0), ('checked', 5)]:
            _, mask = localize_query(text[:1], spatial, rgb.shape[:2],
                config.object_search_config.pixel_level_seg,
                config.object_search_config.mask_threshold,
                minimum_component_cells=minimum)
            masks[name] = mask[0, 0] > 0
        overlay = rgb.copy()
        overlay[masks['checked']] = (0.5*overlay[masks['checked']] + [0, 127.5, 0]).astype(np.uint8)
        cv2.imwrite(str(out/(case['name']+'_checked.png')), cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR))
    fh, fw = scores.shape[1:]
    regions = {'scene': [0, 0, rgb.shape[1], rgb.shape[0]], **case.get('regions', {})}
    summary = {}
    for name, (x0, y0, x1, y1) in regions.items():
        sx0, sy0 = int(x0*fw/rgb.shape[1]), int(y0*fh/rgb.shape[0])
        sx1 = min(fw, max(sx0+1, int(np.ceil(x1*fw/rgb.shape[1]))))
        sy1 = min(fh, max(sy0+1, int(np.ceil(y1*fh/rgb.shape[0]))))
        cells = scores[:, sy0:sy1, sx0:sx1]
        summary[name] = {'maxima': dict(zip(labels, cells.max(axis=(1,2)).tolist())),
                         'means': dict(zip(labels, cells.mean(axis=(1,2)).tolist())),
                         'target_cells_above_threshold': int((cells[0] > config.object_search_config.mask_threshold).sum()),
                         'target_winning_cells': int(((cells[0] > config.object_search_config.mask_threshold) &
                             (cells[0] > cells[1:].max(axis=0))).sum())}
    np.savez_compressed(out/(case['name']+'.npz'), scores=scores)
    results.append({'name': case['name'], 'feature_shape': list(scores.shape), 'regions': summary,
                    'legacy_mask_pixels': int(masks['legacy'].sum()),
                    'checked_mask_pixels': int(masks['checked'].sum()),
                    'checked_bbox': list(cv2.boundingRect(masks['checked'].astype(np.uint8)))})
report = {'labels': labels, 'threshold': float(config.object_search_config.mask_threshold), 'results': results}
(out/'report.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report), flush=True)
