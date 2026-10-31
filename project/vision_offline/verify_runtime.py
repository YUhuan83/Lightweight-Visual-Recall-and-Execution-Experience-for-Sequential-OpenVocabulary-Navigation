"""Run only on the Linux server. CPU smoke test by default; --cuda is opt-in."""
import argparse
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import sys


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cuda', action='store_true')
    args = p.parse_args()
    assert platform.system() == 'Linux' and platform.machine() == 'x86_64'
    assert sys.version_info[:2] == (3, 10)
    os.environ.update(HF_HUB_OFFLINE='1', TRANSFORMERS_OFFLINE='1')
    if not args.cuda:
        os.environ['CUDA_VISIBLE_DEVICES'] = ''
    manifest = json.loads((Path(__file__).parent / 'manifest.json').read_text())
    for item in manifest['wheels']:
        assert importlib.metadata.version(item['name']) == item['version'], item['name']
    for name in ('transformers', 'timm', 'einops', 'sentencepiece', 'psutil',
                 'safetensors', 'uvloop', 'PIL', 'cv2', 'triton'):
        importlib.import_module(name)
    import numpy as np
    import cv2
    import torch
    import torchvision
    assert torch.__version__ == '2.7.0+cu128'
    assert torchvision.__version__ == '0.22.0+cu128'
    assert torch.version.cuda == '12.8'
    a = np.zeros((8, 8), dtype=np.uint8)
    a[2:4, 2:4] = 1
    assert cv2.connectedComponents(a)[0] == 2
    assert torch.from_numpy(a).sum().item() == 4
    boxes = torch.tensor([[0., 0., 4., 4.], [0., 0., 4., 4.]])
    scores = torch.tensor([0.9, 0.8])
    assert torchvision.ops.nms(boxes, scores, 0.5).tolist() == [0]
    if args.cuda:
        assert torch.cuda.is_available(), 'CUDA unavailable'
        assert torch.cuda.get_device_capability() == (8, 9), 'Expected RTX 4090 class architecture'
        assert torchvision.ops.nms(boxes.cuda(), scores.cuda(), 0.5).cpu().tolist() == [0]
        x = torch.ones((32, 32), device='cuda')
        assert (x @ x).sum().item() == 32768
        torch.cuda.synchronize()
        print('CUDA device:', torch.cuda.get_device_name())
    print('Runtime smoke tests passed; CUDA tested:', args.cuda)


if __name__ == '__main__':
    main()
