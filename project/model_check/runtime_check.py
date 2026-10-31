"""Run isolated probes and an offline-only synthetic SigLIP smoke test."""
import datetime
import importlib.metadata
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import traceback

HERE = Path(__file__).resolve().parent
MODEL = Path(r'C:\Users\Yu\ckpts\siglip2_ms')
VENV = r'C:\Users\Yu\nebula2-wildos\.venv\Scripts\python.exe'


def now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def memory():
    import ctypes
    class Status(ctypes.Structure):
        _fields_ = [('length', ctypes.c_ulong), ('load', ctypes.c_ulong)] + [(name, ctypes.c_ulonglong) for name in ('total', 'available', 'total_page', 'available_page', 'total_virtual', 'available_virtual', 'extended')]
    status = Status()
    status.length = ctypes.sizeof(status)
    if not ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(status)):
        raise OSError('GlobalMemoryStatusEx failed')
    return {'total_ram_bytes': status.total, 'available_ram_bytes': status.available}


def child(mode):
    # Defense in depth: forbid socket networking, not merely Hub downloads.
    def deny(*args, **kwargs):
        raise RuntimeError('Network disabled by local-only smoke test')
    socket.socket.connect = deny
    socket.socket.connect_ex = deny
    socket.create_connection = deny
    result = {'started_utc': now(), 'mode': mode, 'executable': sys.executable,
              'python': sys.version, 'memory': memory(), 'packages': {},
              'offline': {key: os.environ.get(key) for key in ('HF_HUB_OFFLINE', 'TRANSFORMERS_OFFLINE', 'HF_HUB_DISABLE_TELEMETRY')},
              'inference_ran': False}
    start = time.monotonic()
    code = 0
    try:
        for package in ('torch', 'transformers', 'safetensors', 'Pillow', 'tokenizers', 'accelerate', 'torchvision'):
            try:
                result['packages'][package] = importlib.metadata.version(package)
            except importlib.metadata.PackageNotFoundError:
                result['packages'][package] = None
        import torch
        result['torch'] = {'version': torch.__version__, 'cuda_runtime': torch.version.cuda,
                           'cuda_available': torch.cuda.is_available(), 'architectures': torch.cuda.get_arch_list()}
        if torch.cuda.is_available():
            result['torch'].update(gpu=torch.cuda.get_device_name(0), capability=torch.cuda.get_device_capability(0), free_total_vram=list(torch.cuda.mem_get_info()))
        if mode == 'infer':
            from transformers import Siglip2Model, Siglip2Processor
            from PIL import Image, ImageDraw
            audit = json.loads((HERE / 'audit_results.json').read_text(encoding='utf-8'))
            checks = [c for c in audit['safetensors'] if c['path'] == str(MODEL / 'model.safetensors')]
            if not audit['passed'] or len(checks) != 1 or not checks[0]['valid']:
                raise RuntimeError('Checkpoint audit did not pass; refusing model load')
            if result['memory']['available_ram_bytes'] < 6 * 1024**3:
                raise RuntimeError('Less than 6 GiB available RAM; refusing large-model load')
            if not result['packages']['accelerate']:
                raise RuntimeError('accelerate is absent; low_cpu_mem_usage loading unavailable in this Transformers runtime')
            device = 'cuda' if torch.cuda.is_available() else 'cpu'
            torch.set_num_threads(min(8, os.cpu_count() or 1))
            result.update(model_path=str(MODEL), device=device, dtype='float16', batch_size=1,
                          input_description='Synthetic 224x224 RGB image: green background, gray trapezoid road, blue sky. Not a model-quality evaluation or simulator image.',
                          text='a gray road through green grass under a blue sky', stage='processor_load')
            processor = Siglip2Processor.from_pretrained(str(MODEL), local_files_only=True, trust_remote_code=False)
            result['stage'] = 'model_load'
            load_start = time.monotonic()
            model, loading = Siglip2Model.from_pretrained(str(MODEL), local_files_only=True,
                trust_remote_code=False, use_safetensors=True, torch_dtype=torch.float16,
                low_cpu_mem_usage=True, output_loading_info=True)
            result['loading_info'] = loading
            if any(loading.get(key) for key in ('missing_keys', 'unexpected_keys', 'mismatched_keys', 'error_msgs')):
                raise RuntimeError('Non-exact checkpoint load; see loading_info')
            model = model.eval().to(device)
            result['load_seconds'] = time.monotonic() - load_start
            result['parameter_count'] = sum(p.numel() for p in model.parameters())
            result['parameter_dtypes'] = sorted({str(p.dtype) for p in model.parameters()})
            image = Image.new('RGB', (224, 224), (40, 130, 50))
            draw = ImageDraw.Draw(image)
            draw.rectangle((0, 0, 223, 74), fill=(80, 160, 230))
            draw.polygon([(96, 75), (128, 75), (208, 223), (16, 223)], fill=(110, 110, 110))
            image.save(HERE / 'synthetic_input.png')
            inputs = processor(text=[result['text']], images=[image], padding='max_length', max_length=64, truncation=True, return_tensors='pt')
            inputs = {key: value.to(device=device, dtype=torch.float16) if value.is_floating_point() else value.to(device) for key, value in inputs.items()}
            result['input_shapes'] = {key: list(value.shape) for key, value in inputs.items()}
            result['stage'] = 'forward'
            forward_start = time.monotonic()
            with torch.inference_mode():
                outputs = model(**inputs)
            if device == 'cuda':
                torch.cuda.synchronize()
            result['forward_seconds'] = time.monotonic() - forward_start
            result['inference_ran'] = True
            result['output'] = {}
            for key in ('image_embeds', 'text_embeds', 'logits_per_image', 'logits_per_text'):
                tensor = getattr(outputs, key).detach().float().cpu()
                result['output'][key] = {'shape': list(tensor.shape), 'all_finite': bool(torch.isfinite(tensor).all()), 'norm': tensor.norm().item()}
                if tensor.numel() <= 4:
                    result['output'][key]['values'] = tensor.tolist()
            result['output']['cosine_similarity'] = torch.nn.functional.cosine_similarity(outputs.image_embeds.float(), outputs.text_embeds.float()).item()
            if not all(v['all_finite'] for v in result['output'].values() if isinstance(v, dict)):
                raise RuntimeError('Nonfinite model output')
            result['stage'] = 'complete'
        result['status'] = 'passed'
    except Exception as exc:
        result.update(status='blocked', error_type=type(exc).__name__, error=str(exc), traceback=traceback.format_exc())
        code = 1
    result.update(finished_utc=now(), elapsed_seconds=time.monotonic() - start)
    print(json.dumps(result, indent=2), flush=True)
    return code


def main():
    if len(sys.argv) > 1:
        return child(sys.argv[1])
    env = os.environ.copy()
    env.update(HF_HUB_OFFLINE='1', TRANSFORMERS_OFFLINE='1', HF_HUB_DISABLE_TELEMETRY='1',
               HF_HOME=str(HERE / 'offline_cache'), TORCH_HOME=str(HERE / 'offline_cache'),
               PYTHONDONTWRITEBYTECODE='1', TOKENIZERS_PARALLELISM='false', DO_NOT_TRACK='1')
    report = {'started_utc': now(), 'commands': [], 'policy': 'No installation, downloads, pickle deserialization, or original source scripts. All outputs confined to model_check.'}
    commands = [('gpu', ['nvidia-smi'], 30),
                ('current_python', [sys.executable, '-I', '-B', str(Path(__file__).resolve()), 'probe'], 60),
                ('existing_venv', [VENV, '-I', '-B', str(Path(__file__).resolve()), 'probe'], 60),
                ('siglip_inference', [VENV, '-I', '-B', str(Path(__file__).resolve()), 'infer'], 300)]
    for name, command, timeout in commands:
        entry = {'name': name, 'command': command, 'started_utc': now(), 'timeout_seconds': timeout}
        print('Running ' + name, flush=True)
        try:
            process = subprocess.run(command, env=env, cwd=str(HERE), capture_output=True, text=True, encoding='utf-8', errors='replace', timeout=timeout)
            entry.update(returncode=process.returncode, stdout=process.stdout, stderr=process.stderr)
            try:
                entry['result'] = json.loads(process.stdout)
            except json.JSONDecodeError:
                pass
        except subprocess.TimeoutExpired as exc:
            entry.update(error='TimeoutExpired; child killed and reaped by subprocess.run', stdout=str(exc.stdout), stderr=str(exc.stderr))
        except Exception as exc:
            entry.update(error=f'{type(exc).__name__}: {exc}')
        entry['finished_utc'] = now()
        report['commands'].append(entry)
        report['finished_utc'] = now()
        (HERE / 'runtime_results.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(entry, indent=2), flush=True)
    return 0 if report['commands'][-1].get('result', {}).get('status') == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
