# Local Checkpoint Audit

Executed on 2026-09-09. All created files are under
`C:\Users\Yu\Desktop\拟真\model_check`. Original source, environments,
dependencies, and weights were not edited. No server precheck was run.

## Actual Results

- `audit_results.json`: passed. SHA256 streamed over 18 files, including RADIO,
  trav/frontier heads, SigLIP weights, configuration, and tokenizer assets.
- `siglip2_ms/model.safetensors`: 4,542,792,928 bytes; 888 F32 tensors;
  1,135,670,962 elements. Header: 109,072 bytes. Payload: 4,542,683,848 bytes.
  Every tensor's shape/dtype byte count and offsets were checked against the
  actual file length. No overlap, gap, out-of-bounds range, or trailing payload.
- JSON assets parsed successfully with duplicate-key rejection. This is syntax
  and inventory validation, not proof of processor/model semantic compatibility.
- `siglip2/` is an incomplete Hugging Face cache: its inspected snapshot contains
  tokenizer.json and tokenizer.model, but no model weights or model config.
  Both tokenizer hashes match the corresponding files in `siglip2_ms/`.
- RADIO and the two heads were hashed only, not deserialized or structurally
  validated. No pickle loading or `torch.load` was performed.
- Optional ONNX, incomplete downloads, and cache bookkeeping were skipped.
- Hashes are measured local fingerprints, not verified publisher signatures.

## Runtime And Blocker

`runtime_results.json` records actual executable commands, UTC timestamps,
exit codes, stdout, stderr, package versions, GPU/RAM metadata, and traceback.

GPU: NVIDIA GeForce RTX 5060 Laptop GPU, 8,151 MiB reported by nvidia-smi,
driver 610.88, compute capability 12.0.

| Runtime | PyTorch | CUDA | Transformers | Other blocker |
| --- | --- | --- | --- | --- |
| `D:\python.exe` | 2.11.0+cu128 | Available, CUDA 12.8, sm_120 supported | Absent | tokenizers absent |
| `C:\Users\Yu\nebula2-wildos\.venv\Scripts\python.exe` | 2.14.0+cpu | Unavailable | 4.51.3 | accelerate absent |

The offline inference child successfully imported Siglip2Model and
Siglip2Processor, but deliberately stopped before processor/weight loading:

```text
RuntimeError: Less than 6 GiB available RAM; refusing large-model load
```

Available RAM at inference-child startup was 2,007,314,432 bytes (1.87 GiB).
The 6 GiB threshold is this script's conservative safety guard, not a measured
minimum requirement of SigLIP. The selected low_cpu_mem_usage loader would
also need accelerate, which is absent in the existing venv.

**Inference did not run.** No embeddings, logits, synthetic image artifact,
model-quality result, or BeamNG image result was produced. ExploRFM was not run.
The script's unreached inference path uses a synthetic image and one caption,
batch size 1, FP16, safetensors-only loading, and checks for exact loading and
finite outputs. No FP32 model copy was instantiated in this run.

Offline child processes use `HF_HUB_OFFLINE=1`, `TRANSFORMERS_OFFLINE=1`,
telemetry disabled, and socket-connection blocking. Load calls explicitly set
`local_files_only=True` and `trust_remote_code=False`. Bytecode writes are
disabled with `-B`; intended cache/output locations are confined here.
No original application scripts were invoked. No installation was attempted.

## Measured SHA256

Paths below are relative to `C:\Users\Yu\ckpts`.

| File | Bytes | SHA256 |
| --- | ---: | --- |
| `c-radio_v3-b_half.pth.tar` | 546632054 | `530b0e827b1d409ca3b175466e11062885baedd823ec5dae3a88348e76d4c398` |
| `trav_head.ckpt` | 39523061 | `462e545c7e528795a0b2728d8223cfbbd71ff284757cd479a2a72f7f00b9cad6` |
| `frontier_head.ckpt` | 39524786 | `5c6d3c91d3e4edb5408b770674d87cbfa769c6123d8b071a8cdecd08179c6461` |
| `siglip2_ms/model.safetensors` | 4542792928 | `11a61a2068800d5f4f35cb041c1fea25de86ce87725e4c97a9ed046d0b22c076` |

Full asset hashes and per-tensor validation data are in `audit_results.json`.

## Executed Entry Commands

Working directory: `C:\Users\Yu\Desktop\拟真`.

```powershell
python -I -B "C:\Users\Yu\Desktop\拟真\model_check\audit.py"
python -I -B "C:\Users\Yu\Desktop\拟真\model_check\runtime_check.py"
```

Audit exit code: 0. Runtime entry exit code: 1, reflecting the recorded inference
blocker. The runtime entry executes isolated current-Python and existing-venv
probes before the existing-venv inference child, with bounded timeouts.
