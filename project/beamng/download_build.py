"""Four-worker ranged download. URL exists only in inherited environment."""
import concurrent.futures
import json
import os
from pathlib import Path
import shutil
import threading
import time
import urllib.request
import zipfile


def main():
    url = os.environ.pop('BEAMNG_PRIVATE_DOWNLOAD_URL')
    folder = Path(os.environ.pop('BEAMNG_DOWNLOAD_DIR'))
    folder.mkdir(parents=True, exist_ok=True)
    status = folder / 'download_status.json'
    partial = folder / 'BeamNG.tech.v0.39.4.0.zip.part'
    final = folder / 'BeamNG.tech.v0.39.4.0.zip'
    lock = threading.Lock()
    started = time.time()
    done = 0
    total = 0

    def report(state, **extra):
        data = dict(state=state, pid=os.getpid(), workers=4, downloaded_bytes=done,
                    total_bytes=total, elapsed_s=round(time.time()-started), **extra)
        temp = status.with_suffix('.tmp')
        temp.write_text(json.dumps(data, indent=2), encoding='utf-8')
        temp.replace(status)

    try:
        if partial.exists() or final.exists():
            raise RuntimeError('Target exists; refusing overwrite')
        report('probing')
        request = urllib.request.Request(url, headers={'Range':'bytes=0-0','Accept-Encoding':'identity'})
        with urllib.request.urlopen(request,timeout=60) as response:
            content_range = response.headers.get('Content-Range','')
            if response.status != 206 or not content_range.startswith('bytes 0-0/'):
                raise RuntimeError('Server did not honor byte ranges; parallel download refused')
            total = int(content_range.split('/')[-1])
            validator = response.headers.get('ETag') or response.headers.get('Last-Modified')
            response.read()
        if shutil.disk_usage(folder).free < total + 1024**3:
            raise RuntimeError('Not enough free disk space for archive plus 1 GiB margin')
        with partial.open('xb') as stream:
            stream.truncate(total)
        report('downloading')
        chunk_size = 16*1024*1024

        def chunk(start):
            nonlocal done
            end = min(total-1,start+chunk_size-1)
            for attempt in range(4):
                try:
                    headers={'Range':f'bytes={start}-{end}','Accept-Encoding':'identity'}
                    if validator:
                        headers['If-Range']=validator
                    with urllib.request.urlopen(urllib.request.Request(url,headers=headers),timeout=90) as response:
                        if response.status != 206 or response.headers.get('Content-Range') != f'bytes {start}-{end}/{total}':
                            raise RuntimeError('Range/remote version mismatch')
                        data=response.read(end-start+2)
                    if len(data) != end-start+1:
                        raise RuntimeError('Incomplete chunk')
                    with partial.open('r+b') as stream:
                        stream.seek(start)
                        stream.write(data)
                    with lock:
                        done += len(data)
                        report('downloading', percent=round(100*done/total,2))
                    return
                except Exception:
                    if attempt == 3:
                        raise RuntimeError(f'Chunk failed after retries at byte {start}') from None
                    time.sleep(2**attempt)
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
            for _ in pool.map(chunk,range(0,total,chunk_size)):
                pass
        report('verifying_crc')
        with zipfile.ZipFile(partial) as archive:
            for info in archive.infolist():
                if info.is_dir() or info.filename.lower().endswith(('.key','.pem')):
                    continue
                with archive.open(info) as stream:
                    while stream.read(8*1024*1024):
                        pass
            names=[name.lower().replace('\\','/') for name in archive.namelist()]
            linux=any(name.endswith('binlinux/beamng.tech.x64') for name in names)
            windows=any(name.endswith('bin64/beamng.tech.x64.exe') for name in names)
        partial.rename(final)
        report('complete', linux_binary_present=linux, windows_binary_present=windows,
               crc='passed_non_secret_entries', filename=final.name)
    except Exception as exc:
        # Never serialize urllib exceptions: they can contain the credential URL.
        safe = str(exc) if type(exc) is RuntimeError else type(exc).__name__
        report('failed', error=safe)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
