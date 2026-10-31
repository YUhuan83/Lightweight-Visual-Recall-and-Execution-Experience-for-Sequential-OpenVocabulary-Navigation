"""Spool compressed frames temporarily; create MP4 only for failed trials.

Camera callbacks use exact ROS source nanoseconds. Screen frames record pipe
receipt time, not a claimed ROS or camera acquisition timestamp.
"""
from dataclasses import dataclass, asdict
import json
import math
from pathlib import Path
import signal
import subprocess
import threading
import time
import cv2
import numpy as np

MAX_RECORDING_BYTES = 2 * 1024 * 1024 * 1024

@dataclass(frozen=True)
class Frame:
    offset: int
    size: int
    elapsed: float
    source_stamp_ns: int | None


class JpegSpool:
    def __init__(self, directory, name, origin, *, max_bytes=MAX_RECORDING_BYTES, max_frames=6000):
        if name not in ('front_camera', 'rviz_live'):
            raise ValueError('Unknown owned recording stream')
        self.path = Path(directory)/f'{name}.frames.tmp'
        self.name, self.origin, self.max_bytes = name, origin, max_bytes
        self.max_frames = max_frames
        self.file = self.path.open('xb')
        self.frames = []
        self.shape = None
        self.closed = False

    def add_encoded(self, data, *, elapsed=None, source_stamp_ns=None):
        if self.closed:
            raise RuntimeError('Recording already closed')
        if len(self.frames)>=self.max_frames:
            raise RuntimeError('Temporary recording frame bound reached')
        if not data.startswith(b'\xff\xd8') or not data.endswith(b'\xff\xd9'):
            raise ValueError('Expected one complete encoded JPEG')
        elapsed = time.monotonic()-self.origin if elapsed is None else elapsed
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError('Invalid frame elapsed time')
        if self.frames and elapsed < self.frames[-1].elapsed:
            raise ValueError('Frame receipt time went backwards')
        if source_stamp_ns is not None and (type(source_stamp_ns) is not int or source_stamp_ns < 0):
            raise ValueError('Source timestamp must be integer nanoseconds')
        offset = self.file.tell()
        if offset+len(data)>self.max_bytes:
            raise RuntimeError('Temporary recording size bound reached')
        self.file.write(data)
        self.frames.append(Frame(offset, len(data), elapsed, source_stamp_ns))

    def add(self, image, source_stamp_ns=None, *, elapsed=None):
        ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            raise RuntimeError('Cannot compress recording frame')
        self.add_encoded(encoded.tobytes(), elapsed=elapsed, source_stamp_ns=source_stamp_ns)

    def finish(self, failed, *, fps, end_elapsed=None):
        self.file.close(); self.closed=True
        report = dict(stream=self.name, failed=failed, captured_frames=len(self.frames),
                      source_frames=[asdict(f) for f in self.frames], saved=False,
                      capture_clock='monotonic receipt', source_clock='ROS ns when present')
        if failed is None:
            report.update(review_pending=True, temporary_path=str(self.path),
                          fps=fps, end_elapsed=end_elapsed)
            metadata=self.path.parent/f'{self.name}.capture.json'
            with metadata.open('x') as target:
                json.dump(report,target,indent=2)
            report['metadata_path']=str(metadata)
            return report
        if not failed:
            self.path.unlink()
            report['discarded_temporary_frames']=True
            return report
        if not self.frames:
            report['error']='No frames captured; MP4 unavailable'
            return report
        if not math.isfinite(fps) or fps<=0:
            raise ValueError('Positive finite export FPS required')
        start = self.frames[0].elapsed
        end = self.frames[-1].elapsed if end_elapsed is None else end_elapsed
        if not math.isfinite(end) or end<self.frames[-1].elapsed:
            raise ValueError('Export end precedes captured frames')
        output=self.path.parent/f'{self.name}.mp4'
        partial=self.path.parent/f'{self.name}.partial.mp4'
        if output.exists() or partial.exists():
            raise FileExistsError('Refusing to overwrite prior recording')
        writer=None; index=[]
        try:
            with self.path.open('rb') as raw:
                current=-1; decoded=None
                for slot in range(int(math.floor((end-start)*fps+1e-7))+1):
                    when=start+slot/fps
                    selected=current
                    while selected+1<len(self.frames) and self.frames[selected+1].elapsed<=when+1e-9:
                        selected+=1
                    if selected!=current:
                        frame=self.frames[selected]; raw.seek(frame.offset)
                        decoded=cv2.imdecode(np.frombuffer(raw.read(frame.size),np.uint8),cv2.IMREAD_COLOR)
                        if decoded is None: raise RuntimeError('Corrupt temporary JPEG')
                        if writer is None:
                            self.shape=decoded.shape
                            writer=cv2.VideoWriter(str(partial),cv2.VideoWriter_fourcc(*'mp4v'),fps,
                                                   (decoded.shape[1],decoded.shape[0]))
                            if not writer.isOpened(): raise RuntimeError('Cannot create failure MP4')
                        elif decoded.shape!=self.shape:
                            raise RuntimeError('Recording resolution changed')
                        current=selected
                    writer.write(decoded)
                    index.append(dict(index=slot,pts=slot/fps,elapsed=when,
                                      source_stamp_ns=self.frames[current].source_stamp_ns))
            writer.release(); writer=None
            partial.replace(output)
            report.update(saved=True,path=str(output),video_start_elapsed=start,
                          video_end_elapsed=start+index[-1]['pts'],frame_index=index)
            self.path.unlink()
        except Exception as exc:
            report['error']=f'{type(exc).__name__}: {exc}'
            report['retained_temporary_frames']=str(self.path)
        finally:
            if writer is not None: writer.release()
        return report

    @classmethod
    def load_pending(cls, metadata):
        """Reopen our sealed spool after visual review; never recapture video."""
        metadata=Path(metadata)
        info=json.loads(metadata.read_text())
        name=info['stream']
        if name not in ('front_camera','rviz_live') or info.get('review_pending') is not True:
            raise ValueError('Not an owned pending capture')
        path=metadata.parent/f'{name}.frames.tmp'
        if Path(info['temporary_path']).resolve()!=path.resolve():
            raise ValueError('Pending capture path differs from its directory')
        frames=[Frame(**f) for f in info['source_frames']]
        size=path.stat().st_size; expected=0; previous=-1.
        if len(frames)>6000 or size>MAX_RECORDING_BYTES:
            raise ValueError('Pending capture exceeds recording bounds')
        for f in frames:
            if (type(f.offset) is not int or type(f.size) is not int
                    or f.offset!=expected or f.size<=0 or f.offset+f.size>size
                    or not math.isfinite(f.elapsed) or f.elapsed<previous
                    or f.elapsed<0 or (f.source_stamp_ns is not None
                    and (type(f.source_stamp_ns) is not int or f.source_stamp_ns<0))):
                raise ValueError('Invalid pending capture frame index')
            expected+=f.size; previous=f.elapsed
        if expected!=size:
            raise ValueError('Pending capture has unindexed bytes')
        obj=cls.__new__(cls)
        obj.path=path; obj.name=name; obj.frames=frames; obj.shape=None
        obj.closed=False; obj.file=path.open('rb')
        return obj, info


class ScreenCapture:
    """Owned ffmpeg pipe and reader; no remote GUI or process lifecycle changes."""
    def __init__(self, spool, display, *, fps=8):
        self.spool=spool; self.error=None; self.dropped_after_error=0
        self.log=(spool.path.parent/'screen_capture.log').open('x')
        command=['ffmpeg','-nostdin','-hide_banner','-loglevel','warning',
            '-f','x11grab','-framerate',str(fps),'-video_size','1280x1024','-i',display,
            '-threads','1','-c:v','mjpeg','-q:v','5',
            '-progress',str(spool.path.parent/'screen_ffmpeg_progress.log'),
            '-f','image2pipe','pipe:1']
        try:
            self.process=subprocess.Popen(command,stdin=subprocess.DEVNULL,
                                          stdout=subprocess.PIPE,stderr=self.log)
        except Exception:
            self.log.close()
            raise
        (spool.path.parent/'screen_capture.pid').write_text(str(self.process.pid))
        self.reader=threading.Thread(target=self.read,name='rviz-jpeg-receipts',daemon=True)
        self.reader.start()

    def read(self):
        pending=bytearray()
        try:
            while True:
                chunk=self.process.stdout.read(16384)
                if not chunk: break
                pending.extend(chunk)
                while True:
                    start=pending.find(b'\xff\xd8'); end=pending.find(b'\xff\xd9',start+2)
                    if start<0 or end<0: break
                    if self.error is None:
                        try:
                            self.spool.add_encoded(bytes(pending[start:end+2]))
                        except Exception as exc:
                            self.error=f'{type(exc).__name__}: {exc}'
                            self.dropped_after_error+=1
                    else:
                        self.dropped_after_error+=1
                    # Keep draining after a bound/IO failure. Otherwise ffmpeg
                    # blocks on its pipe and cannot finish on SIGINT/SIGTERM.
                    del pending[:end+2]
                if len(pending)>8*1024*1024: raise RuntimeError('Invalid screen JPEG framing')
        except Exception as exc:
            self.error=f'{type(exc).__name__}: {exc}'

    def stop(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
        try: self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try: self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill(); self.process.wait(timeout=5)
        self.reader.join(timeout=3)
        if self.reader.is_alive(): raise RuntimeError('Screen reader did not stop')
        self.process.stdout.close(); self.log.close()
