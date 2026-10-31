"""Read stationary smoke captures; write diagnostics only, never calibration."""
import argparse
import json
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw


def vector(row, keys):
    for key in keys:
        if key in row:
            value = row[key]
            if isinstance(value, dict):
                value = [value.get(k) for k in ('x','y','z')]
            try:
                arr = np.asarray(value,dtype=float)
                if arr.shape == (3,) and np.isfinite(arr).all():
                    return arr
            except (TypeError,ValueError):
                pass
    return None


def stats(values):
    arr = np.asarray(values,dtype=float)
    return dict(count=len(arr), minimum=float(arr.min()),median=float(np.median(arr)),
                maximum=float(arr.max()),mean=float(arr.mean())) if len(arr) else None


def analyze(directory):
    report = json.loads((directory/'metadata.json').read_text())
    result = {'source_status':report.get('status'), 'calibrated':False,
              'notes':['Point plot uses world-XYZ hypothesis and queried origin, not calibrated sensor-frame XYZ.',
                       'All times below are source IMU timestamps, not host polling frequency.',
                       'No inference, navigation or simulator changes performed.']}
    rows = []
    batch = []
    for path in sorted(directory.glob('imu_[0-9][0-9].json')):
        data = json.loads(path.read_text())
        if not isinstance(data,list):
            raise ValueError('Expected measurement-list IMU format from latest smoke script')
        times=[]
        for row in data:
            if not isinstance(row,dict):
                continue
            t=row.get('time')
            if isinstance(t,bool) or not isinstance(t,(int,float)) or not np.isfinite(t):
                continue
            a=vector(row,('accRaw','accSmooth'))
            w=vector(row,('angVel','angVelSmooth'))
            if a is not None and w is not None:
                rows.append((float(t),a,w))
                times.append(float(t))
        dt=np.diff(times)
        batch.append(dict(file=path.name,valid_records=len(times),nonpositive_steps=int((dt<=0).sum()),
                          positive_dt_s=stats(dt[dt>0])))
    if rows:
        times=np.array([r[0] for r in rows])
        # Preserve arrival order to expose resets and repeat batches.
        dt=np.diff(times)
        a=np.array([r[1] for r in rows]); w=np.array([r[2] for r in rows])
        result['imu']=dict(batches=batch, records=len(rows),time_start=float(times[0]),time_end=float(times[-1]),
            duplicate_steps=int((dt==0).sum()),backward_steps=int((dt<0).sum()),
            positive_dt_s=stats(dt[dt>0]),
            median_dt_rate_hz=float(1/np.median(dt[dt>0])) if np.any(dt>0) else None,
            acceleration_norm_m_s2=stats(np.linalg.norm(a,axis=1)),
            angular_speed_norm_rad_s=stats(np.linalg.norm(w,axis=1)),
            acceleration_mean_components=a.mean(axis=0).tolist(),
            angular_velocity_mean_components=w.mean(axis=0).tolist(),
            note='Norm near gravity is a diagnostic, not an axis/sign or covariance calibration.')
    samples=report.get('sensor_samples',[])
    candidate=next((s for s in reversed(samples) if s.get('lidar',{}).get('quality',{}).get('quality_pass')),None)
    if candidate:
        info=candidate['lidar']; path=directory/info['file']
        if path.resolve().parent != directory.resolve():
            raise ValueError('Cloud reference outside capture directory')
        points=np.load(path,allow_pickle=False)
        if points.ndim!=2 or points.shape[1]!=3:
            raise ValueError('Expected Nx3 point array')
        origin=np.array(info['quality']['position_before'],dtype=float)
        finite=np.isfinite(points).all(axis=1)
        local=points[finite].astype(float)-origin
        radius=62.0; size=800
        plotted=local[(np.abs(local[:,:2])<=radius).all(axis=1)]
        image=Image.new('RGB',(size,size),(20,25,30))
        draw=ImageDraw.Draw(image)
        for xyz in plotted:
            px=int((xyz[0]/(2*radius)+.5)*(size-1)); py=int((.5-xyz[1]/(2*radius))*(size-1))
            z=float(np.clip((xyz[2]+4)/10,0,1))
            draw.point((px,py),fill=(int(50+205*z),int(210-130*z),int(220-180*z)))
        draw.ellipse((395,395,405,405),outline='white',width=2)
        draw.text((10,10),'World-aligned XY / origin: queried LiDAR position / +X right, +Y up',fill='white')
        image.save(directory/'diagnostic_cloud_xy.png')
        result['lidar']=dict(source_file=path.name,finite_points=int(finite.sum()),
            plotted_points=len(plotted),range_m=stats(np.linalg.norm(local,axis=1)),
            z_relative_to_sensor_m=stats(local[:,2]),plot='diagnostic_cloud_xy.png')
    if (directory/'colour.png').is_file():
        image=Image.open(directory/'colour.png').convert('RGB')
        p=np.asarray(image); mask=(p>=250).all(axis=2)
        overlay=np.array(p,copy=True); overlay[mask]=[255,0,180]
        Image.fromarray(overlay).save(directory/'diagnostic_near_white_mask.png')
        result['image']=dict(near_white_fraction=float(mask.mean()),
            row_band_fractions=[float(b.mean()) for b in np.array_split(mask,6,axis=0)],
            mask_file='diagnostic_near_white_mask.png',
            note='Magenta highlights near-white pixels, not identified faults. Original colour.png untouched.')
    (directory/'diagnostic_report.json').write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture',type=Path)
    args=p.parse_args()
    print(json.dumps(analyze(args.capture.expanduser().resolve()),indent=2,allow_nan=False))
