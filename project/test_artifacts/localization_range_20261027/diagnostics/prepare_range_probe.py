from pathlib import Path
s=Path('probe_static_lidar_coverage.py').read_text(encoding='utf-8-sig')
s=s.replace("root=base/'v47_depth_geometry'","root=base/'v48_range_candidate'").replace("out=base/'depth_coverage_probe_20261027'","out=base/'depth_range_probe_20261027'")
a=s.index("rep=json.loads(");b=s.index("config=json.loads(",a)
s=s[:a]+'''chosen=[]
for m in sorted(masks,key=area,reverse=True):
 if all(abs(stamp(m)-stamp(o))>8e9 for o in chosen):chosen.append(m)
 if len(chosen)==6:break
'''+s[b:]
s=s.replace('for lines in (16,32,64):','for lines,density,max_range in ((16,30.,40.),(16,30.,80.),(64,60.,80.)):')
s=s.replace("f'coverage_{index}_{run}_{lines}'","f'range_{index}_{run}_{lines}_{int(max_range)}'")
s=s.replace('max_distance=40.,density=30.','max_distance=max_range,density=density')
s=s.replace('sampled=limit_points(world,30000);sampled=sampled[np.linalg.norm(sampled-nav,axis=1)<=40.]','sampled=limit_points(world,30000);sampled=sampled[np.linalg.norm(sampled-nav,axis=1)<=max_range]')
s=s.replace('lines=lines,mask_pixels=', 'lines=lines,density=density,max_range_m=max_range,mask_pixels=')
s=s.replace("f'frame{index}_{lines}.npz'","f'frame{index}_{lines}_{int(max_range)}.npz'")
Path('probe_static_lidar_range.py').write_text(s,encoding='utf8',newline='\n')
s=Path('probe_static_lidar_coverage.sh').read_text(encoding='utf-8-sig').replace('probe_static_lidar_coverage.py','probe_static_lidar_range.py').replace('v47_depth_geometry','v48_range_candidate')
Path('probe_static_lidar_range.sh').write_text(s,encoding='utf8',newline='\n')
