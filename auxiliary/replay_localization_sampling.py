"""Run unchanged production geometry on matched raw scans; prove old scan reconstruction first."""
from pathlib import Path
import sys

source=Path(__file__).with_name('replay_v46_localization.py').read_text()
source=source.replace('import ast,collections,hashlib,json,sqlite3,sys',
    'import ast,collections,hashlib,json,sqlite3,sys\nfrom beamng_wildos_adapter.pointcloud import decode_xyz,limit_points')
source=source.replace("    actual={d['observation_stamp_ns']", """    mapping={d['mapped_stamp_ns']:d['source_stamp_ns'] for d in timings
        if d.get('sensor')=='lidar' and d.get('status')=='published'}
    sources={mapping[s] for s in clouds}
    raw_clouds={ns(m):m for _,m in messages('/vehicles/ego/sensors/front_lidar') if ns(m) in sources}
    reconstruction=[]
    actual={d['observation_stamp_ns']""")
source=source.replace("        n.lidar_tf_frame=cloud.header.frame_id;n.get_clock", """        tf=n.tf_buffer.lookup_transform('spot1/odom',cloud.header.frame_id,rclpy.time.Time.from_msg(cloud.header.stamp))
        q=tf.transform.rotation;p=tf.transform.translation
        rotation=R.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
        origin=np.asarray([p.x,p.y,p.z])
        world=decode_xyz(raw_clouds[mapping[ns(cloud)]])
        sampled=(limit_points(world,30000)-origin)@rotation
        sampled=sampled[np.linalg.norm(sampled,axis=1)<=40.]
        original=decode_xyz(cloud)
        assert sampled.shape==original.shape,(sampled.shape,original.shape)
        error=float(np.max(np.abs(sampled-original))) if len(sampled) else 0.
        assert error<0.0002,error
        accepted_world=world[np.linalg.norm(world-origin,axis=1)<=40.]
        dense=(limit_points(accepted_world,30000)-origin)@rotation
        if sys.argv[1]=='dense':dense=(accepted_world-origin)@rotation
        reconstruction.append(dict(stamp_ns=ns(cloud),error_m=error,sampled=len(sampled),dense=len(dense),raw_points=len(world),valid_before_sampling=len(accepted_world)))
        cloud=deepcopy(cloud);cloud.height=1;cloud.width=len(dense);cloud.point_step=12;cloud.row_step=12*len(dense)
        cloud.data=np.asarray(dense,dtype='<f4').tobytes()
        n.lidar_tf_frame=cloud.header.frame_id;n.get_clock""")
source=source.replace("reports.append(dict(trial=trial.name,mask_count", "reports.append(dict(reconstruction=reconstruction,trial=trial.name,mask_count")
source=source.replace('localization_failure_replay.json',f'localization_sampling_{sys.argv[1]}.json')
exec(compile(source,'matched_raw_density_replay','exec'))
