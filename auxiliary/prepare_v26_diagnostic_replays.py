"""Reuse audited diagnostic logic with v26 and the verified odom TF bridge."""
from pathlib import Path
base=Path(__file__).resolve().parent
s=(base/'audit_recorded_lidar_depth.py').read_text(encoding='utf8')
s=s.replace('v25_fast_shift','v26_body_arrival')
old="""    transforms=[tf for _,m in messages('/tf') for tf in m.transforms]
    for tf in sorted(transforms,key=ns):buffer.set_transform(tf,'recorded')"""
new="""    from geometry_msgs.msg import TransformStamped
    recorded={ns(tf):tf for _,m in messages('/tf') for tf in m.transforms if tf.child_frame_id=='spot1/base_link'}
    exact_matches=[]
    for _,odom in messages('/spot1/odom'):
        tf=TransformStamped();tf.header=odom.header;tf.child_frame_id=odom.child_frame_id
        tf.transform.translation.x=odom.pose.pose.position.x
        tf.transform.translation.y=odom.pose.pose.position.y
        tf.transform.translation.z=odom.pose.pose.position.z
        tf.transform.rotation=odom.pose.pose.orientation
        if ns(odom) in recorded:
            a=tf.transform;b=recorded[ns(odom)].transform
            exact_matches.append(max(abs(getattr(getattr(a,p),k)-getattr(getattr(b,p),k)) for p,ks in [('translation','xyz'),('rotation','xyzw')] for k in ks))
        buffer.set_transform(tf,'verified_odom')
    assert exact_matches and max(exact_matches)==0.,exact_matches[:5]"""
assert s.count(old)==1;s=s.replace(old,new)
s=s.replace("foreground_return_lines=dict(","odom_tf_exact_matches=len(exact_matches),sync_counts=dict(collections.Counter(d['status']+':'+d.get('reason','') for d in timings if d.get('sensor')=='mask_lidar_sync')),foreground_return_lines=dict(")
s=s.replace("source_lines=source.read_text().splitlines()", "source_lines=source.read_text().splitlines()")
(base/'v26_lidar_depth_replay.py').write_text(s,encoding='utf8',newline='\n')
s=(base/'audit_bearing_geometry.py').read_text(encoding='utf8')
s=s.replace('v25_fast_shift','v26_body_arrival')
s=s.replace('first_lidar_proposal_inside_mask','first_recorded_proposal_inside_mask')
s=s.replace('all_views_include_lidar_anchor','all_views_include_recorded_anchor')
s=s.replace("Raw LiDAR not in this bag, so failed depth gates cannot be replayed.","Depth rejection gates are audited separately using raw paired clouds and verified odom transforms.")
(base/'v26_bearing_replay.py').write_text(s,encoding='utf8',newline='\n')
