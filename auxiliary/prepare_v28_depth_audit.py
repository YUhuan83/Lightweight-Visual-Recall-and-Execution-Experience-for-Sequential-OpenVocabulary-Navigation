from pathlib import Path
base=Path(r'C:\Users\Yu\Desktop\Beamng')
s=(base/'v26_lidar_depth_replay.py').read_text().replace('v26_body_arrival','v28_intent_handoff')
s=s.replace("row.update(result='lidar_localized',replayed_position=position.tolist(),quality=convert(quality))",
 "row.update(result='lidar_localized',replayed_position=position.tolist(),quality=convert(quality))\n            pixels=points@k.T\n            pixels=pixels[:,:2]/pixels[:,2:]\n            h,w=masks_array[camera].shape\n            visible=(points[:,2]>.1)&(pixels[:,0]>=0)&(pixels[:,0]<w)&(pixels[:,1]>=0)&(pixels[:,1]<h)\n            chosen=points[visible];uv=pixels[visible].astype(int)\n            chosen=chosen[masks_array[camera][uv[:,1],uv[:,0]]]\n            np.savez_compressed(root/('depth_fixture_'+str(stamp)+'.npz'),points_cam=chosen,k=k,mask=masks_array[camera])")
(base/'v28_lidar_depth_replay.py').write_text(s)
