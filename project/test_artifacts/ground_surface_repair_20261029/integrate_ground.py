"""Replace the extrapolated far-plane test in this isolated candidate only."""
from pathlib import Path
root=Path(__file__).resolve().parent
path=root/'candidate/nav2_adapter/wildos_nav2_adapter/pointcloud.py'
text=path.read_text(encoding='utf8')
prototype=(root/'patch_prototype.py').read_text(encoding='utf-8-sig')
function=prototype[prototype.index('def supported_ground('):prototype.index("if __name__=='__main__':")]
function=function.replace('def supported_ground(points,plane,radius,obstacle_height,max_slope,fit=None):','''def supported_ground_patches(points, plane, radius, obstacle_height, max_slope):
    """Grow measured local road surfaces across overlapping supported patches.

    The nearby plane seeds ground only within 4 m. Every subsequent patch
    needs its own robust lower-return fit and at least 12 shared, noncollinear
    ground cells. Height offsets across steps and unsupported isolated tops
    cannot establish that connection. Per-return residuals preserve objects
    above a supported surface; no clearing endpoints are added.
    """''')
begin=text.index('def supported_far_ground_returns(');end=text.index('def split_costmap_points(',begin)
text=text[:begin]+function+'\n'+text[end:]
old='''            if np.any(near_obstacle):
                # A supported curved road can be above the large plane. Only
                # remove positively confirmed ground, retaining unsupported
                # surfaces, step-separated patches and all higher returns.
                ground = connected_ground_returns(points, plane, ground_plane_radius,
                                                   obstacle_min_height, ground_max_slope)
                near_obstacle &= ~ground
            far_body = ((~nearby) & (points[:, 2] >= min_height)
                        & (points[:, 2] <= max_height))
            if np.any(far_body):
                far_body &= ~supported_far_ground_returns(
                    points, plane, far_body, ground_plane_radius,
                    obstacle_min_height, ground_max_slope)
            candidates = points[near_obstacle | far_body]'''
new='''            far_body = ((~nearby) & (points[:, 2] >= min_height)
                        & (points[:, 2] <= max_height))
            obstacles = near_obstacle | far_body
            if np.any(obstacles):
                # Dense nearby scans retain their exact measured-cell
                # continuity. Sparse scan rings use overlapping local fits,
                # rather than extrapolating the vehicle's nearby road plane.
                ground = connected_ground_returns(points, plane, ground_plane_radius,
                                                   obstacle_min_height, ground_max_slope)
                ground |= supported_ground_patches(points, plane, ground_plane_radius,
                                                    obstacle_min_height, ground_max_slope)
                obstacles &= ~ground
            candidates = points[obstacles]'''
assert old in text
text=text.replace(old,new)
path.write_text(text,encoding='utf8')
