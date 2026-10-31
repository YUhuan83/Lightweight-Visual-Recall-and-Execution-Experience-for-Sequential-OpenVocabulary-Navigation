"""Plot closed public costmaps and recorded axle/path; no scene image edits."""
from pathlib import Path
import hashlib
import json
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import Polygon, Patch
import numpy as np

root = Path(__file__).resolve().parent
folder = root/'stall_connectivity_164222'
report = json.loads((folder/'report.json').read_text())
repo = root.parents[1]
for name, want in report['production_helpers'].items():
    path = repo/'nav2_adapter/wildos_nav2_adapter'/name
    assert hashlib.sha256(path.read_bytes()).hexdigest() == want
assert report['source_db_sha256'] == 'dac5d8cfeef67a2a71b72edb324b4f5f80f994749d386a01b5a7a74f33c4ad45'
snapshots = [json.loads((folder/(label+'.json')).read_text())
             for label in ('section_dispatched', 'stall_window_start', 'stall_window_end')]
palette = ListedColormap(['#9b9b9b', '#f6f6f6', '#d8e8ef', '#86b9cf', '#e5a84b', '#c44745'])
norm = BoundaryNorm([-1.5, -.5, .5, 50., 98.5, 99.5, 100.5], palette.N)
fig, axes = plt.subplots(2, 3, figsize=(14, 8), constrained_layout=True)
cx, cy = snapshots[-1]['review']['pose'][:2]
for column, snapshot in enumerate(snapshots):
    px, py, heading = snapshot['review']['pose']
    points = np.array([[p['pose']['position']['x'], p['pose']['position']['y']]
                       for p in snapshot['path']['poses']])
    footprint = np.array([[3.4, 1.05], [3.4, -1.05], [-1.05, -1.05], [-1.05, 1.05]])
    rotation = np.array([[math.cos(heading), -math.sin(heading)], [math.sin(heading), math.cos(heading)]])
    body = footprint @ rotation.T + [px, py]
    for row, topic in enumerate(('/global_costmap/costmap', '/local_costmap/costmap')):
        ax = axes[row, column]; grid = snapshot[topic]; info = grid['info']
        image = np.array(grid['data']).reshape(info['height'], info['width'])
        x = info['origin']['position']['x']; y = info['origin']['position']['y']; res = info['resolution']
        ax.imshow(image, origin='lower', extent=(x, x+info['width']*res, y, y+info['height']*res),
                  cmap=palette, norm=norm, interpolation='nearest')
        ax.plot(points[:, 0], points[:, 1], color='#20995f', linewidth=2, label='Accepted section')
        ax.add_patch(Polygon(body, fill=False, edgecolor='#252525', linewidth=1.7))
        ax.scatter([px], [py], marker='o', s=25, c='#252525', zorder=4)
        ax.arrow(px, py, math.cos(heading), math.sin(heading), width=.03, color='#252525', zorder=4)
        ax.set_xlim(cx-6., cx+6.); ax.set_ylim(cy-6., cy+6.)
        ax.set_aspect('equal'); ax.grid(alpha=.13)
        ax.set_title(f"{topic.split('/')[1]} | t={snapshot['review']['goal_elapsed']:.2f}s")
        ax.set_xlabel('World X (m)'); ax.set_ylabel('World Y (m)')
fig.suptitle('Recorded public maps permit the swept prefix while physical progress fails\n'
             'Black: rear-axle footprint. Green: accepted section. Public maps do not prove private cache identity.', fontsize=13)
fig.legend(handles=[Patch(color=c, label=l) for c, l in zip(palette.colors,
           ['Unknown', 'Free', 'Cost 1-49', 'Cost 50-98', 'Inscribed inflation 99', 'Obstacle 100'])],
           loc='outside lower center', ncol=6, frameon=False)
fig.savefig(root/'stall_public_maps.png', dpi=150)
print('Verified helper hashes and saved stall_public_maps.png')
