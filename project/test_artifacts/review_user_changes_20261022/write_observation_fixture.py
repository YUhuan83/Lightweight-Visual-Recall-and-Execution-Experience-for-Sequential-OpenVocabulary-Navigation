from pathlib import Path
import json
out=Path('/home/ly/execution_continuity_review_20261022/v4/full/trial_20261022_180945/observation_replay')
interfaces=json.loads((out/'request59_interfaces.json').read_text())
grid=json.loads((out/'request59_public_grid.json').read_text())
record=grid['record']
start=interfaces['/spot1/odom']['pose']
goal=interfaces['/spot1/graphnav_planner/path']['ends'][-1]
values=[*record['size'],record['resolution'],*record['origin'][:2],*start,*goal]
with (out/'request59_public_grid.txt').open('w') as f:
    f.write(' '.join(map(str,values))+'\n')
    f.writelines(f'{i} {value}\n' for i,value in enumerate(grid['data']) if value)
print(out/'request59_public_grid.txt')
