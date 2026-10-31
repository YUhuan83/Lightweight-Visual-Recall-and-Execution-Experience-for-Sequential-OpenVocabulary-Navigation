"""Necessary vehicle-map checks for observation proposals; Smac decides reachability."""
from collections import deque
import math
from .graph_route import footprint_blockers


def grid_cell(grid, point):
    info=grid.info
    q=info.origin.orientation
    heading=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    dx=point[0]-info.origin.position.x;dy=point[1]-info.origin.position.y
    x=math.floor((math.cos(heading)*dx+math.sin(heading)*dy)/info.resolution)
    y=math.floor((-math.sin(heading)*dx+math.cos(heading)*dy)/info.resolution)
    return (x,y) if 0<=x<info.width and 0<=y<info.height else None


def connected_free_cells(grid, start):
    """Known, non-inscribed centre cells connected to the actual vehicle.

    Nav2 already inflates obstacles by its configured inscribed radius. This
    is a necessary centre-space check, not another inflation or a car path.
    """
    cell=grid_cell(grid,start)
    if cell is None:
        return bytearray(len(grid.data))
    width=grid.info.width;height=grid.info.height
    free=bytearray(0<=v<99 for v in grid.data)
    first=cell[1]*width+cell[0]
    reached=bytearray(len(free))
    if not free[first]:
        return reached
    queue=deque([first]);reached[first]=1
    while queue:
        index=queue.popleft();x=index%width;y=index//width
        for neighbour,allowed in ((index-1,x>0),(index+1,x+1<width),
                                  (index-width,y>0),(index+width,y+1<height)):
            if allowed and free[neighbour] and not reached[neighbour]:
                reached[neighbour]=1;queue.append(neighbour)
    return reached


def observation_pose_clear(grid, connected, point, heading):
    cell=grid_cell(grid,point)
    return (cell is not None and bool(connected[cell[1]*grid.info.width+cell[0]])
            and not footprint_blockers(grid,point,heading,limit=1))
