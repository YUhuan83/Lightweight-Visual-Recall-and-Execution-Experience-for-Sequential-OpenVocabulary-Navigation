"""Deterministic independent terraced village. All coordinates are metres, Z up."""
import argparse
import hashlib
import json
import math
import random
from pathlib import Path
from PIL import Image, ImageDraw
import base
from mesh import Mesh

ROOT=Path(__file__).resolve().parent
LEVEL='terraced_countryside'
PREFIX='levels/'+LEVEL+'/'
MOD_NAME='terraced_countryside.zip'
SPAWN=(-103.,0.,10.65)
QUAT=base.QUAT
write_json=base.write_json
COLORS=dict(base.COLORS, soil=(104,83,55),crop=(105,117,58),sack=(156,142,103),
            metal=(77,83,75), plaster=(184,177,155),brick=(137,91,67),drygrass=(151,145,82))
HOUSES=[(-76,13,8,7,2),(-49,20,10,8,3),(-19,16,7,7,2),(23,22,9,8,2),
        (58,14,8,6,3),(78,26,10,8,2),(-66,-20,9,7,2),(-32,-28,8,8,3),
        (8,-19,10,7,2),(43,-25,8,7,2),(73,-17,7,6,2)]
YS=sorted(set(list(range(-64,-9))+[-9,-8,-7,-6,-5,-4,-3,-2.3,-1.7,0,1.7,2.3,3,4,5,6,7,8,9]+list(range(10,65))))
SEED=20260911
MODE='passable'


def height(x,y):
    s=base.smooth
    if y>=0:
        z=(1.55+.18*math.sin(x/19))*s((y-2.3)/3.7)
        z+=.55*s((y-19-2*math.sin(x/16))/2.5)+.65*s((y-37-4*math.sin(x/23))/3)
    else:
        z=-1.05*s((-y-2.3)/2.7)+.38*s((-y-6)/3)
        z+=.045*math.sin(x/7)*s((-y-9)/3)+.045*math.sin(y*2.1)*s((-y-10)/3)
    # Broad end aprons and a 28 m passing bay are part of the same elevation mesh.
    apron=s((abs(x)-80)/10)*(1-s((abs(y)-12)/5))
    bay=(1-s((abs(x)-14)/8))*(1-s((abs(y)-6)/3))
    return 10+z*(1-max(apron,bay))


def surface(x,y):
    if (abs(x)<=116 and abs(y)<1.7) or (90<=abs(x)<=116 and abs(y)<12): return 'concrete'
    if abs(x)<=14 and -1.7<y<6: return 'concrete'
    if abs(y)<2.3: return 'mortar'
    if y<-3 and y>-8: return 'soil'
    if abs(y)>10: return 'crop' if int((x+128)/17)%3 else 'soil'
    return 'grass'


def obstacle_mesh(record):
    m=Mesh(record['id'])
    x,y=record['center']; w,d,h=record['size']; z=record['base_z']
    kind=record['kind']
    if kind=='crate':
        m.box((x,y,z+h/2),(w,d,h),'nc_wood')
        for f in (-1,1):
            for dz in (.12,h-.12): m.box((x,y+f*(d/2-.025),z+dz),(w,.05,.08),'nc_metal')
    else:
        rings={'barrel':[(0,.85),(.08,1),(.85,1),(1,.85)],
               'sack':[(0,.65),(.18,1),(.7,.95),(.95,.45),(1,.2)],
               'rock':[(0,.65),(.25,1),(.72,.8),(1,.15)]}[kind]
        m.lathe((0,0,0),[(a*h,b*.5) for a,b in rings],
                {'barrel':'nc_metal','sack':'nc_sack','rock':'nc_rock'}[kind],12 if kind!='rock' else 8)
        m.vertices=[(x+a*w,y+b*d,z+c) for a,b,c in m.vertices]
        # Nonuniform scaling requires normals rebuilt, not inherited from the unit lathe.
        old=m; m=Mesh(record['id'])
        for mat,indices in old.groups.items():
            for i in range(0,len(indices),3): m.triangle(*(old.vertices[j] for j in indices[i:i+3]),mat)
        if kind=='barrel':
            for dz in (.14,h-.14):
                m.lathe((x,y,z),[(dz,.5*min(w,d)),(dz+.045,.5*min(w,d))],'nc_black',12)
    return m


def bounds(m):
    return [[min(p[k] for p in m.vertices) for k in range(3)],
            [max(p[k] for p in m.vertices) for k in range(3)]]


def obstacles(seed,mode):
    rng=random.Random(seed)
    records=[]
    # All encroachments stay on the bank side, preserving a continuous straight corridor.
    for i in range(16):
        x=-73+i*9.6+rng.uniform(-1.4,1.4)
        if abs(x)<20: continue
        d=rng.uniform(.42,.68); w=rng.uniform(.55,.95)
        low=rng.uniform(1.05,1.27)
        records.append(dict(id='obstacle_road_'+str(i),kind=rng.choice(['sack','barrel','rock','crate']),
                            center=[x,low+d/2],size=[w,d,rng.uniform(.35,.8)],role='encroachment'))
    for i in range(44):
        x=-77+i*3.5+rng.uniform(-.45,.45)
        side=-1 if i%2 else 1
        records.append(dict(id='obstacle_edge_'+str(i),kind=['sack','barrel','rock','crate'][i%4],
                            center=[x,side*rng.uniform(2.65,2.9)],size=[.55,.45,rng.uniform(.3,.85)],role='roadside'))
    if mode=='blocked':
        for i in range(4):
            records.append(dict(id='closure_'+str(i),kind='crate',center=[35,-1.275+i*.85],
                                size=[1.2,.83,1.1],role='intentional_closure'))
    for r in records:
        x,y=r['center']; w,d,_=r['size']
        r['base_z']=min(height(x+a*w/2,y+b*d/2) for a in (-1,1) for b in (-1,1))-.015
        r['aabb']=bounds(obstacle_mesh(r))
    return records


def generate_meshes():
    rng=random.Random(1847)
    meshes={}
    for tx in range(-128,128,16):
        m=Mesh('ground_'+str(tx+128))
        for x in range(tx,tx+16):
            for ya,yb in zip(YS,YS[1:]):
                m.quad(*[(a,b,height(a,b)) for a,b in ((x,ya),(x+1,ya),(x+1,yb),(x,yb))],
                       'nc_'+surface(x+.5,(ya+yb)/2))
        meshes[m.name]=m
    village=Mesh('village'); details=Mesh('village_details')
    for index,(x,y,w,d,floors) in enumerate(HOUSES):
        z=min(height(x+a*w/2,y+b*d/2) for a in (-1,1) for b in (-1,1))-.2
        h=floors*2.7; face=-1 if y>0 else 1
        village.box((x,y,z+h/2),(w,d,h),'nc_plaster' if index%2 else 'nc_wall')
        village.box((x,y,z+.3),(w+.15,d+.15,.6),'nc_brick')
        for side in (-1,1):
            a,b=(x-w/2-.35,y+side*(d/2+.45),z+h),(x+w/2+.35,y+side*(d/2+.45),z+h)
            c,e=(x+w/2+.35,y,z+h+1.8),(x-w/2-.35,y,z+h+1.8)
            village.quad(*((a,b,c,e) if side==-1 else (b,a,e,c)),'nc_roof')
        for side in (-1,1):
            xx=x+side*w/2
            tri=[(xx,y-d/2,z+h),(xx,y+d/2,z+h),(xx,y,z+h+1.65)]
            village.triangle(*(tri if side==1 else tri[::-1]),'nc_brick')
        for floor in range(floors):
            for dx in (-w*.29,w*.29):
                zz=z+floor*2.7+1.7; yy=y+face*(d/2+.04)
                details.box((x+dx,yy,zz),(1.45,.12,1.5),'nc_white')
                details.box((x+dx,yy+face*.08,zz),(1.21,.05,1.26),'nc_window')
                details.box((x+dx,yy+face*.12,zz),(.055,.04,1.27),'nc_wood')
                details.box((x+dx,yy+face*.12,zz),(1.27,.04,.055),'nc_wood')
        details.box((x,y+face*(d/2+.1),z+1.1),(1.15,.2,2.2),'nc_wood')
        for step in range(3):
            details.box((x,y+face*(d/2+.3+step*.28),z+.1+(.3-step*.09)),(1.6,.3,.18),'nc_stone')
        details.box((x+w*.32,y+.8,z+h+.5),(.7,.8,2.3),'nc_brick')
        details.lathe((x-w/2-.12,y-face*d/2,z),[(0,.07),(h,.07)],'nc_metal',8)
        fy=y+face*(d/2+2)
        for dx in (-w/2,-w/2+2,w/2-2,w/2):
            zz=height(x+dx,fy)
            details.box((x+dx,fy,zz+.5),(.12,.12,1),'nc_wood')
        for dx in (-w/2+1,w/2-1):
            for dz in (.4,.8): details.box((x+dx,fy,height(x+dx,fy)+dz),(2,.1,.1),'nc_wood')
    meshes[village.name]=village; meshes[details.name]=details
    utility=Mesh('utility_edges')
    for x in (-80,-46,-9,31,68,83):
        y=7.5; z=height(x,y)
        utility.lathe((x,y,z),[(0,.14),(6,.095)],'nc_wood',8)
        utility.box((x,y,z+5.6),(.15,1.4,.14),'nc_wood')
    # Joint recesses and eroded shoulder stones are physical, millimetre-scale relief.
    joints=Mesh('road_joints')
    for x in range(-87,89,3):
        joints.box((x,0,10.001),(.016,3.4,.002),'nc_mortar')
    for i in range(260):
        x=rng.uniform(-79,79); y=rng.choice([-1,1])*rng.uniform(1.8,2.22)
        utility.lathe((x,y,height(x,y)-.01),[(0,.06),(.025,.085),(.04,.03)],'nc_stone',6)
    meshes[utility.name]=utility; meshes[joints.name]=joints
    for chunk in range(12):
        m=Mesh('grass_patch_'+str(chunk))
        for i in range(330):
            x=-84+chunk*14+rng.uniform(0,14)
            y=rng.choice([-1,1])*rng.uniform(2.34,7.8)
            if abs(x)<22 and y>0: continue
            for blade in range(7):
                xx=x+rng.uniform(-.22,.22); yy=y+rng.uniform(-.2,.2)
                z=height(xx,yy)-.025; h=rng.uniform(.22,.65); angle=rng.uniform(0,math.tau)
                dx=.025*math.cos(angle); dy=.025*math.sin(angle)
                a=(xx-dx,yy-dy,z); b=(xx+dx,yy+dy,z)
                c=(xx+rng.uniform(-.13,.13),yy+rng.uniform(-.13,.13),z+h)
                mat='nc_drygrass' if i%4==0 else 'nc_grass'
                m.triangle(a,b,c,mat,[(0,0),(1,0),(.5,1)])
                m.triangle(c,b,a,mat,[(.5,1),(1,0),(0,0)])
        meshes[m.name]=m
    border=Mesh('boundary')
    for x in range(-128,128,4):
        for y in (-63.5,63.5): border.box((x+2,y,height(x+2,y)+.5),(4,.5,1),'nc_rock')
    for y in range(-64,64,4):
        for x in (-127.5,127.5): border.box((x,y+2,height(x,y+2)+.5),(.5,4,1),'nc_rock')
    meshes[border.name]=border
    for r in obstacles(SEED,MODE): meshes[r['id']]=obstacle_mesh(r)
    return meshes,[]


def preview(path,unused):
    im=Image.new('RGB',(960,600),(30,39,33)); d=ImageDraw.Draw(im)
    d.text((28,22),'TERRACED COUNTRYSIDE | LAYOUT, NOT IN-GAME RENDER',fill='white')
    for x in range(-128,128):
        for y in range(-64,64):
            xx=96+(x+128)*3; yy=110+(64-y)*3
            col=COLORS[surface(x+.5,y+.5)]
            d.rectangle((xx,yy,xx+3,yy+3),fill=col)
    for x,y,w,h,_ in HOUSES:
        xx=96+(x+128)*3; yy=110+(64-y)*3
        d.rectangle((xx-w*1.5,yy-h*1.5,xx+w*1.5,yy+h*1.5),fill=COLORS['roof'])
    for r in obstacles(SEED,MODE):
        x,y=r['center']; xx=96+(x+128)*3; yy=110+(64-y)*3
        d.ellipse((xx-2,yy-2,xx+2,yy+2),fill=(230,195,126))
    d.text((28,530),'256 x 128 m | 180 m narrow lane | 3.4 m | 11 houses | '+MODE,fill='white')
    d.text((28,560),'LEFT +Y: TERRACES   RIGHT -Y: DITCH / LOW FIELDS   END APRONS: TURN / BACKTRACK',fill='white')
    im.save(path)


def build(seed=20260911,mode='passable'):
    global SEED,MODE
    SEED,MODE=seed,mode
    # Remove only obsolete closure meshes authored by this generator, never arbitrary content.
    art_dir=ROOT/'generated/mod'/PREFIX/'art/shapes'
    for i in range(4):
        old=art_dir/('closure_'+str(i)+'.dae')
        if old.exists(): old.unlink()
    for key,value in dict(ROOT=ROOT,LEVEL=LEVEL,PREFIX=PREFIX,ART='/'+PREFIX+'art/shapes/',
                          MOD_NAME=MOD_NAME,SPAWN=SPAWN,COLORS=COLORS,HOUSES=HOUSES,
                          generate_meshes=generate_meshes,preview=preview).items(): setattr(base,key,value)
    base.build()
    # Final native metadata and checksums are rebuilt after replacing baseline-specific labels.
    level=ROOT/'generated/mod'/PREFIX
    info=json.loads((level/'info.json').read_text())
    info.update(title='Terraced Countryside',size=[256,128],description='Terraced single-lane village. Layout preview, not a render.',
                roads='3.4 m aged concrete lane, passing bay, end turning aprons')
    info['spawnPoints'][0]['translationId']='West Turning Apron'
    write_json(level/'info.json',info)
    rows=level/'main/MissionGroup/Geometry/items.level.json'
    objects=[json.loads(line) for line in rows.read_text().splitlines()]
    for obj in objects:
        if 'grass_patch_' in obj['name']: obj['collisionType']='None'
    base.write_rows(rows,objects)
    # Purpose-made authored architectural textures; no downloaded assets required.
    art=level/'art/shapes'
    for name in ('wall','plaster','brick','roof','wood','concrete','sack','grass','drygrass','crop','soil'):
        rng=random.Random(777+list(COLORS).index(name))
        im=Image.new('RGB',(512,512)); pixels=[]
        for y in range(512):
            for x in range(512):
                n=rng.gauss(0,5)+3*math.sin(x/31)*math.sin(y/43)
                if name in ('brick','roof'): n-=27 if y%32<3 or (x+(y//32%2)*32)%64<2 else 0
                if name=='wood': n+=12*math.sin(x/5)+6*math.sin(x/2+y/70)
                if name=='sack': n+=8*((x%4==0)+(y%4==0))
                if name in ('grass','drygrass'): n+=22*y/512+8*math.sin(x/7)
                pixels.append(tuple(max(0,min(255,int(c+n))) for c in COLORS[name]))
        im.putdata(pixels)
        if name=='concrete':
            d=ImageDraw.Draw(im)
            for i in range(19):
                x=rng.randrange(512); y=rng.randrange(512)
                points=[(x,y)]
                for j in range(rng.randrange(3,9)):
                    x+=rng.randrange(-15,16); y+=rng.randrange(4,19); points.append((x,y))
                d.line(points,fill=(100,99,86),width=1)
            for i in range(140):
                x,y=rng.randrange(512),rng.randrange(512); r=rng.randrange(1,5)
                d.ellipse((x,y,x+r,y+r),fill=(128,124,108))
        im.save(art/(name+'.color.png'))
    import zipfile
    files=sorted(p for p in level.rglob('*') if p.is_file())
    archive=ROOT/'generated'/MOD_NAME
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=9) as z:
        for p in files:
            zi=zipfile.ZipInfo(p.relative_to(ROOT/'generated/mod').as_posix(),(2026,9,11,0,0,0))
            zi.compress_type=zipfile.ZIP_DEFLATED; z.writestr(zi,p.read_bytes())
    manifest=json.loads((ROOT/'generated/build_manifest.json').read_text())
    for key in ('loop_length_m','cobble_relief_max_m','trees'): manifest.pop(key,None)
    manifest.update(zip_bytes=archive.stat().st_size,zip_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
                    files={p.relative_to(ROOT/'generated/mod').as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
                    seed=seed,mode=mode,ground_size_m=[256,128],road_width_m=3.4,lane_length_m=180,
                    ground_implementation='TSStatic visible collision, not editable TerrainBlock',
                    grass_triangles=sum(v for k,v in manifest['mesh_triangles'].items() if k.startswith('grass_patch_')))
    write_json(ROOT/'generated/build_manifest.json',manifest)
    records=obstacles(seed,mode)
    write_json(ROOT/'generated/evaluator/ground_truth.json',dict(seed=seed,mode=mode,obstacles=records,
        vehicle_envelope_m=[5.8,2.2],lateral_margin_per_side_m=.25,
        guaranteed_geometric_corridor_y=[-1.7,1.05] if mode=='passable' else None,
        min_free_width_m=2.75 if mode=='passable' else .02,
        spawn_keepout_x=[-116,-85],end_keepout_x=[85,116],
        topology='West apron -> lane -> passing bay -> lane -> east apron',
        blocked_recovery='Stop before x=35 closure; reverse to x=0 passing bay or west apron to turn. No lateral bypass at closure.',
        navigation_success_guaranteed=False))
    print(json.dumps({k:manifest[k] for k in ('level','seed','mode','zip_bytes','triangles','grass_triangles')},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seed',type=int,default=20260911)
    p.add_argument('--mode',choices=['passable','blocked'],default='passable')
    a=p.parse_args(); build(a.seed,a.mode)
