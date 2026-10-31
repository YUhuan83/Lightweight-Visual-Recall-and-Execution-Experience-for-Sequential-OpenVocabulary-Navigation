"""Build an independent native FreeRoam level using only original assets."""
import hashlib
import json
import math
from pathlib import Path
import random
import uuid
import zipfile
from PIL import Image, ImageDraw, ImageFont
from mesh import Mesh

ROOT = Path(__file__).resolve().parent
LEVEL = 'nizhen_countryside'
PREFIX = 'levels/' + LEVEL + '/'
ART = '/' + PREFIX + 'art/shapes/'
MOD_NAME = 'nizhen_countryside_standalone.zip'
SPAWN = (-40.0, -80.0, 10.65)
QUAT = (0.0, 0.0, math.sqrt(.5), math.sqrt(.5))
COLORS = {'grass':(99,117,65),'concrete':(163,160,145),'mortar':(93,87,75),
          'stone':(140,135,118),'wood':(93,69,43),'wall':(195,180,145),
          'roof':(116,65,46),'window':(46,67,70),'leaf':(56,91,43),
          'leaf2':(76,103,48),'rock':(114,113,101),'white':(225,219,194),
          'black':(29,33,30),'red':(173,66,39)}
HOUSES = [(x,y) for y in (-101,-59) for x in (-48,-16,16,48)]


def smooth(t):
    t = min(1.0,max(0.0,t))
    return t*t*(3-2*t)


def height(x,y):
    return 10 + smooth((y+24)/140)*(7+2.2*math.sin(x/48)+1.2*math.sin(y/37))


def road_distance(x,y):
    qx,qy = abs(x)-56,abs(y)-56
    return abs(math.hypot(max(qx,0),max(qy,0))+min(max(qx,qy),0)-24)


def surface(x,y):
    road = road_distance(x,y) < 4 or (abs(x)<4 and -80<=y<=8) or math.hypot(x,y-8)<11
    if not road:
        return 'grass'
    return 'mortar' if -24<=x<24 and -84<=y<-76 else 'concrete'


def native(name, cls, parent=None, **fields):
    obj = {'name':name,'class':cls,'persistentId':str(uuid.uuid5(uuid.NAMESPACE_URL, LEVEL+'/'+name))}
    if parent:
        obj['__parent'] = parent
    return dict(obj, **fields)


def write_json(path, value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')


def write_rows(path, rows):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(''.join(json.dumps(r,separators=(',',':'))+'\n' for r in rows),encoding='utf-8')


def generate_meshes():
    meshes = {}
    for ty in range(-128,128,32):
        for tx in range(-128,128,32):
            m = Mesh(f'ground_{tx+128}_{ty+128}')
            for y in range(ty,ty+32):
                for x in range(tx,tx+32):
                    m.quad(*[(a,b,height(a,b)) for a,b in ((x,y),(x+1,y),(x+1,y+1),(x,y+1))],
                           'nc_'+surface(x+.5,y+.5))
            meshes[m.name] = m
    # Bedding is the ground itself. Bevelled stones meet it only at their rims.
    rng = random.Random(3911)
    for chunk in range(4):
        m = Mesh('cobbles_'+str(chunk))
        for row in range(chunk*24,(chunk+1)*24):
            for col in range(16):
                x,y = -24+row*.5,-84+col*.5
                h = rng.uniform(.012,.025)*min(1,(row+.5)/4,(95.5-row)/4)
                base = [(x+.007,y+.007,10),(x+.493,y+.007,10),(x+.493,y+.493,10),(x+.007,y+.493,10)]
                top = [(x+.035,y+.035,10+h),(x+.465,y+.035,10+h),(x+.465,y+.465,10+h),(x+.035,y+.465,10+h)]
                m.quad(*top,'nc_stone')
                for i in range(4):
                    j=(i+1)%4
                    m.quad(base[i],base[j],top[j],top[i],'nc_stone')
        meshes[m.name]=m
    village = Mesh('village')
    fence = Mesh('fences')
    for x,y in HOUSES:
        z=height(x,y)
        village.box((x,y,z+1.9),(10,8,4.2),'nc_wall')
        for side in (-1,1):
            a,b=(x-5.5,y+side*4.5,z+4),(x+5.5,y+side*4.5,z+4)
            c,d=(x+5.5,y,z+6),(x-5.5,y,z+6)
            village.quad(*((a,b,c,d) if side==-1 else (b,a,d,c)),'nc_roof')
        village.triangle((x-5,y-4,z+4),(x-5,y,z+5.78),(x-5,y+4,z+4),'nc_wall')
        village.triangle((x+5,y+4,z+4),(x+5,y,z+5.78),(x+5,y-4,z+4),'nc_wall')
        facing = 1 if y < -80 else -1
        village.box((x,y+facing*4.03,z+1.25),(1.3,.12,2.5),'nc_wood')
        for wx in (-3,3):
            village.box((x+wx,y+facing*4.07,z+2.1),(1.7,.16,1.45),'nc_white')
            village.box((x+wx,y+facing*4.17,z+2.1),(1.4,.06,1.15),'nc_window')
        village.box((x+3,y+1,z+5),(1,1,3),'nc_roof')
        fy = y+facing*9
        for dx in (-10,-7,-4,4,7,10):
            fence.box((x+dx,fy,z+.6),(.16,.16,1.2),'nc_wood')
        for dx in (-7,7):
            for dz in (.4,.9):
                fence.box((x+dx,fy,z+dz),(6,.12,.12),'nc_wood')
    meshes[village.name],meshes[fence.name]=village,fence
    # Eight spatial batches, not hundreds of separately registered tree objects.
    trees=[]
    for group in range(8):
        m=Mesh('forest_'+str(group))
        attempts=0
        while len(trees)<(group+1)*22:
            attempts+=1
            if attempts>10000:
                raise ValueError('Tree placement exhausted')
            x,y=rng.uniform(-120,120),rng.uniform(-120,120)
            if road_distance(x,y)<10 or (abs(x)<17 and -88<y<25):
                continue
            if any(abs(x-a)<17 and abs(y-b)<15 for a,b in HOUSES):
                continue
            if any(math.hypot(x-a,y-b)<6 for a,b in trees):
                continue
            trees.append((x,y))
            z=height(x,y)-.15
            h=rng.uniform(5.5,9)
            m.lathe((x,y,z),[(0,.28),(h*.7,.17)],'nc_wood',8)
            m.lathe((x,y,z),[(h*.32,.5),(h*.52,2.4),(h*.78,1.9),(h,.12)],
                    'nc_leaf' if group%2 else 'nc_leaf2',9)
        meshes[m.name]=m
    rocks=Mesh('rocks')
    for x,y in ((-104,-32),(-109,18),(104,24),(106,55),(-35,106),(38,110),(24,26),(-23,29)):
        rocks.lathe((x,y,height(x,y)-.25),[(0,1.4),(.7,1.8),(1.5,.8),(1.8,.1)],'nc_rock',7)
    meshes[rocks.name]=rocks
    targets=Mesh('targets')
    for x,y in ((-9,-35),(10,-15),(-12,12)):
        z=height(x,y)
        targets.box((x,y,z+1),(.15,.15,2),'nc_wood')
        targets.box((x,y,z+2),(1.6,.14,1.6),'nc_white')
        for row in range(4):
            for col in range(4):
                if (row+col)%2==0:
                    targets.box((x-.6+col*.4,y-.08,z+1.4+row*.4),(.4,.03,.4),'nc_black')
    for x,y in ((-9,3),(9,3)):
        targets.lathe((x,y,height(x,y)),[(0,.45),(.95,.45)],'nc_red',16)
    meshes[targets.name]=targets
    border=Mesh('boundary')
    for side in (-1,1):
        for k in range(-128,128,4):
            for vertical in (False,True):
                x,y=(side*126,k+2) if vertical else (k+2,side*126)
                border.box((x,y,height(x,y)+.6),(.5,4,1.6) if vertical else (4,.5,1.6),'nc_rock')
    meshes[border.name]=border
    return meshes, trees


def preview(path,trees):
    image=Image.new('RGB',(960,600),(27,38,32))
    d=ImageDraw.Draw(image)
    font=ImageFont.load_default(size=29)
    small=ImageFont.load_default(size=17)
    d.text((32,22),'NIZHEN COUNTRYSIDE',font=font,fill=(233,225,200))
    d.text((32,64),'256 x 256 m | ORIGINAL STANDALONE LEVEL',font=small,fill=(161,185,153))
    def p(x,y): return (int(280+x*1.65),int(331-y*1.65))
    for iy in range(424):
        for ix in range(424):
            x,y=(ix-212)/1.65,(212-iy)/1.65
            color=COLORS[surface(x,y)]
            shade=1+(height(x,y)-14)*.016
            image.putpixel((68+ix,119+iy),tuple(min(255,int(v*shade)) for v in color))
    for x,y in trees:
        px,py=p(x,y)
        d.ellipse((px-4,py-4,px+4,py+4),fill=COLORS['leaf'])
    for x,y in HOUSES:
        px,py=p(x,y)
        d.rectangle((px-8,py-6,px+8,py+6),fill=COLORS['roof'],outline=(235,215,172))
    px,py=p(*SPAWN[:2])
    d.polygon(((px+10,py),(px-6,py-6),(px-6,py+6)),fill=(255,218,96))
    for i,(label,color) in enumerate((('Concrete loop + branch',COLORS['concrete']),('Physical cobbles: 48 m',COLORS['mortar']),('Village: 8 houses',COLORS['roof']),('176 trees / gentle hills',COLORS['leaf']),('Start: eastbound', (255,218,96)))):
        yy=145+i*49
        d.rectangle((550,yy,566,yy+16),fill=color)
        d.text((582,yy-2),label,font=small,fill=(225,220,199))
    d.text((550,440),'LAYOUT DIAGRAM',font=font,fill=(231,213,165))
    d.text((550,480),'Not an in-game render',font=small,fill=(185,192,178))
    d.text((32,565),'Driving baseline | Suggested speed: 15-25 km/h | N up',font=small,fill=(194,206,185))
    image.save(path)


def build():
    out=ROOT/'generated'
    level=out/'mod'/PREFIX
    art=level/'art/shapes'
    art.mkdir(parents=True,exist_ok=True)
    meshes,trees=generate_meshes()
    files=[]
    materials={}
    for index,(name,color) in enumerate(COLORS.items()):
        rng=random.Random(index+3911)
        im=Image.new('RGB',(128,128))
        pixels=[]
        for y in range(128):
            for x in range(128):
                noise=rng.gauss(0,5)
                if name=='wood': noise+=9*math.sin(x*math.tau/16)
                if name=='concrete' and (x<1 or y<1): noise-=16
                pixels.append(tuple(max(0,min(255,round(v+noise))) for v in color))
        im.putdata(pixels)
        texture=art/(name+'.color.png')
        im.save(texture)
        files.append(texture)
        mat='nc_'+name
        ground={'grass':'GRASS','wood':'WOOD','leaf':'WOOD','leaf2':'WOOD','rock':'ROCK'}.get(name,'ASPHALT')
        materials[mat]=dict(name=mat,mapTo=mat,**{'class':'Material'},version=1.5,
                            Stages=[dict(baseColorMap=ART+texture.name,baseColorFactor=[1,1,1,1],
                                         roughnessFactor=.93,metallicFactor=0,useAnisotropic=True),{},{},{}],
                            groundType=ground,translucentBlendOp='None',doubleSided=False)
    write_json(art/'main.materials.json',materials)
    files.append(art/'main.materials.json')
    statics=[]
    for name,m in meshes.items():
        dest=art/(name+'.dae')
        m.write(dest)
        files.append(dest)
        statics.append(native('nc_'+name,'TSStatic','Geometry',position=[0,0,0],scale=[1,1,1],
                              rotationMatrix=[1,0,0,0,1,0,0,0,1],shapeName=ART+dest.name,
                              collisionType='Visible Mesh Final',decalType='Visible Mesh',
                              useInstanceRenderData=True))
    def rows(relative,items):
        p=level/relative
        write_rows(p,items)
        files.append(p)
    rows('main/items.level.json',[native('MissionGroup','SimGroup',enabled='1')])
    rows('main/MissionGroup/items.level.json',[native(n,'SimGroup','MissionGroup') for n in ('Geometry','Sky','PlayerDropPoints')])
    rows('main/MissionGroup/Geometry/items.level.json',statics)
    rows('main/MissionGroup/PlayerDropPoints/items.level.json',[
        native('spawn_default','SpawnSphere','PlayerDropPoints',position=list(SPAWN),
               rotationMatrix=[0,1,0,-1,0,0,0,0,1],dataBlock='SpawnSphereMarker',radius=1,
               spawnClass='player',spawnDatablock='DefaultPlayerData',sphereWeight='1',enabled='1')])
    rows('main/MissionGroup/Sky/items.level.json',[
        native('theLevelInfo','LevelInfo','Sky',gravity=-9.81,visibleDistance=1800,
               fogDensity=.0003,fogAtmosphereHeight=300,fogColor=[.74,.81,.9,1],
               globalEnviromentMap='BNG_Sky_01_cubemap',canvasClearColor=[1,1,1,255]),
        native('sunsky','ScatterSky','Sky',position=[0,0,100],azimuth=230,elevation=45,
               skyBrightness=40,mieScattering=.00043,enableFogFallBack=False,
               ambientScale=[1,.93,.84,1],sunScale=[1,.91,.8,1],colorize=[.22,.35,.6,1],
               fogScale=[.4,.67,1,1],castShadows=True,
               **{key:'art/sky_gradients/default/gradient_'+value+'.png' for key,value in
                  (('ambientScaleGradientFile','ambient'),('colorizeGradientFile','colorize'),
                   ('fogScaleGradientFile','fog'),('sunScaleGradientFile','sunscale'))}),
        native('tod','TimeOfDay','Sky',play=False,axisTilt=10,startTime=.15,time=.15),
        native('nc_clouds','CloudLayer','Sky',texture='art/skies/clouds/clouds_normal_displacement',
               coverage=.45,windSpeed=.1,Textures=[{},{},{}])])
    info=dict(title='Nizhen Countryside',description='Original rural driving baseline. Layout preview, not a game render.',
              authors='Nizhen workspace',size=[256,256],biome='Temperate countryside',
              roads='Concrete loop, physical cobbles, village branch',previews=['preview.png'],
              defaultSpawnPointName='spawn_default',spawnPoints=[dict(objectname='spawn_default',
              translationId='Village Road',preview='preview.png')],isAuxiliary=False,
              supportsTraffic=False,supportsTimeOfDay=True)
    write_json(level/'info.json',info)
    files.append(level/'info.json')
    preview(level/'preview.png',trees)
    files.append(level/'preview.png')
    license_path=level/'ASSET_LICENSE.txt'
    license_path.write_text('Original procedural meshes, textures and layout: CC0 1.0.\n'
                            'No stock art or vehicle assets are redistributed.\n'
                            'Requires a separately licensed BeamNG installation and its common sky resources.\n',encoding='ascii')
    files.append(license_path)
    archive=out/MOD_NAME
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=9) as z:
        for p in sorted(files):
            member=p.relative_to(out/'mod').as_posix()
            zi=zipfile.ZipInfo(member,(2026,9,11,0,0,0))
            zi.compress_type=zipfile.ZIP_DEFLATED
            z.writestr(zi,p.read_bytes())
    manifest=dict(level=LEVEL,zip=MOD_NAME,zip_bytes=archive.stat().st_size,
                  zip_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
                  files={p.relative_to(out/'mod').as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
                  spawn=dict(pos=SPAWN,rot_quat=QUAT),statics=len(statics),trees=len(trees),houses=len(HOUSES),
                  triangles=sum(len(m.vertices)//3 for m in meshes.values()),runtime_verified=False,
                  ground_size_m=[256,256],loop_length_m=448+48*math.pi,road_width_m=8,
                  cobble_relief_max_m=.025,mesh_triangles={n:len(m.vertices)//3 for n,m in meshes.items()})
    write_json(out/'build_manifest.json',manifest)
    print(json.dumps({k:v for k,v in manifest.items() if k not in ('files','mesh_triangles')},indent=2))


if __name__=='__main__':
    build()
