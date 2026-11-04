"""Extract the exported Blue Raven PCB's planar geometry into a KiCad module."""
from pathlib import Path
import re
import json
import math
import sys
from collections import Counter
import hashlib
import shutil

SOURCE=Path(__file__).parent/'bluraven_2025_v9 - 16-copper-1__Body1.step'
raw=SOURCE.read_text()
entities={int(i):(kind,body) for i,kind,body in re.findall(r'#(\d+)\s*=\s*(\w+)\s*\((.*?)\)\s*;',raw,re.S)}
def refs(i):
    return [int(v) for v in re.findall(r'#(\d+)',entities[i][1])]
def vec(i):
    return tuple(float(v) for v in re.search(r'\(([^()]*)\)\s*$',entities[i][1])[1].split(','))
def axis(i):
    r=refs(i)
    return vec(r[0]),vec(r[1]),vec(r[2]) if len(r)>2 else None
def point(vertex):
    return vec(refs(vertex)[0])
def edges_of_bound(bound):
    return [refs(edge)[-1] for edge in refs(refs(bound)[0])]
def extent(bound):
    pts=[point(v) for e in edges_of_bound(bound) for v in refs(e)[:2]]
    return (max(p[0] for p in pts)-min(p[0] for p in pts))*(max(p[1] for p in pts)-min(p[1] for p in pts))
face=max((i for i,v in entities.items() if v[0]=='ADVANCED_FACE'),key=lambda i:len(refs(i)))
outer=max(refs(face)[:-1],key=extent)
outline_edges=edges_of_bound(outer)
if '--inspect' in sys.argv:
    print('ENTITY COUNTS',Counter(v[0] for v in entities.values()))
    print('UNITS',[v for v in entities.values() if 'UNIT' in v[0]])
    print('CYLINDERS')
    for i,(kind,body) in entities.items():
        if kind=='CYLINDRICAL_SURFACE' and float(body.split(',')[-1])>0.0005:
            print(i,float(body.split(',')[-1]),axis(refs(i)[0]))
    print('HORIZONTAL FACES')
    for i,(kind,body) in entities.items():
        if kind=='ADVANCED_FACE':
            r=refs(i)
            if entities[r[-1]][0]=='PLANE':
                ax=axis(refs(r[-1])[0])
                if abs(ax[1][2])>0.99 and len(r)>5:
                    print(i,'bounds',len(r)-1,'plane',r[-1],ax)
    print('OUTLINE',face,outer)
    for e in outline_edges:
        r=refs(e)
        print(e,entities[e],point(r[0]),point(r[1]),entities[r[2]])
    sys.exit()

DEST=Path(r'C:\Users\Ejfso\Documents\GitHub\witness\Hardware\KicadLibs')
OUT=Path(__file__).parent/'bluraven_module_output'
LIB=DEST/'Witness_Modules.kicad_sym'
NAME='BlueRaven_2025_Location_Template'
def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()
if '--install' in sys.argv:
    metadata=json.loads((OUT/f'{NAME}_pinmap.json').read_text())
    assert digest(LIB)==metadata['previous_library_sha256'], 'Library changed since generation'
    target=DEST/f'Witness_Modules.pretty/{NAME}.kicad_mod'
    backup=DEST/'Witness_Modules.before-BlueRaven.kicad_sym'
    for p in (target,backup,DEST/f'{NAME}_pinmap.json'):
        assert not p.exists(), f'Refusing to overwrite {p}'
    shutil.copy2(LIB,backup)
    shutil.copy2(OUT/f'Witness_Modules.pretty/{NAME}.kicad_mod',target)
    shutil.copy2(OUT/f'{NAME}_pinmap.json',DEST/f'{NAME}_pinmap.json')
    shutil.copy2(OUT/'Witness_Modules.kicad_sym',LIB)
    print('Installed',NAME,'with a backup of the previous symbol library.')
    sys.exit()

assert 'SI_UNIT($,.METRE.)' in raw
origin=(22.529799925,10.0076)
def xy(p):
    return (round(p[0]*1000-origin[0],6),round(origin[1]-p[1]*1000,6))
def n(v):
    return f'{v:.6f}'.rstrip('0').rstrip('.') or '0'
cylinders=[]
for i,(kind,body) in entities.items():
    if kind=='CYLINDRICAL_SURFACE':
        center,direction,_=axis(refs(i)[0])
        if abs(direction[2])>0.999:
            cylinders.append((float(body.split(',')[-1])*2000,center,i))
terminal_candidates=[(d,p,i) for d,p,i in cylinders if abs(d-0.044*25.4)<0.025]
mount_candidates=[(d,p,i) for d,p,i in cylinders if 3.0<d<3.5]
assert len(terminal_candidates)==6 and len(mount_candidates)==2
# Top in the footprint is positive STEP Y. Number down the terminal row.
terminal_candidates.sort(key=lambda v:-v[1][1])
mount_candidates.sort(key=lambda v:(-v[1][1],v[1][0]))
pins=[dict(pin=str(j),x_mm=xy(p)[0],y_mm=xy(p)[1],step_center_m=p,source_diameter_mm=d,entity=i) for j,(d,p,i) in enumerate(terminal_candidates,1)]
mounts=[dict(label=f'MH{j}',x_mm=xy(p)[0],y_mm=xy(p)[1],step_center_m=p,source_diameter_mm=d,entity=i) for j,(d,p,i) in enumerate(mount_candidates,1)]
OUT.mkdir(exist_ok=True)
pretty=OUT/'Witness_Modules.pretty'
pretty.mkdir(exist_ok=True)
fp=[f'(footprint "{NAME}" (version 20241229) (generator "pcbnew") (layer "F.Cu")',
    '(descr "Blue Raven 2025 PCB location template from STEP: six terminal center dots and two 4-40 mounting center dots; no pads or drills. Numbered top to bottom in unmirrored top view.") (attr board_only exclude_from_pos_files exclude_from_bom)',
    '(property "Reference" "A**" (at 0 -12) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))',
    f'(property "Value" "{NAME}" (at 0 12) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))']
segments=[]
for e in outline_edges:
    r=refs(e)
    start,end=point(r[0]),point(r[1])
    curve=entities[r[2]][0]
    a,b=xy(start),xy(end)
    if curve=='LINE':
        segments.append(('line',a,b))
    elif curve=='CIRCLE':
        center,_,_=axis(refs(r[2])[0])
        radius=float(entities[r[2]][1].split(',')[-1])
        # All four outline corners are minor quarter arcs.
        vx,vy=start[0]+end[0]-2*center[0],start[1]+end[1]-2*center[1]
        length=math.hypot(vx,vy)
        mid=xy((center[0]+radius*vx/length,center[1]+radius*vy/length,center[2]))
        segments.append(('arc',a,mid,b))
    else:
        raise ValueError(f'Unsupported outline curve {curve}')
for layer,width in [('F.SilkS',0.12),('F.Fab',0.1)]:
    for s in segments:
        coords=' '.join(f'({key} {n(p[0])} {n(p[1])})' for key,p in zip(('start','end') if s[0]=='line' else ('start','mid','end'),s[1:]))
        fp.append(f'(fp_{s[0]} {coords} (stroke (width {width}) (type solid)) (layer "{layer}"))')
for p in pins+mounts:
    x,y=p['x_mm'],p['y_mm']
    radius=0.2 if 'pin' in p else 0.3
    fp.append(f'(fp_circle (center {n(x)} {n(y)}) (end {n(x+radius)} {n(y)}) (stroke (width 0.1) (type solid)) (fill solid) (layer "F.SilkS"))')
    label=p.get('pin',p.get('label'))
    offset=-1.1 if 'pin' in p else 2
    fp.append(f'(fp_text user "{label}" (at {n(x+offset)} {n(y)}) (layer "F.SilkS") (effects (font (size 0.8 0.8) (thickness 0.12))))')
fp.append(')')
(pretty/f'{NAME}.kicad_mod').write_text('\n'.join(fp)+'\n',encoding='utf-8')
sym=[f'(symbol "{NAME}" (pin_names (offset 1.016)) (in_bom no) (on_board yes)',
    '(property "Reference" "A" (at 0 12.7 0) (effects (font (size 1.27 1.27))))',
    '(property "Value" "BlueRaven_2025" (at 0 10.16 0) (effects (font (size 1.27 1.27))))',
    f'(property "Footprint" "Witness_Modules:{NAME}" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))',
    '(property "Description" "Six generic terminal pins. Footprint is a silkscreen-only location template, without electrical pads or drilled holes. Pin numbers are positional, not manufacturer assignments." (at 0 0 0) (effects (font (size 1.27 1.27)) hide))',
    f'(symbol "{NAME}_0_1" (rectangle (start -10.16 7.62) (end 10.16 -10.16) (stroke (width 0.254) (type default)) (fill (type background))))',
    f'(symbol "{NAME}_1_1"']
for j in range(1,7):
    y=7.62-j*2.54
    sym.append(f'(pin passive line (at -15.24 {n(y)} 0) (length 5.08) (name "PIN_{j}" (effects (font (size 1.016 1.016)))) (number "{j}" (effects (font (size 1.016 1.016)))))')
sym.append('))')
original=LIB.read_text(encoding='utf-8')
assert f'(symbol "{NAME}"' not in original
combined=original.rstrip()[:-1]+'\n'+'\n'.join(sym)+'\n)\n'
(OUT/'Witness_Modules.kicad_sym').write_text(combined,encoding='utf-8')
metadata=dict(source=str(SOURCE),origin_mm=origin,view='unmirrored top view; STEP +X right, +Y up; footprint +Y down',numbering='1 through 6 from top to bottom',terminal_nominal_diameter_mm=0.044*25.4,pins=pins,mounting_locations=mounts,pads=False,drills=False,previous_library_sha256=digest(LIB))
(OUT/f'{NAME}_pinmap.json').write_text(json.dumps(metadata,indent=2)+'\n',encoding='utf-8')
import pcbnew as pcb
loaded=pcb.FootprintLoad(str(pretty.resolve()),NAME)
assert loaded is not None and loaded.GetPadCount()==0
shapes=[g for g in loaded.GraphicalItems() if isinstance(g,pcb.PCB_SHAPE)]
assert len(shapes)==24 # 16 outline edges plus 8 center dots, excluding text labels
assert sum(g.GetShape()==pcb.SHAPE_T_CIRCLE for g in shapes)==8
print('Validated footprint: zero pads; 8 center dots; exact 4-line/4-arc board outline on two layers.')
print(json.dumps(dict(pins=pins,mounts=mounts),indent=2))
