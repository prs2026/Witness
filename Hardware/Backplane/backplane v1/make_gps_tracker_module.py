"""Create the FWGPS mechanical template using measured STEP hole centers."""
from inspect_gps_step import SOURCE,raw,entities,refs,axis,point,edges_of_bound,bound_points
from pathlib import Path
import hashlib
import json
import shutil
import sys
import pcbnew as pcb

DEST=Path(r'C:\Users\Ejfso\Documents\GitHub\witness\Hardware\KicadLibs')
OUT=Path(__file__).parent/'gps_tracker_module_output'
LIB=DEST/'Witness_Modules.kicad_sym'
NAME='FWGPS_Tracker_v15_Location_Template'
def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()
if '--install' in sys.argv:
    metadata=json.loads((OUT/f'{NAME}_geometry.json').read_text())
    assert digest(LIB)==metadata['previous_library_sha256']
    backup=DEST/'Witness_Modules.before-FWGPS.kicad_sym'
    targets=[Path(f'Witness_Modules.pretty/{NAME}.kicad_mod'),Path(f'{NAME}_geometry.json')]
    for p in [backup]+[DEST/t for t in targets]:
        assert not p.exists(),f'Refusing to overwrite {p}'
    shutil.copy2(LIB,backup)
    for t in targets:
        shutil.copy2(OUT/t,DEST/t)
    shutil.copy2(OUT/'Witness_Modules.kicad_sym',LIB)
    print('Installed',NAME,'with library backup.')
    sys.exit()
assert 'SI_UNIT($,.METRE.)' in raw
def area(b):
    pts=bound_points(b)
    return (max(p[0] for p in pts)-min(p[0] for p in pts))*(max(p[1] for p in pts)-min(p[1] for p in pts))
face=max((i for i,v in entities.items() if v[0]=='ADVANCED_FACE' and entities[refs(i)[-1]][0]=='PLANE' and abs(axis(refs(refs(i)[-1])[0])[1][2])>0.99),key=lambda i:(max(area(b) for b in refs(i)[:-1]),axis(refs(refs(i)[-1])[0])[0][2]))
outer=max(refs(face)[:-1],key=area)
pts=bound_points(outer)
origin=((min(p[0] for p in pts)+max(p[0] for p in pts))*500,(min(p[1] for p in pts)+max(p[1] for p in pts))*500)
def xy(p):
    return (round(p[0]*1000-origin[0],6),round(origin[1]-p[1]*1000,6))
def n(v):
    return f'{v:.6f}'.rstrip('0').rstrip('.') or '0'
cylinders=[]
for i,(kind,body) in entities.items():
    if kind=='CYLINDRICAL_SURFACE':
        center,direction,_=axis(refs(i)[0])
        assert abs(direction[2])>0.999
        cylinders.append((float(body.split(',')[-1])*2000,center,i))
mount_candidates=sorted((v for v in cylinders if 3.0<v[0]<3.5),key=lambda v:v[1][0])
terminal_candidates=sorted((v for v in cylinders if 1.0<v[0]<1.2),key=lambda v:v[1][0])
assert len(mount_candidates)==2 and len(terminal_candidates)==2
assert any(abs(v[0]-0.044*25.4)<1e-7 for v in terminal_candidates)
def describe(candidates,prefix):
    return [dict(label=f'{prefix}{j}',x_mm=xy(p)[0],y_mm=xy(p)[1],diameter_mm=d,step_center_m=p,entity=i) for j,(d,p,i) in enumerate(candidates,1)]
mounts=describe(mount_candidates,'MH')
terminals=describe(terminal_candidates,'')
OUT.mkdir(exist_ok=True)
pretty=OUT/'Witness_Modules.pretty'
pretty.mkdir(exist_ok=True)
fp=[f'(footprint "{NAME}" (version 20241229) (generator "pcbnew") (layer "F.Cu")',
    '(descr "FWGPS Tracker v15: exact six-segment PCB outline; two measured 3.2512 mm unplated mounting holes; two screw terminal center dots numbered left to right in top view. Terminal markers have no copper pads or drills.") (attr board_only exclude_from_pos_files exclude_from_bom)',
    '(property "Reference" "A**" (at 0 -12) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))',
    '(property "Value" "FWGPS_Tracker_v15" (at 0 12) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))']
outline=[]
for e in edges_of_bound(outer):
    r=refs(e)
    assert entities[r[2]][0]=='LINE'
    a,b=xy(point(r[0])),xy(point(r[1]))
    outline.append((a,b))
    for layer,width in [('F.SilkS',0.12),('F.Fab',0.1)]:
        fp.append(f'(fp_line (start {n(a[0])} {n(a[1])}) (end {n(b[0])} {n(b[1])}) (stroke (width {width}) (type solid)) (layer "{layer}"))')
assert len(outline)==6
fp.append('(fp_rect (start -21.582 -10.66) (end 21.582 10.66) (stroke (width 0.05) (type solid)) (fill none) (layer "F.CrtYd"))')
for m in mounts:
    x,y,d=m['x_mm'],m['y_mm'],m['diameter_mm']
    fp.append(f'(pad "" np_thru_hole circle (at {n(x)} {n(y)}) (size {n(d)} {n(d)}) (drill {n(d)}) (layers "*.Cu" "*.Mask"))')
    fp.append(f'(fp_circle (center {n(x)} {n(y)}) (end {n(x+d/2)} {n(y)}) (stroke (width 0.1) (type solid)) (fill none) (layer "F.Fab"))')
    fp.append(f'(fp_text user "{m["label"]}" (at {n(x+2.7)} {n(y)}) (layer "F.SilkS") (effects (font (size 0.8 0.8) (thickness 0.12))))')
for t in terminals:
    x,y=t['x_mm'],t['y_mm']
    fp.append(f'(fp_circle (center {n(x)} {n(y)}) (end {n(x+0.25)} {n(y)}) (stroke (width 0.1) (type solid)) (fill solid) (layer "F.SilkS"))')
    fp.append(f'(fp_text user "{t["label"]}" (at {n(x)} {n(y+1.2)}) (layer "F.SilkS") (effects (font (size 0.8 0.8) (thickness 0.12))))')
fp.append('(fp_text user "SCREW TERMINALS" (at 13.865 -0.8) (layer "F.SilkS") (effects (font (size 0.8 0.8) (thickness 0.12))))')
fp.append(')')
(pretty/f'{NAME}.kicad_mod').write_text('\n'.join(fp)+'\n',encoding='utf-8')
sym=[f'(symbol "{NAME}" (in_bom no) (on_board yes) (pin_names (offset 1.016))',
    '(property "Reference" "A" (at 0 12.7 0) (effects (font (size 1.27 1.27))))',
    '(property "Value" "FWGPS_Tracker_v15" (at 0 10.16 0) (effects (font (size 1.27 1.27))))',
    f'(property "Footprint" "Witness_Modules:{NAME}" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))',
    '(property "Description" "FWGPS mechanical template. Two generic terminal pins numbered left to right, silkscreen-only terminal centers, and two unplated mounting drills. Mounting holes shown as graphical circles. No electrical terminal pads or manufacturer pin assignments." (at 0 0 0) (effects (font (size 1.27 1.27)) hide))',
    f'(symbol "{NAME}_0_1" (rectangle (start -15.24 7.62) (end 15.24 -7.62) (stroke (width 0.254) (type default)) (fill (type background)))']
for m in mounts:
    x,y=m['x_mm']*0.6,-m['y_mm']*0.6
    sym.append(f'(circle (center {n(x)} {n(y)}) (radius {n(m["diameter_mm"]*0.3)}) (stroke (width 0.2) (type default)) (fill (type none)))')
    sym.append(f'(text "{m["label"]}" (at {n(x)} {n(y-2)} 0) (effects (font (size 1 1))))')
sym.append(f') (symbol "{NAME}_1_1"')
for j in (1,2):
    y=2.54 if j==1 else 0
    sym.append(f'(pin passive line (at 20.32 {y} 180) (length 5.08) (name "PIN_{j}" (effects (font (size 1.016 1.016)))) (number "{j}" (effects (font (size 1.016 1.016)))))')
sym.append('))')
original=LIB.read_text(encoding='utf-8')
assert f'(symbol "{NAME}"' not in original
(OUT/'Witness_Modules.kicad_sym').write_text(original.rstrip()[:-1]+'\n'+'\n'.join(sym)+'\n)\n',encoding='utf-8')
metadata=dict(source=str(SOURCE),view='unmirrored top view: STEP +X right and +Y up',origin_xy_mm=origin,mounting_holes=mounts,terminal_markers=terminals,terminal_numbering='1 then 2 from left to right',outline_mm=outline,previous_library_sha256=digest(LIB))
(OUT/f'{NAME}_geometry.json').write_text(json.dumps(metadata,indent=2)+'\n',encoding='utf-8')
loaded=pcb.FootprintLoad(str(pretty.resolve()),NAME)
assert loaded and loaded.GetPadCount()==2
assert {pcb.ToMM(p.GetPosition()) for p in loaded.Pads()}=={(m['x_mm'],m['y_mm']) for m in mounts}
for p in loaded.Pads():
    assert p.GetAttribute()==pcb.PAD_ATTRIB_NPTH and p.GetNumber()==''
    assert abs(pcb.ToMM(p.GetDrillSize())[0]-3.2512)<1e-6
assert sum(isinstance(g,pcb.PCB_SHAPE) and g.GetShape()==pcb.SHAPE_T_CIRCLE and g.GetLayer()==pcb.F_SilkS for g in loaded.GraphicalItems())==2
print('Validated 2 mounting drills at 3.2512 mm and 2 terminal center dots; exact PCB outline.')
print(json.dumps(dict(mounts=mounts,terminals=terminals),indent=2))
