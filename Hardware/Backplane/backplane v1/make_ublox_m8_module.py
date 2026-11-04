"""Create a u-blox M8 breakout module symbol from the source PCB pinout."""
from pathlib import Path
import sys
import json
import hashlib
import shutil
import math
import pcbnew as pcb

SOURCE=Path(r'C:\Users\Ejfso\Documents\GitHub\witness\Hardware\Breakouts\ubloxm8breakout\ubloxm8breakout.kicad_pcb')
board=pcb.LoadBoard(str(SOURCE))
if '--inspect' in sys.argv:
    for f in board.GetFootprints():
        print('FOOTPRINT',f.GetReference(),f.GetValue(),f.GetFPID().GetLibItemName(),'position',pcb.ToMM(f.GetPosition()))
        if f.GetReference().startswith(('J','H','P')) or f.GetReference() in ('U1','R2','R3'):
            for p in f.Pads():
                print('PAD',p.GetNumber(),'xy',pcb.ToMM(p.GetPosition()),'size',pcb.ToMM(p.GetSize()),'drill',pcb.ToMM(p.GetDrillSize()),'net',p.GetNetname())
    for d in board.GetDrawings():
        if d.GetLayer()==pcb.Edge_Cuts:
            print('EDGE',d.GetShapeStr(),pcb.ToMM(d.GetStart()),pcb.ToMM(d.GetEnd()))
    sys.exit()

DEST=Path(r'C:\Users\Ejfso\Documents\GitHub\witness\Hardware\KicadLibs')
OUT=Path(__file__).parent/'ublox_m8_module_output'
LIB=DEST/'Witness_Modules.kicad_sym'
NAME='UBlox_M8_Breakout'
def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()
if '--install' in sys.argv:
    metadata=json.loads((OUT/f'{NAME}_pinmap.json').read_text())
    assert digest(LIB)==metadata['previous_library_sha256']
    backup=DEST/'Witness_Modules.before-UBlox-M8.kicad_sym'
    targets=[Path(f'Witness_Modules.pretty/{NAME}.kicad_mod'),Path(f'{NAME}_pinmap.json')]
    for p in [backup]+[DEST/t for t in targets]:
        assert not p.exists(),f'Refusing to overwrite {p}'
    shutil.copy2(LIB,backup)
    for t in targets:
        shutil.copy2(OUT/t,DEST/t)
    shutil.copy2(OUT/'Witness_Modules.kicad_sym',LIB)
    print('Installed',NAME,'symbol and linked footprint, preserving a library backup.')
    sys.exit()
origin=(143.75,90.5)
def xy(p):
    x,y=pcb.ToMM(p.GetPosition())
    return round(x-origin[0],6),round(y-origin[1],6)
def n(v):
    return f'{v:.6f}'.rstrip('0').rstrip('.') or '0'
connector=next(f for f in board.GetFootprints() if f.GetReference()=='J1')
holes=sorted((f for f in board.GetFootprints() if f.GetReference() in ('H1','H2')),key=lambda f:f.GetReference())
pins=[]
for p in sorted(connector.Pads(),key=lambda p:int(p.GetNumber())):
    x,y=xy(p)
    pins.append(dict(pin=p.GetNumber(),name=p.GetNetname().rsplit('/',1)[-1],x_mm=x,y_mm=y,size_mm=pcb.ToMM(p.GetSize()),drill_mm=pcb.ToMM(p.GetDrillSize())[0],rotation_deg=p.GetOrientationDegrees()))
assert [p['name'] for p in pins]==['+3V3','RX','TX','GND']
mounts=[]
for f in holes:
    p=next(iter(f.Pads()))
    x,y=xy(p)
    mounts.append(dict(label=f.GetReference(),x_mm=x,y_mm=y,drill_mm=pcb.ToMM(p.GetDrillSize())[0]))
OUT.mkdir(exist_ok=True)
pretty=OUT/'Witness_Modules.pretty'
pretty.mkdir(exist_ok=True)
fp=[f'(footprint "{NAME}" (version 20241229) (generator "pcbnew") (layer "F.Cu")',
    '(descr "NEO-M8N breakout module. J1: 1 +3V3, 2 RX, 3 TX, 4 GND. Exact Micro-Lock pad coordinates; 29x17 mm R2 board outline and two 2.2 mm NPTH mounting holes. Unmirrored source top view.") (attr through_hole)',
    '(property "Reference" "A**" (at 0 -10.5) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))',
    '(property "Value" "UBlox_M8_Breakout" (at 0 10.5) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))']
lines=[((-12.5,-8.5),(12.5,-8.5)),((14.5,-6.5),(14.5,6.5)),((12.5,8.5),(-12.5,8.5)),((-14.5,6.5),(-14.5,-6.5))]
d=2/math.sqrt(2)
arcs=[((12.5,-8.5),(12.5+d,-6.5-d),(14.5,-6.5)),((14.5,6.5),(12.5+d,6.5+d),(12.5,8.5)),((-12.5,8.5),(-12.5-d,6.5+d),(-14.5,6.5)),((-14.5,-6.5),(-12.5-d,-6.5-d),(-12.5,-8.5))]
for layer,width in [('F.SilkS',0.12),('F.Fab',0.1)]:
    for a,b in lines:
        fp.append(f'(fp_line (start {n(a[0])} {n(a[1])}) (end {n(b[0])} {n(b[1])}) (stroke (width {width}) (type solid)) (layer "{layer}"))')
    for a,m,b in arcs:
        fp.append(f'(fp_arc (start {n(a[0])} {n(a[1])}) (mid {n(m[0])} {n(m[1])}) (end {n(b[0])} {n(b[1])}) (stroke (width {width}) (type solid)) (layer "{layer}"))')
fp.append('(fp_rect (start -15 -9) (end 15 9) (stroke (width 0.05) (type solid)) (fill none) (layer "F.CrtYd"))')
for p in pins:
    shape='rect' if p['pin']=='1' else 'oval'
    sx,sy=p['size_mm']
    fp.append(f'(pad "{p["pin"]}" thru_hole {shape} (at {n(p["x_mm"])} {n(p["y_mm"])} {n(p["rotation_deg"])}) (size {n(sx)} {n(sy)}) (drill {n(p["drill_mm"])}) (layers "*.Cu" "*.Mask"))')
fp.append('(fp_text user "J1" (at 7 0) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))')
for m in mounts:
    x,y,drill=m['x_mm'],m['y_mm'],m['drill_mm']
    fp.append(f'(pad "" np_thru_hole circle (at {n(x)} {n(y)}) (size {n(drill)} {n(drill)}) (drill {n(drill)}) (layers "*.Cu" "*.Mask"))')
fp.append(')')
(pretty/f'{NAME}.kicad_mod').write_text('\n'.join(fp)+'\n',encoding='utf-8')
sym=[f'(symbol "{NAME}" (pin_names (offset 1.016)) (in_bom yes) (on_board yes)',
    '(property "Reference" "A" (at 0 12.7 0) (effects (font (size 1.27 1.27))))',
    f'(property "Value" "{NAME}" (at 0 10.16 0) (effects (font (size 1.27 1.27))))',
    f'(property "Footprint" "Witness_Modules:{NAME}" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))',
    '(property "Description" "u-blox NEO-M8N breakout module: Micro-Lock J1 pins 1 +3V3, 2 RX (module input), 3 TX (module output), 4 GND. Two 2.2 mm mounting holes." (at 0 0 0) (effects (font (size 1.27 1.27)) hide))',
    f'(symbol "{NAME}_0_1" (rectangle (start -12.7 7.62) (end 12.7 -7.62) (stroke (width 0.254) (type default)) (fill (type background)))']
for m in mounts:
    x,y=m['x_mm']*0.65,-m['y_mm']*0.65
    sym.append(f'(circle (center {n(x)} {n(y)}) (radius 0.715) (stroke (width 0.15) (type default)) (fill (type none)))')
sym.append(f') (symbol "{NAME}_1_1"')
positions={'1':(-17.78,2.54,0,'power_in'),'2':(-17.78,0,0,'input'),'3':(17.78,0,180,'output'),'4':(17.78,-2.54,180,'power_in')}
for p in pins:
    x,y,angle,kind=positions[p['pin']]
    sym.append(f'(pin {kind} line (at {x} {y} {angle}) (length 5.08) (name "{p["name"]}" (effects (font (size 1.016 1.016)))) (number "{p["pin"]}" (effects (font (size 1.016 1.016)))))')
sym.append('))')
original=LIB.read_text(encoding='utf-8')
assert f'(symbol "{NAME}"' not in original
(OUT/'Witness_Modules.kicad_sym').write_text(original.rstrip()[:-1]+'\n'+'\n'.join(sym)+'\n)\n',encoding='utf-8')
metadata=dict(source=str(SOURCE),origin_xy_mm=origin,view='source board top view, unmirrored',pins=pins,mounting_holes=mounts,previous_library_sha256=digest(LIB))
(OUT/f'{NAME}_pinmap.json').write_text(json.dumps(metadata,indent=2)+'\n',encoding='utf-8')
loaded=pcb.FootprintLoad(str(pretty.resolve()),NAME)
assert loaded and loaded.GetPadCount()==6
for p in loaded.Pads():
    if p.GetNumber():
        source=next(v for v in pins if v['pin']==p.GetNumber())
        assert pcb.ToMM(p.GetPosition())==(source['x_mm'],source['y_mm'])
        assert pcb.ToMM(p.GetDrillSize())==(0.5,0.5)
    else:
        assert p.GetAttribute()==pcb.PAD_ATTRIB_NPTH
        assert pcb.ToMM(p.GetDrillSize())==(2.2,2.2)
print('Validated four J1 connector pads and two 2.2 mm mounting holes; source pinout:',[(p['pin'],p['name']) for p in pins])
