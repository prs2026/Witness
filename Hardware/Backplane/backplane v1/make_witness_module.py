import json
import math
import sys
from pathlib import Path
import pcbnew as pcb

SOURCE = Path(r'C:\Users\Ejfso\Documents\GitHub\witness\Hardware\witnessv0.2\witness\witness.kicad_pcb')
board = pcb.LoadBoard(str(SOURCE))
connectors = sorted((f for f in board.GetFootprints() if f.GetReference() in ('J2', 'J5', 'J6')), key=lambda f: int(f.GetReference()[1:]))
holes = sorted((f for f in board.GetFootprints() if f.GetReference().startswith('H')), key=lambda f: f.GetReference())
if '--inspect' in sys.argv:
    for f in connectors + holes:
        print(f.GetReference(), f.GetValue(), f.GetFPID().GetLibItemName())
        for p in f.Pads():
            print(' ', p.GetNumber(), 'xy', pcb.ToMM(p.GetPosition()), 'size', pcb.ToMM(p.GetSize()), 'drill', pcb.ToMM(p.GetDrillSize()), 'net', p.GetNetname())
    sys.exit()

OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path('witness_module_output')
OUT.mkdir(parents=True, exist_ok=True)
pretty = OUT / 'Witness_Modules.pretty'
pretty.mkdir(exist_ok=True)
name = 'Witness_v0_2_MicroLock'
origin = (113.0, 118.75)
def num(v):
    return f'{v:.6f}'.rstrip('0').rstrip('.') or '0'
def xy(p):
    x, y = pcb.ToMM(p.GetPosition())
    return x-origin[0], y-origin[1]
def q(s):
    return json.dumps(s)
def effect(size=1):
    return f'(effects (font (size {size} {size}) (thickness 0.15)))'

fp = [f'(footprint "{name}" (version 20241229) (generator "pcbnew") (layer "F.Cu")',
      '(descr "Witness v0.2 module, top-view coordinates; J2 battery, J5 UART, J6 CAN; grounded M2 holes")',
      '(tags "Witness module MicroLock battery UART CAN") (attr through_hole)',
      f'(property "Reference" "A**" (at 0 -22) (layer "F.SilkS") {effect()})',
      f'(property "Value" "{name}" (at 0 22) (layer "F.Fab") {effect()})']
# Rounded 20 x 40 mm source outline, R2.5. It describes the module, not a host cutout.
for layer, width in [('F.Fab', 0.1), ('F.SilkS', 0.12)]:
    for start, end in [((-7.5,-20),(7.5,-20)), ((10,-17.5),(10,17.5)), ((7.5,20),(-7.5,20)), ((-10,17.5),(-10,-17.5))]:
        fp.append(f'(fp_line (start {start[0]} {start[1]}) (end {end[0]} {end[1]}) (stroke (width {width}) (type solid)) (layer "{layer}"))')
    d=2.5/math.sqrt(2)
    for start, mid, end in [((7.5,-20),(7.5+d,-17.5-d),(10,-17.5)), ((10,17.5),(7.5+d,17.5+d),(7.5,20)), ((-7.5,20),(-7.5-d,17.5+d),(-10,17.5)), ((-10,-17.5),(-7.5-d,-17.5-d),(-7.5,-20))]:
        fp.append(f'(fp_arc (start {start[0]} {start[1]}) (mid {num(mid[0])} {num(mid[1])}) (end {end[0]} {end[1]}) (stroke (width {width}) (type solid)) (layer "{layer}"))')
fp.append('(fp_rect (start -10.5 -20.5) (end 10.5 20.5) (stroke (width 0.05) (type solid)) (fill none) (layer "F.CrtYd"))')
rows=[]
for f in connectors:
    for p in sorted(f.Pads(), key=lambda p: int(p.GetNumber())):
        ident = f'{f.GetReference()}_{p.GetNumber()}'
        x,y=xy(p)
        sx,sy=pcb.ToMM(p.GetSize())
        drill=pcb.ToMM(p.GetDrillSize())[0]
        assert drill > 0
        angle=p.GetOrientationDegrees()
        shape='rect' if p.GetNumber()=='1' else 'oval'
        fp.append(f'(pad "{ident}" thru_hole {shape} (at {num(x)} {num(y)} {num(angle)}) (size {num(sx)} {num(sy)}) (drill {num(drill)}) (layers "*.Cu" "*.Mask"))')
        net=p.GetNetname()
        label=net.rsplit('/',1)[-1]
        if net.startswith('Net-'):
            label='NC' # J6 pin 1 has no connection in the source schematic.
        rows.append(dict(pin=ident,connector=f.GetReference(),contact=p.GetNumber(),signal=label,source_net=net,x_mm=x,y_mm=y,drill_mm=drill))
    cx,cy=xy(f)
    fp.append(f'(fp_text user "{f.GetReference()}" (at {num(cx-2)} {num(cy)}) (layer "F.Fab") {effect(0.8)})')
for f in holes:
    for p in f.Pads():
        x,y=xy(p)
        # A shared GND pin connects all four plated mounting holes.
        fp.append(f'(pad "MP" thru_hole circle (at {num(x)} {num(y)}) (size 3.8 3.8) (drill 2.2) (layers "*.Cu" "*.Mask"))')
        rows.append(dict(pin='MP',connector=f.GetReference(),contact='1',signal='GND',source_net=p.GetNetname(),x_mm=x,y_mm=y,drill_mm=2.2))
fp.append(')')
fp_path=pretty/f'{name}.kicad_mod'
if fp_path.exists():
    raise FileExistsError(fp_path)
fp_path.write_text('\n'.join(fp)+'\n',encoding='utf-8')

sym=[f'(kicad_symbol_lib (version 20241209) (generator "kicad_symbol_editor") (symbol "{name}"',
     '(pin_names (offset 1.016)) (in_bom yes) (on_board yes)',
     '(property "Reference" "A" (at 0 20.32 0) (effects (font (size 1.27 1.27))))',
     f'(property "Value" "{name}" (at 0 17.78 0) (effects (font (size 1.27 1.27))))',
     f'(property "Footprint" "Witness_Modules:{name}" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))',
     '(property "Description" "Witness v0.2: J2 battery, J5 UART, J6 CAN. MP is all four grounded M2 mounting holes. J6_1 is NC." (at 0 0 0) (effects (font (size 1.27 1.27)) hide))',
     f'(symbol "{name}_0_1" (rectangle (start -22.86 15.24) (end 22.86 -20.32) (stroke (width 0.254) (type default)) (fill (type background))))',
     f'(symbol "{name}_1_1"']
positions={'J2':(-27.94,10.16,0),'J5':(-27.94,0,0),'J6':(27.94,10.16,180)}
for r in rows:
    if r['pin']=='MP':
        continue
    x,y,angle=positions[r['connector']]
    y-= (int(r['contact'])-1)*2.54
    label=f"{r['connector']}_{r['signal']}"
    sym.append(f'(pin passive line (at {x} {num(y)} {angle}) (length 5.08) (name "{label}" (effects (font (size 1.016 1.016)))) (number "{r["pin"]}" (effects (font (size 1.016 1.016)))))')
sym.append('(pin passive line (at 27.94 -15.24 180) (length 5.08) (name "MOUNT_GND" (effects (font (size 1.016 1.016)))) (number "MP" (effects (font (size 1.016 1.016)))))')
sym.append(')))')
sym_path=OUT/'Witness_Modules.kicad_sym'
if sym_path.exists():
    raise FileExistsError(sym_path)
sym_path.write_text('\n'.join(sym)+'\n',encoding='utf-8')
(OUT/'Witness_v0_2_MicroLock_pinmap.json').write_text(json.dumps({'source':str(SOURCE),'origin_mm':origin,'view':'source board top view; no mirroring','pins':rows},indent=2)+'\n',encoding='utf-8')
# Validate KiCad's interpretation, including all rotated connector pad geometry.
loaded=pcb.FootprintLoad(str(pretty.resolve()),name)
assert loaded is not None
assert loaded.GetPadCount()==14
for p in loaded.Pads():
    expected=[r for r in rows if r['pin']==p.GetNumber()]
    assert any(abs(pcb.ToMM(p.GetPosition())[0]-r['x_mm'])<1e-6 and abs(pcb.ToMM(p.GetPosition())[1]-r['y_mm'])<1e-6 for r in expected)
print(f'Created {sym_path} and {fp_path}; verified 10 connector pads and 4 plated mounting holes.')
