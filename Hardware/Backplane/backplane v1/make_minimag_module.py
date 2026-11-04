"""Recover the miniMagSwitch module interface from EasyEDA and fabrication data."""
import base64
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import sqlite3
import sys
import zipfile
import pcbnew as pcb

ROOT = Path(r'C:\Users\Ejfso\Documents\GitHub\miniMagSwitch')
DEST = Path(r'C:\Users\Ejfso\Documents\GitHub\witness\Hardware\KicadLibs')
OUT = Path(__file__).parent / 'minimag_module_output'
LIB = DEST / 'Witness_Modules.kicad_sym'
def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

if '--install' in sys.argv:
    info=json.loads((OUT/'miniMagSwitch_pinmap.json').read_text())
    assert digest(LIB)==info['previous_library_sha256'], 'Library changed since generation'
    fp=DEST/'Witness_Modules.pretty/miniMagSwitch.kicad_mod'
    backup=DEST/'Witness_Modules.before-miniMagSwitch.kicad_sym'
    for path in (fp,backup,DEST/'miniMagSwitch_pinmap.json'):
        assert not path.exists(), f'Refusing to overwrite {path}'
    shutil.copy2(LIB,backup)
    shutil.copy2(OUT/'Witness_Modules.pretty/miniMagSwitch.kicad_mod',fp)
    shutil.copy2(OUT/'miniMagSwitch_pinmap.json',DEST/'miniMagSwitch_pinmap.json')
    shutil.copy2(OUT/'Witness_Modules.kicad_sym',LIB)
    print('Installed miniMagSwitch symbol and footprint. Previous symbol library backed up:',backup)
    sys.exit()

con=sqlite3.connect((ROOT/'miniMagSwitch.eprj').as_uri()+'?mode=ro',uri=True)
def decode(data):
    return gzip.decompress(base64.b64decode(data[6:])).decode() if data.startswith('base64') else data
def records(data):
    return [json.loads(line) for line in decode(data).splitlines() if line.strip()]
data=con.execute('select dataStr from documents where docType=3').fetchone()[0]
rec=records(data)
designators={r[3]:r[8] for r in rec if r[0]=='ATTR' and r[7]=='Designator'}
devices={r[3]:r[8] for r in rec if r[0]=='ATTR' and r[7]=='Device'}
nets={(r[1],r[2]):r[3] for r in rec if r[0]=='PAD_NET'}
# Coordinates in mil, with source Y pointing up. Footprint is an unmirrored top view.
origin=(230*0.0254,205*0.0254)
def transform(x,y):
    return (round(x-origin[0],6),round(-y-origin[1],6))
pins=[]
for c in rec:
    if c[0]!='COMPONENT' or designators.get(c[1]) not in ('H1','H2'):
        continue
    cid=c[1]
    fp_uuid=con.execute("select value from attributes where device_uuid=? and key='Footprint'",(devices[cid],)).fetchone()[0]
    fp_rec=records(con.execute('select dataStr from components where uuid=?',(fp_uuid,)).fetchone()[0])
    a=math.radians(c[6])
    for p in fp_rec:
        if p[0]!='PAD':
            continue
        x=(c[4]+p[6]*math.cos(a)-p[7]*math.sin(a))*0.0254
        y=(c[5]+p[6]*math.sin(a)+p[7]*math.cos(a))*0.0254
        px,py=transform(x,y)
        pins.append(dict(pin=f'{designators[cid]}_{p[5]}',connector=designators[cid],contact=p[5],signal=nets[cid,p[5]],x_mm=px,y_mm=py,source_x_mm=round(x,5),source_y_mm=round(y,5),size_mm=round(p[10][1]*0.0254,5),drill_mm=1.0))
pins.sort(key=lambda p:p['pin'])
with zipfile.ZipFile(ROOT/'fabrication/miniMagSwitch.zip') as z:
    outline_raw=z.read('Gerber_BoardOutlineLayer.GKO').decode()
    outline=[transform(int(x)/100000,int(y)/100000) for x,y in re.findall(r'X(-?\d+)Y(-?\d+)D0[12]',outline_raw)]
    pth=z.read('Drill_PTH_Through.DRL').decode().split('T02\n')[-1]
    drill_positions={(float(x),float(y)) for x,y in re.findall(r'X(-?[\d.]+)Y(-?[\d.]+)',pth)}
    assert {(p['source_x_mm'],p['source_y_mm']) for p in pins}==drill_positions
    npth=z.read('Drill_NPTH_Through.DRL').decode()
    hole_diameter=float(re.search(r'T01C([\d.]+)',npth)[1])
    holes=[transform(float(x),float(y)) for x,y in re.findall(r'X(-?[\d.]+)Y(-?[\d.]+)',npth)]
assert len(pins)==4 and len(holes)==2 and outline[0]==outline[-1]
OUT.mkdir(exist_ok=True)
pretty=OUT/'Witness_Modules.pretty'
pretty.mkdir(exist_ok=True)
def n(v):
    return f'{v:.6f}'.rstrip('0').rstrip('.') or '0'
fp=['(footprint "miniMagSwitch" (version 20241229) (generator "pcbnew") (layer "F.Cu")',
    '(descr "miniMagSwitch module: H2 input, H1 switched output, two unplated 4-40 mounting holes; source board top view") (tags "miniMagSwitch magnetic power switch module") (attr through_hole)',
    '(property "Reference" "A**" (at 0 -7) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))',
    '(property "Value" "miniMagSwitch" (at 0 7) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))']
for layer,width in [('F.Fab',0.1),('F.SilkS',0.12)]:
    for a,b in zip(outline,outline[1:]):
        fp.append(f'(fp_line (start {n(a[0])} {n(a[1])}) (end {n(b[0])} {n(b[1])}) (stroke (width {width}) (type solid)) (layer "{layer}"))')
for p in pins:
    shape='rect' if p['contact']=='1' else 'circle'
    fp.append(f'(pad "{p["pin"]}" thru_hole {shape} (at {n(p["x_mm"])} {n(p["y_mm"])}) (size {p["size_mm"]} {p["size_mm"]}) (drill 1) (layers "*.Cu" "*.Mask"))')
for x,y in holes:
    fp.append(f'(pad "" np_thru_hole circle (at {n(x)} {n(y)}) (size {hole_diameter} {hole_diameter}) (drill {hole_diameter}) (layers "*.Cu" "*.Mask"))')
fp.append('(fp_rect (start -6.35 -5.75) (end 6.35 5.75) (stroke (width 0.05) (type solid)) (fill none) (layer "F.CrtYd"))')
fp.append(')')
(pretty/'miniMagSwitch.kicad_mod').write_text('\n'.join(fp)+'\n',encoding='utf-8')
sym=['(symbol "miniMagSwitch" (pin_names (offset 1.016)) (in_bom yes) (on_board yes)',
     '(property "Reference" "A" (at 0 10.16 0) (effects (font (size 1.27 1.27))))',
     '(property "Value" "miniMagSwitch" (at 0 7.62 0) (effects (font (size 1.27 1.27))))',
     '(property "Footprint" "Witness_Modules:miniMagSwitch" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))',
     '(property "Description" "Magnetic power switch module. H2 input (VCC/GND), H1 output (SWITCH_OUT/GND). Two isolated 4-40 mounting holes." (at 0 0 0) (effects (font (size 1.27 1.27)) hide))',
     '(symbol "miniMagSwitch_0_1" (rectangle (start -15.24 5.08) (end 15.24 -7.62) (stroke (width 0.254) (type default)) (fill (type background))))',
     '(symbol "miniMagSwitch_1_1"']
for p in pins:
    x,angle=(-20.32,0) if p['connector']=='H2' else (20.32,180)
    y=2.54 if p['contact']=='1' else -5.08
    label={'H2_1':'VIN','H2_2':'GND_IN','H1_1':'VOUT_SWITCHED','H1_2':'GND_OUT'}[p['pin']]
    sym.append(f'(pin passive line (at {x} {y} {angle}) (length 5.08) (name "{label}" (effects (font (size 1.016 1.016)))) (number "{p["pin"]}" (effects (font (size 1.016 1.016)))))')
sym.append('))')
original=LIB.read_text(encoding='utf-8')
assert '(symbol "miniMagSwitch"' not in original
assert original.rstrip().endswith(')')
combined=original.rstrip()[:-1]+'\n'+'\n'.join(sym)+'\n)\n'
(OUT/'Witness_Modules.kicad_sym').write_text(combined,encoding='utf-8')
(OUT/'miniMagSwitch_pinmap.json').write_text(json.dumps(dict(source=str(ROOT),view='top; no mirroring',origin_mm=origin,pins=pins,mounting_holes_mm=holes,mounting_drill_mm=hole_diameter,previous_library_sha256=digest(LIB)),indent=2)+'\n',encoding='utf-8')
loaded=pcb.FootprintLoad(str(pretty.resolve()),'miniMagSwitch')
assert loaded and loaded.GetPadCount()==6
for p in loaded.Pads():
    if not p.GetNumber():
        assert p.GetAttribute()==pcb.PAD_ATTRIB_NPTH
        assert abs(pcb.ToMM(p.GetDrillSize())[0]-hole_diameter)<1e-6
    else:
        r=next(r for r in pins if r['pin']==p.GetNumber())
        x,y=pcb.ToMM(p.GetPosition())
        assert abs(x-r['x_mm'])<1e-6 and abs(y-r['y_mm'])<1e-6
print('Validated 4 I/O pads against source nets and fabrication drill coordinates; 2 NPTH mounting holes.')
print(json.dumps(pins,indent=2))
