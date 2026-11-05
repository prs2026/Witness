"""Refresh the installed module footprint from the latest breakout PCB."""
from pathlib import Path
import hashlib,json,math,re,shutil,sys
from datetime import datetime
import pcbnew as pcb

ROOT=Path(__file__).parent
SOURCE=Path(r'C:\Users\Ejfso\Documents\GitHub\witness\Hardware\Breakouts\ubloxm8breakout\ubloxm8breakout.kicad_pcb')
DEST=Path(r'C:\Users\Ejfso\Documents\GitHub\witness\Hardware\KicadLibs')
OUT=ROOT/'ublox_m8_updated_output'
NAME='UBlox_M8_Breakout'
targets=[Path(f'Witness_Modules.pretty/{NAME}.kicad_mod'),Path(f'{NAME}_pinmap.json')]
def digest(p): return hashlib.sha256(p.read_bytes()).hexdigest()
if '--install' in sys.argv:
    staged=json.loads((OUT/'update_manifest.json').read_text())
    assert digest(SOURCE)==staged['source_sha256'],'Source board changed since extraction'
    for rel in targets: assert digest(DEST/rel)==staged['original_hashes'][str(rel)],f'Library changed: {rel}'
    stamp=datetime.now().strftime('%Y%m%d-%H%M%S')
    for rel in targets:
        backup=Path(str(DEST/rel)+f'.before-update-{stamp}.bak')
        assert not backup.exists()
        shutil.copy2(DEST/rel,backup)
        shutil.copy2(OUT/rel,DEST/rel)
    print('Updated UBlox_M8_Breakout footprint and pin map; previous versions backed up. Symbol library unchanged.')
    sys.exit()

board=pcb.LoadBoard(str(SOURCE))
previous=json.loads((DEST/targets[1]).read_text())
origin=previous['origin_xy_mm']
assert previous['view']=='source board top view, unmirrored'
def xy(p):
    x,y=pcb.ToMM(p.GetPosition()); return round(x-origin[0],6),round(y-origin[1],6)
def n(v): return f'{v:.6f}'.rstrip('0').rstrip('.') or '0'
connector=next(f for f in board.GetFootprints() if f.GetReference()=='J1')
pins=[]; guides=[]; mounts=[]
for p in connector.Pads():
    x,y=xy(p)
    if p.GetNumber():
        pins.append(dict(pin=p.GetNumber(),name=p.GetNetname().rsplit('/',1)[-1],x_mm=x,y_mm=y,size_mm=pcb.ToMM(p.GetSize()),drill_mm=pcb.ToMM(p.GetDrillSize())[0],rotation_deg=p.GetOrientationDegrees(),shape='rect' if p.GetShape()==pcb.PAD_SHAPE_RECT else 'circle'))
    elif p.GetAttribute()==pcb.PAD_ATTRIB_NPTH:
        guides.append(dict(x_mm=x,y_mm=y,source_drill_mm=pcb.ToMM(p.GetDrillSize())[0]))
pins.sort(key=lambda p:int(p['pin']))
assert [(p['pin'],p['name']) for p in pins]==[(p['pin'],p['name']) for p in previous['pins']]
for f in sorted(board.GetFootprints(),key=lambda f:f.GetReference()):
    if f.GetReference() in ('H1','H2'):
        p=next(iter(f.Pads())); x,y=xy(p)
        mounts.append(dict(label=f.GetReference(),x_mm=x,y_mm=y,drill_mm=pcb.ToMM(p.GetDrillSize())[0]))
assert len(mounts)==2
edge=next(d for d in board.GetDrawings() if d.GetLayer()==pcb.Edge_Cuts)
assert edge.GetShape()==pcb.SHAPE_T_RECT
start=pcb.ToMM(edge.GetStart()); end=pcb.ToMM(edge.GetEnd())
x0,y0=start[0]-origin[0],start[1]-origin[1]
x1,y1=end[0]-origin[0],end[1]-origin[1]
source_text=SOURCE.read_text()
radius=float(re.search(r'\(gr_rect\s*\(start[^)]*\)\s*\(end[^)]*\)\s*\(radius\s+([\d.]+)\)',source_text)[1])
r=radius; d=r/math.sqrt(2)
lines=[((x0+r,y0),(x1-r,y0)),((x1,y0+r),(x1,y1-r)),((x1-r,y1),(x0+r,y1)),((x0,y1-r),(x0,y0+r))]
arcs=[((x1-r,y0),(x1-r+d,y0+r-d),(x1,y0+r)),((x1,y1-r),(x1-r+d,y1-r+d),(x1-r,y1)),((x0+r,y1),(x0+r-d,y1-r+d),(x0,y1-r)),((x0,y0+r),(x0+r-d,y0+r-d),(x0+r,y0))]
OUT.mkdir(exist_ok=True); pretty=OUT/'Witness_Modules.pretty'; pretty.mkdir(exist_ok=True)
fp=[f'(footprint "{NAME}" (version 20241229) (generator "pcbnew") (layer "F.Cu")',
    '(descr "Updated NEO-M8N breakout module: 26x16.5 mm R2 outline. J1 pins 1 +3V3, 2 RX, 3 TX, 4 GND; 0.6604 mm terminal drills. Two 2.2 mm mounting drills. Source top view; original module origin retained.") (attr through_hole)',
    '(property "Reference" "A**" (at 0 -10.5) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))',
    '(property "Value" "UBlox_M8_Breakout" (at 0 10.5) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))']
for layer,width in [('F.SilkS',0.12),('F.Fab',0.1)]:
    for a,b in lines:
        fp.append(f'(fp_line (start {n(a[0])} {n(a[1])}) (end {n(b[0])} {n(b[1])}) (stroke (width {width}) (type solid)) (layer "{layer}"))')
    for a,m,b in arcs:
        fp.append(f'(fp_arc (start {n(a[0])} {n(a[1])}) (mid {n(m[0])} {n(m[1])}) (end {n(b[0])} {n(b[1])}) (stroke (width {width}) (type solid)) (layer "{layer}"))')
fp.append(f'(fp_rect (start {n(x0-.5)} {n(y0-.5)}) (end {n(x1+.5)} {n(y1+.5)}) (stroke (width 0.05) (type solid)) (fill none) (layer "F.CrtYd"))')
for p in pins:
    sx,sy=p['size_mm']
    fp.append(f'(pad "{p["pin"]}" thru_hole {p["shape"]} (at {n(p["x_mm"])} {n(p["y_mm"])} {n(p["rotation_deg"])}) (size {n(sx)} {n(sy)}) (drill {n(p["drill_mm"])}) (layers "*.Cu" "*.Mask"))')
fp.append('(fp_text user "J1" (at 4.5 0) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))')
for m in mounts:
    x,y,drill=m['x_mm'],m['y_mm'],m['drill_mm']
    fp.append(f'(pad "" np_thru_hole circle (at {n(x)} {n(y)}) (size {n(drill)} {n(drill)}) (drill {n(drill)}) (layers "*.Cu" "*.Mask"))')
# The new connector's guide bores belong to the module itself; document their
# positions in fabrication graphics without introducing host mounting drills.
for g in guides:
    x,y,r=g['x_mm'],g['y_mm'],g['source_drill_mm']/2
    fp.append(f'(fp_circle (center {n(x)} {n(y)}) (end {n(x+r)} {n(y)}) (stroke (width 0.1) (type solid)) (fill none) (layer "F.Fab"))')
fp.append(')')
(OUT/targets[0]).write_text('\n'.join(fp)+'\n',encoding='utf-8')
meta=dict(source=str(SOURCE),source_sha256=digest(SOURCE),origin_xy_mm=origin,view=previous['view'],pins=pins,mounting_holes=mounts,connector_guide_holes_reference_only=guides,board_outline_mm=dict(start=[x0,y0],end=[x1,y1],radius=radius))
(OUT/targets[1]).write_text(json.dumps(meta,indent=2)+'\n',encoding='utf-8')
(OUT/'update_manifest.json').write_text(json.dumps(dict(source_sha256=digest(SOURCE),original_hashes={str(rel):digest(DEST/rel) for rel in targets}),indent=2)+'\n',encoding='utf-8')
loaded=pcb.FootprintLoad(str(pretty.resolve()),NAME)
assert loaded and loaded.GetPadCount()==6
for p in loaded.Pads():
    x,y=pcb.ToMM(p.GetPosition())
    candidates=[v for v in (pins if p.GetNumber() else mounts) if (not p.GetNumber() or v['pin']==p.GetNumber()) and abs(v['x_mm']-x)<1e-6 and abs(v['y_mm']-y)<1e-6]
    assert len(candidates)==1
    assert abs(pcb.ToMM(p.GetDrillSize())[0]-candidates[0]['drill_mm'])<1e-6
    if not p.GetNumber(): assert p.GetAttribute()==pcb.PAD_ATTRIB_NPTH
assert next(m for m in mounts if m['label']=='H1')==next(m for m in previous['mounting_holes'] if m['label']=='H1')
assert abs(x1-x0-26)<1e-6 and abs(y1-y0-16.5)<1e-6
print('Validated updated 26 x 16.5 mm outline, four source-matched connector pads and two 2.2 mm mounting holes. Origin and pinout retained.')
