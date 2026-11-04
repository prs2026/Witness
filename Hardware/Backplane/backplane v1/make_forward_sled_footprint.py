"""Convert the forward sled DXF into a KiCad outline and hole-center template."""
from pathlib import Path
import collections
import math
import json
import sys
import hashlib
import shutil

SOURCE=Path(r'C:\Users\Ejfso\Documents\GitHub\witness\Hardware\R2\IRIS R2\1101 - Nosecone - 1101005 - Foward Sled.dxf')
lines=SOURCE.read_text().splitlines()
assert len(lines)%2==0
pairs=[(int(lines[i].strip()),lines[i+1].strip()) for i in range(0,len(lines),2)]
entities=[]
section=None
current=None
for j,(code,value) in enumerate(pairs):
    if code==0 and value=='SECTION':
        section=pairs[j+1][1]
    if code==0 and value=='ENDSEC':
        section=None
    if section=='ENTITIES' and code==0:
        current={'type':value,'data':[]}
        entities.append(current)
    elif section=='ENTITIES' and current is not None:
        current['data'].append((code,value))
def values(e,code):
    return [v for c,v in e['data'] if c==code]
def val(e,code,default=None):
    a=values(e,code)
    return a[0] if a else default
if '--inspect' in sys.argv:
    print('HEADER UNITS')
    for j,p in enumerate(pairs):
        if p[1] in ('$INSUNITS','$MEASUREMENT','$EXTMIN','$EXTMAX'):
            print(p,pairs[j+1:j+4])
    print('COUNTS',collections.Counter(e['type'] for e in entities))
    for e in entities:
        print(e)
    sys.exit()

NAME='Forward_Sled_Outline'
DEST=Path(r'C:\Users\Ejfso\Documents\GitHub\witness\Hardware\KicadLibs')
OUT=Path(__file__).parent/'forward_sled_output'
if '--install' in sys.argv:
    targets=[Path(f'Witness_Modules.pretty/{NAME}.kicad_mod'),Path(f'{NAME}_geometry.json')]
    for t in targets:
        assert not (DEST/t).exists(),f'Refusing to overwrite {DEST/t}'
    for t in targets:
        shutil.copy2(OUT/t,DEST/t)
    print('Installed',DEST/targets[0])
    sys.exit()

# The DXF declares imperial measurement, and its 2-inch end and 1-inch tab
# establish the inch scale. No $INSUNITS value is present in this legacy DXF.
measurement=next(pairs[j+1][1] for j,p in enumerate(pairs) if p[1]=='$MEASUREMENT')
assert measurement=='0'
SCALE=25.4
line_entities=[e for e in entities if e['type']=='LINE']
circle_entities=[e for e in entities if e['type']=='CIRCLE']
assert len(line_entities)==20 and len(circle_entities)==12
assert all(e['type'] in ('SECTION','LINE','CIRCLE') for e in entities)
segments=[]
for e in line_entities:
    a=(float(val(e,10)),float(val(e,20)))
    b=(float(val(e,11)),float(val(e,21)))
    assert abs(float(val(e,30,'0')))<1e-10 and abs(float(val(e,31,'0')))<1e-10
    segments.append((a,b))
pts=[p for a,b in segments for p in (a,b)]
xmin,xmax=min(p[0] for p in pts),max(p[0] for p in pts)
ymin,ymax=min(p[1] for p in pts),max(p[1] for p in pts)
origin=((xmin+xmax)/2,(ymin+ymax)/2)
def xy(p):
    return (round((p[0]-origin[0])*SCALE,6),round((origin[1]-p[1])*SCALE,6))
def n(v):
    return f'{v:.6f}'.rstrip('0').rstrip('.') or '0'
segments=[(xy(a),xy(b)) for a,b in segments]
# Confirm this is one closed perimeter rather than several unconnected lines.
adj=collections.defaultdict(list)
for a,b in segments:
    assert a!=b
    adj[a].append(b)
    adj[b].append(a)
assert all(len(v)==2 for v in adj.values()),'DXF outline is not closed'
seen=set()
pending=[next(iter(adj))]
while pending:
    p=pending.pop()
    if p not in seen:
        seen.add(p)
        pending.extend(adj[p])
assert len(seen)==len(adj)==20
holes=[]
for e in circle_entities:
    center=(float(val(e,10)),float(val(e,20)))
    x,y=xy(center)
    holes.append(dict(x_mm=x,y_mm=y,source_diameter_mm=round(float(val(e,40))*2*SCALE,6)))
holes.sort(key=lambda h:(h['y_mm'],h['x_mm']))
for j,h in enumerate(holes,1):
    h['label']=f'H{j}'
OUT.mkdir(exist_ok=True)
pretty=OUT/'Witness_Modules.pretty'
pretty.mkdir(exist_ok=True)
height=(ymax-ymin)*SCALE
width=(xmax-xmin)*SCALE
fp=[f'(footprint "{NAME}" (version 20241229) (generator "pcbnew") (layer "F.Cu")',
    '(descr "Forward sled PCB outline from imperial DXF. Closed perimeter on Edge.Cuts; all twelve DXF holes represented only by silkscreen center dots. No pads or drilled holes.") (attr board_only exclude_from_pos_files exclude_from_bom)',
    f'(property "Reference" "REF**" (at 0 {n(-height/2-2)} 0) (layer "F.SilkS") hide (effects (font (size 1 1) (thickness 0.15))))',
    f'(property "Value" "{NAME}" (at 0 {n(height/2+2)} 0) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))']
for layer in ('Edge.Cuts','F.Fab'):
    for a,b in segments:
        fp.append(f'(fp_line (start {n(a[0])} {n(a[1])}) (end {n(b[0])} {n(b[1])}) (stroke (width 0.05) (type solid)) (layer "{layer}"))')
for h in holes:
    x,y=h['x_mm'],h['y_mm']
    fp.append(f'(fp_circle (center {n(x)} {n(y)}) (end {n(x+0.25)} {n(y)}) (stroke (width 0.1) (type solid)) (fill solid) (layer "F.SilkS"))')
fp.append(')')
(pretty/f'{NAME}.kicad_mod').write_text('\n'.join(fp)+'\n',encoding='utf-8')
metadata=dict(source=str(SOURCE),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),source_units='inches; DXF $MEASUREMENT=0; $INSUNITS absent',scale_to_mm=SCALE,origin_source_inches=origin,view='DXF X right and Y up; KiCad Y down',width_mm=width,height_mm=height,outline_layer='Edge.Cuts',outline_segments_mm=segments,hole_center_dots=holes,drills=False,pads=False)
(OUT/f'{NAME}_geometry.json').write_text(json.dumps(metadata,indent=2)+'\n',encoding='utf-8')
import pcbnew as pcb
loaded=pcb.FootprintLoad(str(pretty.resolve()),NAME)
assert loaded and loaded.GetPadCount()==0
items=list(loaded.GraphicalItems())
assert sum(isinstance(g,pcb.PCB_SHAPE) and g.GetLayer()==pcb.Edge_Cuts for g in items)==20
assert sum(isinstance(g,pcb.PCB_SHAPE) and g.GetShape()==pcb.SHAPE_T_CIRCLE and g.GetLayer()==pcb.F_SilkS for g in items)==12
preview=pcb.BOARD()
preview.Add(loaded)
loaded.SetPosition(pcb.VECTOR2I(pcb.FromMM(100),pcb.FromMM(100)))
pcb.SaveBoard(str(OUT/'Forward_Sled_Outline_preview.kicad_pcb'),preview)
print(f'Validated one closed 20-segment perimeter, {width:.3f} x {height:.3f} mm, and twelve silkscreen hole dots. Zero pads or drills.')
