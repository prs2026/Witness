"""AIM XTRA STEP geometry extraction and KiCad location template generation."""
from pathlib import Path
import re
import json
import math
import sys
import hashlib
import shutil
from collections import Counter

SOURCE=Path(__file__).parent/'AIM_XTRA_4.0_No_Antenna - Boss-Extrude16.step'
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
def bound_points(bound):
    return [point(v) for e in edges_of_bound(bound) for v in refs(e)[:2]]
def bound_area(bound):
    pts=bound_points(bound)
    return (max(p[0] for p in pts)-min(p[0] for p in pts))*(max(p[2] for p in pts)-min(p[2] for p in pts))
pcb_face=max((i for i,v in entities.items() if v[0]=='ADVANCED_FACE' and entities[refs(i)[-1]][0]=='PLANE' and abs(axis(refs(refs(i)[-1])[0])[1][1])>0.99),key=lambda i:(max(bound_area(b) for b in refs(i)[:-1]),axis(refs(refs(i)[-1])[0])[0][1]))
outer=max(refs(pcb_face)[:-1],key=bound_area)
if '--outline' in sys.argv:
    if '--bottom' in sys.argv:
        outer=max(refs(1733)[:-1],key=bound_area)
    print('PCB FACE',pcb_face,'OUTER',outer)
    for e in edges_of_bound(outer):
        r=refs(e)
        print(e,point(r[0]),point(r[1]),entities[r[2]])
    sys.exit()

DEST=Path(r'C:\Users\Ejfso\Documents\GitHub\witness\Hardware\KicadLibs')
OUT=Path(__file__).parent/'aim_xtra_module_output'
LIB=DEST/'Witness_Modules.kicad_sym'
NAME='AIM_XTRA_4_0_Location_Template'
def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()
if '--install' in sys.argv:
    metadata=json.loads((OUT/f'{NAME}_geometry.json').read_text())
    assert digest(LIB)==metadata['previous_library_sha256'], 'Library changed since generation'
    target=DEST/f'Witness_Modules.pretty/{NAME}.kicad_mod'
    backup=DEST/'Witness_Modules.before-AIM-XTRA.kicad_sym'
    for p in (target,backup,DEST/f'{NAME}_geometry.json'):
        assert not p.exists(), f'Refusing to overwrite {p}'
    shutil.copy2(LIB,backup)
    shutil.copy2(OUT/f'Witness_Modules.pretty/{NAME}.kicad_mod',target)
    shutil.copy2(OUT/f'{NAME}_geometry.json',DEST/f'{NAME}_geometry.json')
    shutil.copy2(OUT/'Witness_Modules.kicad_sym',LIB)
    print('Installed',NAME,'and preserved a backup of the symbol library.')
    sys.exit()

if '--inspect' not in sys.argv:
    assert 'SI_UNIT($,.METRE.)' in raw
    board_points=bound_points(outer)
    xmin,xmax=min(p[0] for p in board_points),max(p[0] for p in board_points)
    zmin,zmax=min(p[2] for p in board_points),max(p[2] for p in board_points)
    origin=((xmin+xmax)*500,(zmin+zmax)*500)
    def xy(p):
        # Looking toward the PCB from +Y: +X right, +Z down is unmirrored.
        return (round(p[0]*1000-origin[0],6),round(p[2]*1000-origin[1],6))
    def n(v):
        return f'{v:.6f}'.rstrip('0').rstrip('.') or '0'
    bounds=(xy((xmin,0,zmin)),xy((xmax,0,zmax)))
    cylinders=[]
    for i,(kind,body) in entities.items():
        if kind=='CYLINDRICAL_SURFACE':
            center,direction,_=axis(refs(i)[0])
            if abs(direction[1])>0.999:
                cylinders.append((float(body.split(',')[-1])*2000,center,i))
    holes=[(d,p,i) for d,p,i in cylinders if abs(d-3)<1e-6 and (abs(p[0]-(xmin+0.003))<1e-7 or abs(p[0]-(xmax-0.003))<1e-7)]
    assert len(holes)==4
    holes.sort(key=lambda h:(round(h[1][2],6),h[1][0]))
    mounts=[dict(label=f'MH{j}',x_mm=xy(p)[0],y_mm=xy(p)[1],source_diameter_mm=d,step_center_m=p,entity=i) for j,(d,p,i) in enumerate(holes,1)]
    planar=[]
    for i,(kind,body) in entities.items():
        if kind=='ADVANCED_FACE' and entities[refs(i)[-1]][0]=='PLANE':
            pts=[p for b in refs(i)[:-1] for p in bound_points(b)]
            ext=[(min(p[d] for p in pts),max(p[d] for p in pts)) for d in range(3)]
            planar.append((i,ext))
    # The long, highest raised body is the screw terminal strip. Recover its
    # full projection from faces sharing the same X span, including sloped roof.
    terminal_top=max((v for v in planar if v[1][0][1]-v[1][0][0]>0.02 and v[1][0][1]-v[1][0][0]<0.04),key=lambda v:v[1][1][1])
    txmin,txmax=terminal_top[1][0]
    terminal_faces=[v for v in planar if abs(v[1][0][0]-txmin)<1e-8 and abs(v[1][0][1]-txmax)<1e-8 and v[1][1][1]>0.0016+1e-8]
    tzmin=min(v[1][2][0] for v in terminal_faces)
    tzmax=max(v[1][2][1] for v in terminal_faces)
    terminal=(xy((txmin,0,tzmin)),xy((txmax,0,tzmax)))
    assert abs((txmax-txmin)*1000-31)<1e-6
    assert abs((tzmax-tzmin)*1000-6.5)<1e-6
    OUT.mkdir(exist_ok=True)
    pretty=OUT/'Witness_Modules.pretty'
    pretty.mkdir(exist_ok=True)
    def rect(layer,a,b,fill='none',width=0.12):
        return f'(fp_rect (start {n(a[0])} {n(a[1])}) (end {n(b[0])} {n(b[1])}) (stroke (width {width}) (type solid)) (fill {fill}) (layer "{layer}"))'
    def text(label,x,y,layer='F.SilkS',size=1):
        return f'(fp_text user "{label}" (at {n(x)} {n(y)}) (layer "{layer}") (effects (font (size {size} {size}) (thickness 0.15))))'
    fp=[f'(footprint "{NAME}" (version 20241229) (generator "pcbnew") (layer "F.Cu")',
        '(descr "AIM XTRA 4.0 mechanical location template. 100x30 mm PCB envelope, projected screw terminal bar, four mounting-center dots. No pads or drills.") (attr board_only exclude_from_pos_files exclude_from_bom)',
        '(property "Reference" "A**" (at 0 -17.5) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))',
        '(property "Value" "AIM_XTRA_4_0" (at 0 17.5) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))',
        rect('F.SilkS',*bounds),rect('F.Fab',*bounds,width=0.1),rect('F.CrtYd',(-50.5,-15.5),(50.5,15.5),width=0.05),rect('F.SilkS',*terminal,fill='solid'),rect('F.Fab',*terminal,width=0.1)]
    tcx=(terminal[0][0]+terminal[1][0])/2
    fp.append(text('SCREW TERMINALS',tcx,terminal[0][1]-1.25))
    for m in mounts:
        x,y=m['x_mm'],m['y_mm']
        fp.append(f'(fp_circle (center {n(x)} {n(y)}) (end {n(x+0.3)} {n(y)}) (stroke (width 0.1) (type solid)) (fill solid) (layer "F.SilkS"))')
        fp.append(text(m['label'],x+(2.5 if x<0 else -2.5),y,size=0.8))
        fp.append(f'(fp_circle (center {n(x)} {n(y)}) (end {n(x+1.5)} {n(y)}) (stroke (width 0.1) (type solid)) (fill none) (layer "F.Fab"))')
    fp.append(')')
    (pretty/f'{NAME}.kicad_mod').write_text('\n'.join(fp)+'\n',encoding='utf-8')
    # Mechanical schematic symbol: half-scale top view, mounting-hole circles
    # and terminal bar. STEP does not provide electrical pin assignments.
    sym=[f'(symbol "{NAME}" (in_bom no) (on_board yes)',
        '(property "Reference" "A" (at 0 12.7 0) (effects (font (size 1.27 1.27))))',
        '(property "Value" "AIM_XTRA_4_0" (at 0 10.16 0) (effects (font (size 1.27 1.27))))',
        f'(property "Footprint" "Witness_Modules:{NAME}" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))',
        '(property "Description" "Mechanical mounting template: four 3 mm mounting-hole locations and screw-terminal bar. No electrical pins, pads, or drills. Board envelope from STEP." (at 0 0 0) (effects (font (size 1.27 1.27)) hide))',
        f'(symbol "{NAME}_0_1"',
        '(rectangle (start -25 7.5) (end 25 -7.5) (stroke (width 0.254) (type default)) (fill (type background)))']
    a,b=terminal
    sym.append(f'(rectangle (start {n(a[0]/2)} {n(-a[1]/2)}) (end {n(b[0]/2)} {n(-b[1]/2)}) (stroke (width 0.2) (type default)) (fill (type outline)))')
    for m in mounts:
        x,y=m['x_mm']/2,-m['y_mm']/2
        sym.append(f'(circle (center {n(x)} {n(y)}) (radius 0.75) (stroke (width 0.2) (type default)) (fill (type none)))')
        sym.append(f'(text "{m["label"]}" (at {n(x+(2.6 if x<0 else -2.6))} {n(y)} 0) (effects (font (size 1 1))))')
    sym.append('))')
    original=LIB.read_text(encoding='utf-8')
    assert f'(symbol "{NAME}"' not in original
    combined=original.rstrip()[:-1]+'\n'+'\n'.join(sym)+'\n)\n'
    (OUT/'Witness_Modules.kicad_sym').write_text(combined,encoding='utf-8')
    metadata=dict(source=str(SOURCE),view='top from STEP +Y: X right and Z down',origin_xz_mm=origin,pcb_envelope_mm=bounds,outline_note='PCB bounding envelope; component intersection edges omitted',mounting_locations=mounts,screw_terminal_bar_mm=terminal,terminal_faces=[v[0] for v in terminal_faces],symbol_mounting_holes='graphical circles MH1-MH4; no electrical pins',pads=False,drills=False,previous_library_sha256=digest(LIB))
    (OUT/f'{NAME}_geometry.json').write_text(json.dumps(metadata,indent=2)+'\n',encoding='utf-8')
    import pcbnew as pcb
    loaded=pcb.FootprintLoad(str(pretty.resolve()),NAME)
    assert loaded is not None and loaded.GetPadCount()==0
    shapes=[g for g in loaded.GraphicalItems() if isinstance(g,pcb.PCB_SHAPE)]
    assert sum(g.GetShape()==pcb.SHAPE_T_CIRCLE for g in shapes)==8
    assert len(shapes)==13
    print('Validated: 100 x 30 mm envelope; 31 x 6.5 mm filled terminal bar; 4 mounting dots and 4 fabrication hole circles; zero pads.')
    print(json.dumps(dict(mounts=mounts,terminal=terminal),indent=2))
    sys.exit()
if '--inspect' in sys.argv:
    print('COUNTS',Counter(v[0] for v in entities.values()))
    print('UNITS',raw[-3000:])
    for i,(kind,body) in entities.items():
        if kind=='CYLINDRICAL_SURFACE':
            print('CYLINDER',i,float(body.split(',')[-1]),axis(refs(i)[0]))
        if kind=='ADVANCED_FACE':
            r=refs(i)
            if entities[r[-1]][0]=='PLANE':
                pts=[point(v) for b in r[:-1] for e in edges_of_bound(b) for v in refs(e)[:2]]
                bounds=[(min(p[d] for p in pts),max(p[d] for p in pts)) for d in range(3)]
                print('FACE',i,'bounds',len(r)-1,'plane',axis(refs(r[-1])[0]),'extent',bounds)
    sys.exit()
