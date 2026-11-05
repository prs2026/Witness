"""Extract solder-contact geometry for part 1101008 from the supplied STEP."""
from pathlib import Path
import re,json,math,sys
from collections import Counter
import hashlib,shutil

SOURCE=Path(r'C:\Users\Ejfso\Documents\GitHub\witness\Hardware\KicadLibs\1101 - Nosecone - 1101008 - EMI bracket(1).step')
raw=SOURCE.read_text()
entities={int(i):(kind,body) for i,kind,body in re.findall(r'#(\d+)\s*=\s*(\w+)\s*\((.*?)\)\s*;',raw,re.S)}
def refs(i): return [int(v) for v in re.findall(r'#(\d+)',entities[i][1])]
def vec(i): return tuple(float(v) for v in re.search(r'\(([^()]*)\)\s*$',entities[i][1])[1].split(','))
def axis(i):
    r=refs(i); return vec(r[0]),vec(r[1]),vec(r[2]) if len(r)>2 else None
def point(vertex): return vec(refs(vertex)[0])
def edges_of_bound(bound): return [refs(e)[-1] for e in refs(refs(bound)[0])]
def bound_points(bound): return [point(v) for e in edges_of_bound(bound) for v in refs(e)[:2]]
if '--inspect' in sys.argv:
    print('COUNTS',Counter(v[0] for v in entities.values()))
    print('UNITS',raw[-2200:])
    for i,(kind,body) in entities.items():
        if kind=='ADVANCED_FACE':
            r=refs(i)
            if entities[r[-1]][0]=='PLANE':
                pts=[p for b in r[:-1] for p in bound_points(b)]
                ext=[(min(p[d] for p in pts),max(p[d] for p in pts)) for d in range(3)]
                print('FACE',i,'bounds',len(r)-1,'plane',axis(refs(r[-1])[0]),'extent',ext)
        elif kind=='CYLINDRICAL_SURFACE':
            print('CYLINDER',i,float(body.split(',')[-1]),axis(refs(i)[0]))
    sys.exit()

DEST=SOURCE.parent
OUT=Path(__file__).parent/'emi_bracket_output'
NAME='EMI_Bracket_1101008'
LIB=DEST/'Witness_Modules.kicad_sym'
targets=[Path(f'Witness_Modules.pretty/{NAME}.kicad_mod'),Path(f'{NAME}_geometry.json')]
def digest(p): return hashlib.sha256(p.read_bytes()).hexdigest()
if '--install' in sys.argv:
    meta=json.loads((OUT/targets[1]).read_text())
    assert digest(SOURCE)==meta['source_sha256'],'STEP model changed'
    assert digest(LIB)==meta['previous_library_sha256'],'Shared symbol library changed'
    backup=DEST/'Witness_Modules.before-EMI-1101008.kicad_sym'
    for p in [backup]+[DEST/t for t in targets]: assert not p.exists(),f'Already exists: {p}'
    shutil.copy2(LIB,backup)
    for t in targets: shutil.copy2(OUT/t,DEST/t)
    shutil.copy2(OUT/'Witness_Modules.kicad_sym',LIB)
    print('Installed EMI bracket footprint and GND symbol with linked STEP model. Original symbol library backed up.')
    sys.exit()

assert 'SI_UNIT($,.METRE.)' in raw
planes=[]
for i,(kind,body) in entities.items():
    if kind=='ADVANCED_FACE':
        r=refs(i)
        if entities[r[-1]][0]=='PLANE':
            pts=[p for b in r[:-1] for p in bound_points(b)]
            ext=[(min(p[d] for p in pts)*1000,max(p[d] for p in pts)*1000) for d in range(3)]
            planes.append(dict(entity=i,axis=axis(refs(r[-1])[0]),extents_mm=ext))
# Opening at source X=-1.5875 mm; both thin edge faces touch the PCB.
contacts=[f for f in planes if abs(f['extents_mm'][0][0]+1.5875)<1e-6 and abs(f['extents_mm'][0][1]+1.5875)<1e-6 and abs(f['axis'][1][0])>0.999]
assert len(contacts)==2
zmin=min(f['extents_mm'][2][0] for f in contacts)
zmax=max(f['extents_mm'][2][1] for f in contacts)
zcenter=(zmin+zmax)/2
length=zmax-zmin
assert abs(length-38.1)<1e-6
rows=[]
for f in contacts:
    y0,y1=f['extents_mm'][1]
    assert abs(y1-y0-.508)<1e-6
    rows.append(dict(source_face=f['entity'],x_mm=0,y_mm=-(y0+y1)/2,contact_length_mm=length,contact_width_mm=y1-y0))
rows.sort(key=lambda p:p['y_mm'])
pad_length=round(length+1,6)
pad_width=1.5
assert abs(rows[0]['y_mm']+13.716)<1e-6 and abs(rows[1]['y_mm']-13.716)<1e-6
OUT.mkdir(exist_ok=True)
pretty=OUT/'Witness_Modules.pretty'; pretty.mkdir(exist_ok=True)
def n(v): return f'{v:.9f}'.rstrip('0').rstrip('.') or '0'
def text(label,x,y,layer='F.Fab',size=1):
    return f'(fp_text user "{label}" (at {n(x)} {n(y)}) (layer "{layer}") (effects (font (size {size} {size}) (thickness 0.15))))'
fp=[f'(footprint "{NAME}" (version 20241229) (generator "pcbnew") (layer "F.Cu")',
    '(descr "1101008 EMI bracket; U-channel shield 38.1 x 27.94 x 8.89 mm. Two 38.1 x 0.508 mm contact edges at Y +/-13.716 mm. Shared GND pad 1: 39.1 x 1.5 mm lands, no solder paste apertures. STEP opening aligned to PCB surface.")',
    '(tags "EMI shield bracket solder ground 1101008") (attr smd)',
    '(property "Reference" "SH**" (at 0 -16.5) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))',
    f'(property "Value" "{NAME}" (at 0 16.5) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))',
    '(fp_rect (start -19.05 -13.97) (end 19.05 13.97) (stroke (width 0.1) (type solid)) (fill none) (layer "F.Fab"))',
    '(fp_rect (start -19.8 -14.716) (end 19.8 14.716) (stroke (width 0.05) (type solid)) (fill none) (layer "F.CrtYd"))']
for x in (-19.05,19.05):
    fp.append(f'(fp_line (start {x} -12.4) (end {x} 12.4) (stroke (width 0.12) (type solid)) (layer "F.SilkS"))')
for row in rows:
    y=row['y_mm']
    fp.append(f'(pad "1" smd rect (at 0 {n(y)}) (size {n(pad_length)} {n(pad_width)}) (layers "F.Cu" "F.Mask") (solder_mask_margin 0.05))')
    fp.append(f'(fp_rect (start -19.05 {n(y-.254)}) (end 19.05 {n(y+.254)}) (stroke (width 0.1) (type solid)) (fill none) (layer "F.Fab"))')
fp.append(text('1101008 / GND',0,0))
fp.append(text('HEIGHT 8.89 mm',0,2))
# KiCad model rotations use the opposite sign to the active Cartesian
# rotation. The physical transform is Xworld=Zsource-Zcenter,
# Yworld=Ysource, Zworld=-Xsource-1.5875 (all coordinates in millimeters).
offset=(-zcenter,0,-1.5875)
fp.append(f'(model "{SOURCE.as_posix()}" (offset (xyz {n(offset[0])} 0 -1.5875)) (scale (xyz 1 1 1)) (rotate (xyz 0 -90 0)))')
fp.append(')')
(OUT/targets[0]).write_text('\n'.join(fp)+'\n',encoding='utf-8')
sym=[f'(symbol "{NAME}" (pin_names (offset 0)) (in_bom yes) (on_board yes)',
    '(property "Reference" "SH" (at 0 7.62 0) (effects (font (size 1.27 1.27))))',
    f'(property "Value" "{NAME}" (at 0 5.08 0) (effects (font (size 1.27 1.27))))',
    f'(property "Footprint" "Witness_Modules:{NAME}" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))',
    '(property "Description" "Soldered EMI shield bracket 1101008; both contact rails share pad 1. Connect GND to ground copper." (at 0 0 0) (effects (font (size 1.27 1.27)) hide))',
    f'(symbol "{NAME}_0_1" (rectangle (start -6.35 2.54) (end 6.35 -5.08) (stroke (width 0.254) (type default)) (fill (type background))))',
    f'(symbol "{NAME}_1_1" (pin passive line (at 0 -10.16 90) (length 5.08) (name "GND" (effects (font (size 1.016 1.016)))) (number "1" (effects (font (size 1.016 1.016)))))))']
original=LIB.read_text(); assert f'(symbol "{NAME}"' not in original
(OUT/'Witness_Modules.kicad_sym').write_text(original.rstrip()[:-1]+'\n'+'\n'.join(sym)+'\n)\n',encoding='utf-8')
meta=dict(source=str(SOURCE),source_sha256=digest(SOURCE),contact_plane_source_x_mm=-1.5875,contact_faces=rows,pad_number='1',pad_net_intent='GND',pad_dimensions_mm=[pad_length,pad_width],pad_fillet_allowance_mm=dict(ends=.5,sides=(pad_width-.508)/2),paste_apertures=False,model_offset_mm=offset,model_kicad_rotation_deg=[0,-90,0],physical_model_transform='X=sourceZ-Zcenter; Y=sourceY; Z=-sourceX-1.5875 mm',dimensions_mm=[38.1,27.94,8.89],previous_library_sha256=digest(LIB))
(OUT/targets[1]).write_text(json.dumps(meta,indent=2)+'\n',encoding='utf-8')
import pcbnew as pcb
loaded=pcb.FootprintLoad(str(pretty.resolve()),NAME)
assert loaded and loaded.GetPadCount()==2 and len(loaded.Models())==1
for p in loaded.Pads():
    assert p.GetNumber()=='1' and p.GetAttribute()==pcb.PAD_ATTRIB_SMD
    assert pcb.ToMM(p.GetSize())==(39.1,1.5)
    assert not p.IsOnLayer(pcb.F_Paste) and p.IsOnLayer(pcb.F_Cu)
    assert abs(abs(pcb.ToMM(p.GetPosition())[1])-13.716)<1e-6
assert abs((-1.5875)*-1+offset[2])<1e-6
preview=pcb.BOARD(); preview.Add(loaded); loaded.SetPosition(pcb.VECTOR2I(pcb.FromMM(100),pcb.FromMM(100)))
for a,b in [((75,80),(125,80)),((125,80),(125,120)),((125,120),(75,120)),((75,120),(75,80))]:
    edge=pcb.PCB_SHAPE(); edge.SetShape(pcb.SHAPE_T_SEGMENT); edge.SetStart(pcb.VECTOR2I(pcb.FromMM(a[0]),pcb.FromMM(a[1]))); edge.SetEnd(pcb.VECTOR2I(pcb.FromMM(b[0]),pcb.FromMM(b[1]))); edge.SetLayer(pcb.Edge_Cuts); edge.SetWidth(pcb.FromMM(.05)); preview.Add(edge)
pcb.SaveBoard(str(OUT/'EMI_Bracket_1101008_preview.kicad_pcb'),preview)
print('Validated two 39.1 x 1.5 mm common-ground solder lands at Y +/-13.716 mm, and linked model at contact plane Z=0.')
