"""Widen part 1101008 by translating each side 1.5 mm, retaining gauge/bends."""
from pathlib import Path
from datetime import datetime
import hashlib,json,re,shutil,sys
import pcbnew as pcb

ROOT=Path(__file__).parent
DEST=Path(r'C:\Users\Ejfso\Documents\GitHub\witness\Hardware\KicadLibs')
OUT=ROOT/'emi_bracket_widened_output'
NAME='EMI_Bracket_1101008'
MODEL='1101008_EMI_Bracket_Wide_30.94mm.step'
fp_rel=Path(f'Witness_Modules.pretty/{NAME}.kicad_mod')
meta_rel=Path(f'{NAME}_geometry.json')
source=DEST/'1101 - Nosecone - 1101008 - EMI bracket(1).step'
def digest(p): return hashlib.sha256(p.read_bytes()).hexdigest()
if '--install' in sys.argv:
    manifest=json.loads((OUT/'update_manifest.json').read_text())
    for rel,expected in manifest['original_hashes'].items():
        assert digest(DEST/rel)==expected,f'File changed since staging: {rel}'
    assert not (DEST/MODEL).exists(),'Widened model already exists'
    stamp=datetime.now().strftime('%Y%m%d-%H%M%S')
    for rel in (fp_rel,meta_rel):
        shutil.copy2(DEST/rel,Path(str(DEST/rel)+f'.before-widening-{stamp}.bak'))
    shutil.copy2(OUT/MODEL,DEST/MODEL)
    for rel in (fp_rel,meta_rel): shutil.copy2(OUT/rel,DEST/rel)
    print('Updated EMI_Bracket_1101008 to 30.94 mm overall width, including widened STEP; prior footprint and geometry backed up.')
    sys.exit()
OUT.mkdir(exist_ok=True); (OUT/'Witness_Modules.pretty').mkdir(exist_ok=True)
raw=source.read_text()
assert 'SI_UNIT($,.METRE.)' in raw
number=r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'
pattern=re.compile(r"(CARTESIAN_POINT\s*\(\s*'[^']*'\s*,\s*\(\s*)("+number+r')(\s*,\s*)('+number+r')(\s*,\s*)('+number+r')')
def f(v): return f'{v:.15f}'.rstrip('0').rstrip('.') if abs(v)>1e-14 else '0.'
def widen_point(m):
    y=float(m[4]); delta=.0015 if y>1e-9 else -.0015 if y< -1e-9 else 0
    return m[1]+m[2]+m[3]+f(y+delta)+m[5]+m[6]
wide=pattern.sub(widen_point,raw)
oldpts=[tuple(float(m[i]) for i in (2,4,6)) for m in pattern.finditer(raw)]
newpts=[tuple(float(m[i]) for i in (2,4,6)) for m in pattern.finditer(wide)]
assert len(oldpts)==len(newpts)==75
for a,b in zip(oldpts,newpts):
    assert abs(a[0]-b[0])<1e-13 and abs(a[2]-b[2])<1e-13
    assert abs(b[1]-a[1]-(.0015 if a[1]>1e-9 else -.0015 if a[1]< -1e-9 else 0))<1e-13
assert abs((max(p[1] for p in newpts)-min(p[1] for p in newpts))*1000-30.94)<1e-6
# All surface/curve radii remain exactly the original values. Only points move.
radial=re.compile(r'(?:CIRCLE|CYLINDRICAL_SURFACE)\s*\([^;]*;',re.S)
assert radial.findall(raw)==radial.findall(wide)
(OUT/MODEL).write_text(wide,encoding='utf-8')
fp=(DEST/fp_rel).read_text()
coord=re.compile(r'\((at|start|end|mid|center|xy)(\s+)('+number+r')(\s+)('+number+r')')
def widen_coord(m):
    y=float(m[5])
    if abs(y)<12: return m[0]
    return '('+m[1]+m[2]+m[3]+m[4]+f(y+(1.5 if y>0 else -1.5))
updated=coord.sub(widen_coord,fp)
updated=updated.replace('38.1 x 27.94 x 8.89 mm','38.1 x 30.94 x 8.89 mm').replace('Y +/-13.716 mm','Y +/-15.216 mm')
assert source.as_posix() in updated
updated=updated.replace(source.as_posix(),(DEST/MODEL).as_posix())
(OUT/fp_rel).write_text(updated,encoding='utf-8')
meta=json.loads((DEST/meta_rel).read_text())
meta['original_source']=str(source)
meta['source']=str(DEST/MODEL)
meta['source_sha256']=digest(OUT/MODEL)
meta['dimensions_mm']=[38.1,30.94,8.89]
meta['modification']='Overall width +3 mm; both sides translated outward 1.5 mm; thickness and bend radii retained'
for row in meta['contact_faces']: row['y_mm']+=1.5 if row['y_mm']>0 else -1.5
(OUT/meta_rel).write_text(json.dumps(meta,indent=2)+'\n',encoding='utf-8')
(OUT/'update_manifest.json').write_text(json.dumps({'original_hashes':{str(rel):digest(DEST/rel) for rel in (fp_rel,meta_rel,source.relative_to(DEST))}},indent=2)+'\n',encoding='utf-8')
before=pcb.FootprintLoad(str(DEST/'Witness_Modules.pretty'),NAME)
after=pcb.FootprintLoad(str((OUT/'Witness_Modules.pretty').resolve()),NAME)
assert after and before and before.GetPadCount()==after.GetPadCount()==2
for a,b in zip(before.Pads(),after.Pads()):
    assert a.GetNumber()==b.GetNumber()=='1'
    assert a.GetSize()==b.GetSize()
    x,y=pcb.ToMM(b.GetPosition())
    assert abs(x)<1e-6 and abs(abs(y)-15.216)<1e-6
preview_dir=OUT/'preview.pretty'; preview_dir.mkdir(exist_ok=True)
preview_text=updated.replace((DEST/MODEL).as_posix(),(OUT/MODEL).resolve().as_posix())
(preview_dir/f'{NAME}.kicad_mod').write_text(preview_text,encoding='utf-8')
preview=pcb.BOARD(); preview_fp=pcb.FootprintLoad(str(preview_dir.resolve()),NAME); preview.Add(preview_fp)
preview_fp.SetPosition(pcb.VECTOR2I(pcb.FromMM(100),pcb.FromMM(100)))
for a,b in [((75,78),(125,78)),((125,78),(125,122)),((125,122),(75,122)),((75,122),(75,78))]:
    edge=pcb.PCB_SHAPE(); edge.SetShape(pcb.SHAPE_T_SEGMENT); edge.SetStart(pcb.VECTOR2I(pcb.FromMM(a[0]),pcb.FromMM(a[1]))); edge.SetEnd(pcb.VECTOR2I(pcb.FromMM(b[0]),pcb.FromMM(b[1]))); edge.SetLayer(pcb.Edge_Cuts); edge.SetWidth(pcb.FromMM(.05)); preview.Add(edge)
pcb.SaveBoard(str(OUT/'EMI_Bracket_Widened_preview.kicad_pcb'),preview)
print('Validated overall width 30.94 mm (+3 mm), pads at Y +/-15.216 mm, unchanged 0.508 mm metal gauge and bend radii.')
