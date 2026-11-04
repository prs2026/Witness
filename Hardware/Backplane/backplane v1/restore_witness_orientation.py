"""Undo only Witness's X reflection, retaining subsequent footprint edits."""
from pathlib import Path
import hashlib
import json
import re
import shutil
import sys
import pcbnew as pcb

NAME='Witness_v0_2_MicroLock'
DEST=Path(r'C:\Users\Ejfso\Documents\GitHub\witness\Hardware\KicadLibs\Witness_Modules.pretty')
OUT=Path(__file__).parent/'restored_witness_orientation'
target=DEST/f'{NAME}.kicad_mod'
def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()
if '--install' in sys.argv:
    metadata=json.loads((OUT/'restore.json').read_text())
    assert digest(target)==metadata['source_sha256'],'Witness changed since staging'
    backup=DEST/f'{NAME}.before-restore-original.kicad_mod.bak'
    assert not backup.exists()
    shutil.copy2(target,backup)
    shutil.copy2(OUT/f'{NAME}.kicad_mod',target)
    print('Restored Witness original orientation, retaining latest geometry edits. Raven untouched.')
    sys.exit()
OUT.mkdir(exist_ok=True)
original=target.read_text()
number=r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'
pattern=re.compile(r'\((at|start|end|mid|center|xy|offset)(\s+)('+number+r')(\s+)('+number+r')(?:(\s+)('+number+r'))?')
def n(v):
    return f'{v:.9f}'.rstrip('0').rstrip('.') if abs(v)>1e-10 else '0'
def reflect(m):
    key,ws,x,ws2,y,ws3,angle=m.groups()
    result=f'({key}{ws}{n(-float(x))}{ws2}{y}'
    if angle is not None:
        result+=ws3+(n(-float(angle)) if key=='at' else angle)
    return result
restored=pattern.sub(reflect,original)
restored=re.sub(r'\(justify\s+(left|right)\b',lambda m:'(justify '+('right' if m[1]=='left' else 'left'),restored)
restored=restored.replace('Witness v0.2 backplane mating-face module, X mirrored from source top view;','Witness v0.2 module, top-view coordinates;')
(OUT/f'{NAME}.kicad_mod').write_text(restored,encoding='utf-8')
before=pcb.FootprintLoad(str(DEST),NAME)
after=pcb.FootprintLoad(str(OUT.resolve()),NAME)
assert before and after
assert before.GetPadCount()==after.GetPadCount()
assert len(list(before.GraphicalItems()))==len(list(after.GraphicalItems()))
assert before.GetLayer()==after.GetLayer()
for a,b in zip(before.Pads(),after.Pads()):
    assert a.GetNumber()==b.GetNumber()
    assert a.GetSize()==b.GetSize() and a.GetDrillSize()==b.GetDrillSize()
    x,y=pcb.ToMM(a.GetPosition())
    xx,yy=pcb.ToMM(b.GetPosition())
    assert abs(x+xx)<1e-6 and abs(y-yy)<1e-6
for a,b in zip(pattern.finditer(original),pattern.finditer(restored)):
    assert abs(float(a[3])+float(b[3]))<1e-8
    assert abs(float(a[5])-float(b[5]))<1e-8
(OUT/'restore.json').write_text(json.dumps({'source_sha256':digest(target),'operation':'Witness only: undo X reflection; preserve current geometry'}),encoding='utf-8')
print('Validated Witness-only orientation restoration, preserving current pads and graphics.')
