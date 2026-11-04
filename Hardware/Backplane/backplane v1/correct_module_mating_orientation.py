"""Reflect the current Witness/Raven libraries, preserving user's footprint edits."""
from pathlib import Path
import collections
import hashlib
import json
import re
import shutil
import sys
import pcbnew as pcb

DEST=Path(r'C:\Users\Ejfso\Documents\GitHub\witness\Hardware\KicadLibs')
OUT=Path(__file__).parent/'corrected_module_orientation'
NAMES=('Witness_v0_2_MicroLock','BlueRaven_2025_Location_Template')
def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()
if '--install' in sys.argv:
    manifest=json.loads((OUT/'orientation_correction.json').read_text())
    for name in NAMES:
        target=DEST/f'Witness_Modules.pretty/{name}.kicad_mod'
        backup=target.with_suffix('.before-mating-mirror.kicad_mod.bak')
        assert digest(target)==manifest['source_hashes'][name],f'{name} changed since staging'
        assert not backup.exists(),f'Backup already exists: {backup}'
    for name in NAMES:
        target=DEST/f'Witness_Modules.pretty/{name}.kicad_mod'
        shutil.copy2(target,target.with_suffix('.before-mating-mirror.kicad_mod.bak'))
        shutil.copy2(OUT/f'Witness_Modules.pretty/{name}.kicad_mod',target)
    print('Installed mirrored mating-face Witness and Blue Raven footprints; originals backed up.')
    sys.exit()

OUT.mkdir(exist_ok=True)
pretty=OUT/'Witness_Modules.pretty'
pretty.mkdir(exist_ok=True)
number=r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'
coordinate=re.compile(r'\((at|start|end|mid|center|xy|offset)(\s+)('+number+r')(\s+)('+number+r')(?:(\s+)('+number+r'))?')
def n(value):
    return f'{value:.9f}'.rstrip('0').rstrip('.') if abs(value)>1e-10 else '0'
def reflect(match):
    key,ws,x,ws2,y,ws3,angle=match.groups()
    result=f'({key}{ws}{n(-float(x))}{ws2}{y}'
    if angle is not None:
        result+=ws3+(n(-float(angle)) if key=='at' else angle)
    return result
hashes={}
def pad_signature(p):
    return (p.GetNumber(),p.GetAttribute(),p.GetShape(),pcb.ToMM(p.GetSize()),pcb.ToMM(p.GetDrillSize()))
for name in NAMES:
    source=DEST/f'Witness_Modules.pretty/{name}.kicad_mod'
    hashes[name]=digest(source)
    original=source.read_text(encoding='utf-8')
    before=pcb.FootprintLoad(str(source.parent),name)
    assert before is not None and len(before.Models())==0
    # Reflect X in local coordinates. Angles are negated; text stays readable
    # and all F/B layer assignments stay unchanged. Pin IDs are untouched.
    mirrored=coordinate.sub(reflect,original)
    # A horizontal reflection swaps text anchoring without mirroring glyphs.
    mirrored=re.sub(r'\(justify\s+(left|right)\b',lambda m:'(justify '+('right' if m[1]=='left' else 'left'),mirrored)
    if name==NAMES[0]:
        mirrored=mirrored.replace('Witness v0.2 module, top-view coordinates;','Witness v0.2 backplane mating-face module, X mirrored from source top view;')
    else:
        mirrored=mirrored.replace('Numbered top to bottom in unmirrored top view.','Backplane mating-face view, X mirrored from source top view; terminal IDs preserved.')
    (pretty/f'{name}.kicad_mod').write_text(mirrored,encoding='utf-8')
    after=pcb.FootprintLoad(str(pretty.resolve()),name)
    assert after is not None and after.GetLayer()==before.GetLayer()==pcb.F_Cu
    assert before.GetPadCount()==after.GetPadCount()
    assert collections.Counter(pad_signature(p) for p in before.Pads())==collections.Counter(pad_signature(p) for p in after.Pads())
    for a,b in zip(before.Pads(),after.Pads()):
        x,y=pcb.ToMM(a.GetPosition())
        xx,yy=pcb.ToMM(b.GetPosition())
        assert abs(xx+x)<1e-6 and abs(yy-y)<1e-6
    old_items=list(before.GraphicalItems())
    new_items=list(after.GraphicalItems())
    assert len(old_items)==len(new_items)
    # Every serialized coordinate must obey the same mirror. This also checks
    # user-added silkscreen, custom pad geometry, arc midpoints, and fields.
    for a,b in zip(coordinate.finditer(original),coordinate.finditer(mirrored)):
        assert a[1]==b[1]
        assert abs(float(a[3])+float(b[3]))<1e-8
        assert abs(float(a[5])-float(b[5]))<1e-8
    print(f'{name}: verified X reflection; {after.GetPadCount()} pads and {len(new_items)} graphics preserved, along with pin numbers, drill sizes and layers.')
manifest=dict(transform='local X -> -X, local Y unchanged; pad numbers preserved; front-layer mating-face footprint',source_hashes=hashes,footprints=NAMES)
(OUT/'orientation_correction.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
