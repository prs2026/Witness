"""Stage and install the AIM XTRA template with measured NPTH mounting holes."""
from pathlib import Path
import hashlib
import json
import re
import shutil
import sys
import pcbnew as pcb

DEST=Path(r'C:\Users\Ejfso\Documents\GitHub\witness\Hardware\KicadLibs')
OUT=Path(__file__).parent/'aim_xtra_drilled_output'
NAME='AIM_XTRA_4_0_Location_Template'
targets=[Path(f'Witness_Modules.pretty/{NAME}.kicad_mod'),Path('Witness_Modules.kicad_sym'),Path(f'{NAME}_geometry.json')]
def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()
if '--install' in sys.argv:
    hashes=json.loads((OUT/'original_hashes.json').read_text())
    for rel in targets:
        assert digest(DEST/rel)==hashes[str(rel)],f'File changed: {rel}'
        assert not (DEST/(str(rel)+'.before-mounting-drills.bak')).exists()
    for rel in targets:
        shutil.copy2(DEST/rel,DEST/(str(rel)+'.before-mounting-drills.bak'))
        shutil.copy2(OUT/rel,DEST/rel)
    print('Installed four 3 mm unplated mounting holes; original files backed up.')
    sys.exit()
OUT.mkdir(exist_ok=True)
(OUT/'Witness_Modules.pretty').mkdir(exist_ok=True)
geometry=json.loads((DEST/targets[2]).read_text())
fp=(DEST/targets[0]).read_text()
fp=fp.replace('four mounting-center dots. No pads or drills.','four 3 mm unplated mounting holes. No electrical pads.')
for m in geometry['mounting_locations']:
    x,y=m['x_mm'],m['y_mm']
    pattern=rf'^\(fp_circle \(center {x:g} {y:g}\).*\(fill solid\) \(layer "F.SilkS"\)\)\s*$'
    replacement=f'(pad "" np_thru_hole circle (at {x:g} {y:g}) (size 3 3) (drill 3) (layers "*.Cu" "*.Mask"))'
    fp,count=re.subn(pattern,replacement,fp,flags=re.M)
    assert count==1,(m,count)
(OUT/targets[0]).write_text(fp,encoding='utf-8')
sym=(DEST/targets[1]).read_text()
old='Mechanical mounting template: four 3 mm mounting-hole locations and screw-terminal bar. No electrical pins, pads, or drills. Board envelope from STEP.'
new='Mechanical mounting template: four 3 mm unplated mounting holes and screw-terminal bar. No electrical pins or copper pads. Board envelope from STEP.'
assert sym.count(old)==1
(OUT/targets[1]).write_text(sym.replace(old,new),encoding='utf-8')
geometry.update(pads='four non-electrical NPTH pads only',drills=True,mounting_drill_mm=3.0,mounting_type='unplated')
(OUT/targets[2]).write_text(json.dumps(geometry,indent=2)+'\n',encoding='utf-8')
(OUT/'original_hashes.json').write_text(json.dumps({str(p):digest(DEST/p) for p in targets},indent=2),encoding='utf-8')
loaded=pcb.FootprintLoad(str((OUT/'Witness_Modules.pretty').resolve()),NAME)
assert loaded.GetPadCount()==4
expected={(m['x_mm'],m['y_mm']) for m in geometry['mounting_locations']}
assert {pcb.ToMM(p.GetPosition()) for p in loaded.Pads()}==expected
for p in loaded.Pads():
    assert p.GetAttribute()==pcb.PAD_ATTRIB_NPTH
    assert pcb.ToMM(p.GetDrillSize())==(3.0,3.0)
    assert p.GetNumber()==''
print('Validated four 3 mm NPTH drills at (+/-47, +/-12) mm.')
