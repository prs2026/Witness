from pathlib import Path
import pcbnew as pcb
dest=Path(r'C:\Users\Ejfso\Documents\GitHub\witness\Hardware\KicadLibs\Witness_Modules.pretty')
for name in ('Witness_v0_2_MicroLock','BlueRaven_2025_Location_Template'):
    f=pcb.FootprintLoad(str(dest),name)
    print('LIBRARY',name,'pads',f.GetPadCount(),'graphics',len(list(f.GraphicalItems())))
    for p in f.Pads():
        print('PAD',p.GetNumber(),pcb.ToMM(p.GetPosition()),p.GetOrientationDegrees())
    print('MODELS',len(f.Models()))
b=pcb.LoadBoard(str(Path(__file__).parent/'backplane v1.kicad_pcb'))
for f in b.GetFootprints():
    if any(s in str(f.GetFPID().GetLibItemName()).lower() for s in ('witness','raven')):
        print('PLACED',f.GetReference(),str(f.GetFPID().GetLibItemName()),'side',f.GetLayerName(),'xy',pcb.ToMM(f.GetPosition()),'rotation',f.GetOrientationDegrees())
