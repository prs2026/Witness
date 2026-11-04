from pathlib import Path
import re
from collections import Counter
SOURCE=Path(__file__).parent/'GPS Tracker v15 parts - FWGPS.step'
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
if __name__=='__main__':
    print('COUNTS',Counter(v[0] for v in entities.values()))
    for b in refs(222)[:-1]:
        print('BOUND',b)
        for e in edges_of_bound(b):
            r=refs(e)
            print('EDGE',e,point(r[0]),point(r[1]),entities[r[2]])
    print('UNITS',raw[-2300:])
    for i,(kind,body) in entities.items():
        if kind=='CYLINDRICAL_SURFACE':
            print('CYLINDER',i,float(body.split(',')[-1]),axis(refs(i)[0]))
        if kind=='ADVANCED_FACE':
            r=refs(i)
            if entities[r[-1]][0]=='PLANE':
                pts=[p for b in r[:-1] for p in bound_points(b)]
                ext=[(min(p[d] for p in pts),max(p[d] for p in pts)) for d in range(3)]
                area=sorted((hi-lo for lo,hi in ext),reverse=True)
                if area[0]*area[1]>0.0001:
                    print('FACE',i,'bounds',len(r)-1,'plane',axis(refs(r[-1])[0]),'extent',ext)
