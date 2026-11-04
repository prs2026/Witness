"""Edge-mounted miniMagSwitch, upper/lower right-angle header assembly."""
from pathlib import Path
import json,sys,hashlib,shutil,math
import pcbnew as pcb

ROOT=Path(__file__).parent
DEST=Path(r'C:\Users\Ejfso\Documents\GitHub\witness\Hardware\KicadLibs')
OUT=ROOT/'sideon_magswitch_output'
NAME='miniMagSwitch_SideOn_DualHeader'
LIB=DEST/'Witness_Modules.kicad_sym'
def digest(p): return hashlib.sha256(p.read_bytes()).hexdigest()
targets=[Path(f'Witness_Modules.pretty/{NAME}.kicad_mod'),Path(f'{NAME}_geometry.json'),Path('Witness_Modules.3dshapes/miniMagSwitch_SideOn_PCB.step'),Path('Witness_Modules.3dshapes/miniMagSwitch_SideOn_Headers.step')]
if '--install' in sys.argv:
    meta=json.loads((OUT/targets[1]).read_text())
    assert digest(LIB)==meta['previous_library_sha256'],'Shared symbol library changed'
    backup=DEST/'Witness_Modules.before-SideOn-MagSwitch.kicad_sym'
    for p in [backup]+[DEST/t for t in targets]: assert not p.exists(),f'Already exists: {p}'
    shutil.copy2(LIB,backup)
    (DEST/'Witness_Modules.3dshapes').mkdir(exist_ok=True)
    for t in targets: shutil.copy2(OUT/t,DEST/t)
    shutil.copy2(OUT/'Witness_Modules.kicad_sym',LIB)
    print('Installed side-on switch symbol, footprint, measured PCB model and dual-header assembly model.')
    sys.exit()
OUT.mkdir(exist_ok=True)
pretty=OUT/'Witness_Modules.pretty'; pretty.mkdir(exist_ok=True)
models=OUT/'Witness_Modules.3dshapes'; models.mkdir(exist_ok=True)
source=json.loads((DEST/'miniMagSwitch_pinmap.json').read_text())
backplane=pcb.LoadBoard(str(ROOT/'backplane v1.kicad_pcb'))
thickness=pcb.ToMM(backplane.GetDesignSettings().GetBoardThickness())
assert abs(thickness-1.6)<1e-6
separation=abs(source['pins'][0]['y_mm']-source['pins'][2]['y_mm'])
standoff=(separation-thickness)/2
assert abs(standoff-3.518)<1e-6
pad_y=2.54
pins=[]
for p in source['pins']:
    pins.append(dict(pin=p['pin'],signal=p['signal'],x_mm=p['x_mm'],y_mm=pad_y,assembly_side='front' if p['connector']=='H2' else 'back',lead_center_z_mm=standoff if p['connector']=='H2' else -thickness-standoff))
shutil.copy2(Path(r'C:\Users\Ejfso\Documents\GitHub\miniMagSwitch\miniMagSwitch.step'),models/'miniMagSwitch_SideOn_PCB.step')

# Standalone STEP with manifold BREP solids for the bent 0.64 mm square pins
# and two plastic bodies. Lead geometry is an assembly requirement, not a
# claim of fit for an unspecified off-the-shelf header.
class Step:
    def __init__(self): self.lines=[]; self.solids=[]
    def add(self,s):
        self.lines.append(s); return len(self.lines)
    def point(self,p): return self.add("CARTESIAN_POINT('',("+','.join(f'{x:.9f}' for x in p)+"))")
    def direction(self,p): return self.add("DIRECTION('',("+','.join(f'{x:.9f}' for x in p)+"))")
    def box(self,name,lo,hi,color):
        a,b,c=lo; x,y,z=hi
        points=[(a,b,c),(x,b,c),(x,y,c),(a,y,c),(a,b,z),(x,b,z),(x,y,z),(a,y,z)]
        cp=[self.point(p) for p in points]
        vertices=[self.add(f"VERTEX_POINT('',#{p})") for p in cp]
        edges={}; faces=[]
        for loop in [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]:
            oriented=[]
            for i,j in zip(loop,loop[1:]+loop[:1]):
                key=tuple(sorted((i,j)))
                if key not in edges:
                    u,v=key; delta=[points[v][d]-points[u][d] for d in range(3)]; length=math.sqrt(sum(t*t for t in delta))
                    direction=self.direction([t/length for t in delta]); vector=self.add(f"VECTOR('',#{direction},{length:.9f})")
                    line=self.add(f"LINE('',#{cp[u]},#{vector})")
                    edges[key]=self.add(f"EDGE_CURVE('',#{vertices[u]},#{vertices[v]},#{line},.T.)")
                oriented.append(self.add(f"ORIENTED_EDGE('',*,*,#{edges[key]},{'.T.' if i<j else '.F.'})"))
            edge_loop=self.add("EDGE_LOOP('',("+','.join(f'#{e}' for e in oriented)+'))')
            bound=self.add(f"FACE_OUTER_BOUND('',#{edge_loop},.T.)")
            u,v,w=[points[i] for i in loop[:3]]
            tangent=[v[d]-u[d] for d in range(3)]; other=[w[d]-v[d] for d in range(3)]
            normal=[tangent[1]*other[2]-tangent[2]*other[1],tangent[2]*other[0]-tangent[0]*other[2],tangent[0]*other[1]-tangent[1]*other[0]]
            def unit(v):
                length=math.sqrt(sum(t*t for t in v)); return [t/length for t in v]
            axis=self.add(f"AXIS2_PLACEMENT_3D('',#{cp[loop[0]]},#{self.direction(unit(normal))},#{self.direction(unit(tangent))})")
            plane=self.add(f"PLANE('',#{axis})")
            faces.append(self.add(f"ADVANCED_FACE('',(#{bound}),#{plane},.T.)"))
        shell=self.add("CLOSED_SHELL('',("+','.join(f'#{f}' for f in faces)+'))')
        solid=self.add(f"MANIFOLD_SOLID_BREP('{name}',#{shell})"); self.solids.append(solid)
        rgb=self.add("COLOUR_RGB('',"+','.join(map(str,color))+')')
        fillcolor=self.add(f"FILL_AREA_STYLE_COLOUR('',#{rgb})"); fill=self.add(f"FILL_AREA_STYLE('',(#{fillcolor}))")
        surf=self.add(f"SURFACE_STYLE_FILL_AREA(#{fill})"); side=self.add(f"SURFACE_SIDE_STYLE('',(#{surf}))")
        usage=self.add(f"SURFACE_STYLE_USAGE(.BOTH.,#{side})"); style=self.add(f'PRESENTATION_STYLE_ASSIGNMENT((#{usage}))')
        self.add(f"STYLED_ITEM('',(#{style}),#{solid})")
    def save(self,path):
        app=self.add("APPLICATION_CONTEXT('automotive_design')")
        context=self.add(f"PRODUCT_CONTEXT('',#{app},'mechanical')")
        product=self.add(f"PRODUCT('Side-on headers','Dual right-angle headers','Required 3.518 mm lead standoff',(#{context}))")
        formation=self.add(f"PRODUCT_DEFINITION_FORMATION('','',#{product})")
        definition_context=self.add(f"PRODUCT_DEFINITION_CONTEXT('part definition',#{app},'design')")
        definition=self.add(f"PRODUCT_DEFINITION('design','',#{formation},#{definition_context})")
        shape=self.add(f"PRODUCT_DEFINITION_SHAPE('','',#{definition})")
        length=self.add('(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.))')
        angle=self.add('(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.))')
        solidangle=self.add('(NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT())')
        repcontext=self.add(f"(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNIT_ASSIGNED_CONTEXT((#{length},#{angle},#{solidangle})) REPRESENTATION_CONTEXT('',''))")
        rep=self.add("ADVANCED_BREP_SHAPE_REPRESENTATION('',("+','.join(f'#{s}' for s in self.solids)+f'),#{repcontext})')
        self.add(f'SHAPE_DEFINITION_REPRESENTATION(#{shape},#{rep})')
        content="ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('Side-on magnetic switch headers'),'2;1');\nFILE_NAME('SideOnHeaders.step','2026-11-03T00:00:00',(''),(''),'Codex','Codex','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n"
        content+='\n'.join(f'#{j}={line};' for j,line in enumerate(self.lines,1))+'\nENDSEC;\nEND-ISO-10303-21;\n'
        path.write_text(content,encoding='ascii')
step=Step()
for p in pins:
    x,z=p['x_mm'],p['lead_center_z_mm']; y=-pad_y
    zlo,zhi=(-thickness-0.9,z+0.32) if p['assembly_side']=='front' else (z-0.32,0.9)
    step.box(p['pin']+'_vertical',(x-0.32,y-0.32,zlo),(x+0.32,y+0.32,zhi),(0.8,0.67,0.22))
    step.box(p['pin']+'_horizontal',(x-0.32,y-0.32,z-0.32),(x+0.32,2.5,z+0.32),(0.8,0.67,0.22))
for side in ('front','back'):
    group=[p for p in pins if p['assembly_side']==side]; z=group[0]['lead_center_z_mm']
    step.box(side+'_body',(min(p['x_mm'] for p in group)-1.27,-2.2,z-1.27),(max(p['x_mm'] for p in group)+1.27,0,z+1.27),(0.08,0.08,0.08))
step.save(models/'miniMagSwitch_SideOn_Headers.step')
def n(v): return f'{v:.6f}'.rstrip('0').rstrip('.') or '0'
def rect(layer,a,b,width=0.12):
    return f'(fp_rect (start {n(a[0])} {n(a[1])}) (end {n(b[0])} {n(b[1])}) (stroke (width {width}) (type solid)) (fill none) (layer "{layer}"))'
def text(label,x,y,layer='F.SilkS'):
    mirror=' (justify mirror)' if layer.startswith('B.') else ''
    return f'(fp_text user "{label}" (at {n(x)} {n(y)}) (layer "{layer}") (effects (font (size 0.8 0.8) (thickness 0.12)){mirror}))'
fp=[f'(footprint "{NAME}" (version 20241229) (generator "pcbnew") (layer "F.Cu")',
    '(descr "miniMagSwitch vertical beside PCB edge: H2 input header on FRONT, H1 output header on BACK. 1.6 mm host board; required horizontal lead center 3.518 mm from each face. Long-tail/custom header geometry, verify hardware. Four 1 mm drills at 2.54 mm pair pitch.") (attr through_hole)',
    '(property "Reference" "A**" (at 0 6) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))',
    f'(property "Value" "{NAME}" (at 0 7.5) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))']
for layer in ('F.SilkS','B.SilkS','F.Fab','B.Fab'):
    fp.append(rect(layer,(-5.842,-1.800051),(5.842,-0.2)))
fp.append('(fp_line (start -8 0) (end 8 0) (stroke (width 0.15) (type dash)) (layer "Dwgs.User"))')
fp.append(text('PCB EDGE / GLUE',0,-3,'Dwgs.User'))
for side,layer in [('front','F.SilkS'),('back','B.SilkS')]:
    group=[p for p in pins if p['assembly_side']==side]
    fp.append(rect(layer,(min(p['x_mm'] for p in group)-1.27,0),(max(p['x_mm'] for p in group)+1.27,2.2)))
    fp.append(text('H2 INPUT - FRONT' if side=='front' else 'H1 OUTPUT - BACK',0,4.7,layer))
for layer in ('F.CrtYd','B.CrtYd'):
    fp.append(rect(layer,(-6.75,-2.3),(6.75,3.9),0.05))
for p in pins:
    shape='rect' if p['pin'].endswith('_1') else 'circle'
    fp.append(f'(pad "{p["pin"]}" thru_hole {shape} (at {n(p["x_mm"])} {n(p["y_mm"])}) (size 1.7 1.7) (drill 1) (layers "*.Cu" "*.Mask"))')
    fp.append(text(p['pin'],p['x_mm'],3.7,'F.Fab'))
for model,offset,rotate in [('miniMagSwitch_SideOn_PCB.step',(-5.842,1.8000508,4.407),(-90,0,0)),('miniMagSwitch_SideOn_Headers.step',(0,0,0),(0,0,0))]:
    path=(DEST/'Witness_Modules.3dshapes'/model).as_posix()
    fp.append(f'(model "{path}" (offset (xyz {offset[0]} {offset[1]} {offset[2]})) (scale (xyz 1 1 1)) (rotate (xyz {rotate[0]} {rotate[1]} {rotate[2]})))')
fp.append(')')
(pretty/f'{NAME}.kicad_mod').write_text('\n'.join(fp)+'\n',encoding='utf-8')
sym=[f'(symbol "{NAME}" (pin_names (offset 1.016)) (in_bom yes) (on_board yes)',
    '(property "Reference" "A" (at 0 12.7 0) (effects (font (size 1.27 1.27))))',
    '(property "Value" "miniMagSwitch_SideOn" (at 0 10.16 0) (effects (font (size 1.27 1.27))))',
    f'(property "Footprint" "Witness_Modules:{NAME}" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))',
    '(property "Description" "Side-on miniMagSwitch, glued to PCB edge. H2 input header front; H1 output header back. Requires long-tail right-angle headers with 3.518 mm lead center standoff for 1.6 mm host PCB." (at 0 0 0) (effects (font (size 1.27 1.27)) hide))',
    f'(symbol "{NAME}_0_1" (rectangle (start -15.24 7.62) (end 15.24 -7.62) (stroke (width 0.254) (type default)) (fill (type background))))',
    f'(symbol "{NAME}_1_1"']
for p in pins:
    x,angle=(-20.32,0) if p['assembly_side']=='front' else (20.32,180)
    y=2.54 if p['pin'].endswith('_1') else -2.54
    label={'H2_1':'VIN','H2_2':'GND_IN','H1_1':'VOUT_SWITCHED','H1_2':'GND_OUT'}[p['pin']]
    sym.append(f'(pin passive line (at {x} {y} {angle}) (length 5.08) (name "{label}" (effects (font (size 1.016 1.016)))) (number "{p["pin"]}" (effects (font (size 1.016 1.016)))))')
sym.append(')))')
# Close only the nested symbol and outer symbol (the library is appended below).
sym[-1]='))'
original=LIB.read_text(); assert f'(symbol "{NAME}"' not in original
(OUT/'Witness_Modules.kicad_sym').write_text(original.rstrip()[:-1]+'\n'+'\n'.join(sym)+'\n)\n',encoding='utf-8')
meta=dict(source=str(Path(r'C:\Users\Ejfso\Documents\GitHub\miniMagSwitch')),host_thickness_mm=thickness,connector_row_separation_mm=separation,required_header_lead_standoff_mm=standoff,pins=pins,placement='Origin on host PCB edge; host inside +Y, module outside -Y; source board vertical; H2 on front, H1 on back',hardware='Parametric long-tail bent-pin assembly; not a verified catalogue header',module_model_offset_mm=[-5.842,1.8000508,4.407],module_model_rotation_deg=[-90,0,0],previous_library_sha256=digest(LIB))
(OUT/targets[1]).write_text(json.dumps(meta,indent=2)+'\n',encoding='utf-8')
loaded=pcb.FootprintLoad(str(pretty.resolve()),NAME)
assert loaded and loaded.GetPadCount()==4 and len(loaded.Models())==2
for p in loaded.Pads():
    expected=next(v for v in pins if v['pin']==p.GetNumber())
    assert pcb.ToMM(p.GetPosition())==(expected['x_mm'],expected['y_mm'])
    assert pcb.ToMM(p.GetDrillSize())==(1.0,1.0)
preview_pretty=OUT/'preview.pretty'; preview_pretty.mkdir(exist_ok=True)
preview_text='\n'.join(fp).replace((DEST/'Witness_Modules.3dshapes').as_posix(),models.resolve().as_posix())
(preview_pretty/f'{NAME}.kicad_mod').write_text(preview_text,encoding='utf-8')
loaded=pcb.FootprintLoad(str(preview_pretty.resolve()),NAME)
preview=pcb.BOARD(); preview.Add(loaded); loaded.SetPosition(pcb.VECTOR2I(pcb.FromMM(100),pcb.FromMM(100)))
for a,b in [((90,100),(110,100)),((110,100),(110,115)),((110,115),(90,115)),((90,115),(90,100))]:
    edge=pcb.PCB_SHAPE(); edge.SetShape(pcb.SHAPE_T_SEGMENT); edge.SetStart(pcb.VECTOR2I(pcb.FromMM(a[0]),pcb.FromMM(a[1]))); edge.SetEnd(pcb.VECTOR2I(pcb.FromMM(b[0]),pcb.FromMM(b[1]))); edge.SetLayer(pcb.Edge_Cuts); edge.SetWidth(pcb.FromMM(0.05)); preview.Add(edge)
pcb.SaveBoard(str(OUT/'sideon_preview.kicad_pcb'),preview)
print('Validated four pads and two models. Required upper/lower header standoff:',standoff,'mm. Header STEP contains',len(step.solids),'closed BREP solids.')
