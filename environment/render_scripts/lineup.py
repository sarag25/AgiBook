"""Vista frontale ortografica, stessa scala per tutti: robot, libreria (scena grasp_test), tavolo, 4 libri, 2 oggetti."""
import vtk, math
from PIL import Image, ImageDraw, ImageFont
src=open('scene_render.py').read();exec(src[:src.index("BOOKS=[")])
exec(open('brighten.py').read())
from robot_vtk import load_robot
vtk.vtkObject.GlobalWarningDisplayOff()

def bounds(acts):
    B=[1e9,-1e9,1e9,-1e9,1e9,-1e9]
    for a in acts:
        b=a.GetBounds()
        for k in range(3):B[2*k]=min(B[2*k],b[2*k]);B[2*k+1]=max(B[2*k+1],b[2*k+1])
    return B
def ortho(acts,view,s,pad=6):
    """view: direzione DA cui guarda la camera ('+x','-x','+y','-y'). s = px/m. Ritorna RGBA ritagliata al bbox esatto."""
    B=bounds(acts);c=[(B[0]+B[1])/2,(B[2]+B[3])/2,(B[4]+B[5])/2]
    horiz=(B[3]-B[2]) if view in('+x','-x') else (B[1]-B[0]);Hm=B[5]-B[4]
    W=int(math.ceil(horiz*s))+2*pad;H=int(math.ceil(Hm*s))+2*pad
    ren=vtk.vtkRenderer();[ren.AddActor(a) for a in acts]
    rw=vtk.vtkRenderWindow();rw.SetOffScreenRendering(1);rw.SetSize(W,H);rw.SetAlphaBitPlanes(1);rw.AddRenderer(ren)
    ren.SetBackground(1,1,1);ren.SetBackgroundAlpha(0)
    d={'+x':(1,0,0),'-x':(-1,0,0),'+y':(0,1,0),'-y':(0,-1,0)}[view]
    cam=ren.GetActiveCamera();cam.ParallelProjectionOn();cam.SetFocalPoint(*c)
    cam.SetPosition(c[0]+5*d[0],c[1]+5*d[1],c[2]);cam.SetViewUp(0,0,1);cam.SetParallelScale(H/2/s)
    ren.RemoveAllLights()
    for p,it in [((c[0]+4*d[0]+2*d[1],c[1]+4*d[1]+2*d[0],c[2]+3),1.3),((c[0]+4*d[0]-2*d[1],c[1]+4*d[1]-2*d[0],c[2]+1),0.9)]:
        l=vtk.vtkLight();l.SetPosition(*p);l.SetFocalPoint(*c);l.SetIntensity(it);ren.AddLight(l)
    hl=vtk.vtkLight();hl.SetLightTypeToHeadlight();hl.SetIntensity(0.7);ren.AddLight(hl)
    ren.ResetCameraClippingRange();rw.Render();rw.Render()
    w=vtk.vtkWindowToImageFilter();w.SetInput(rw);w.ReadFrontBufferOff();w.SetInputBufferTypeToRGBA();w.Update()
    from vtk.util.numpy_support import vtk_to_numpy
    import numpy as np
    arr=vtk_to_numpy(w.GetOutput().GetPointData().GetScalars()).reshape(H,W,4)[::-1]
    return Image.fromarray(arr.copy()).crop((pad,pad,W-pad,H-pad))

# ---- oggetti
SHELF_TOP=0.993
ENT=[("books/hunger_games_book.glb",0.37,-0.32,0.205,math.pi),("books/it_book.glb",0.3475,-0.238,0.235,math.pi),
     ("books/ballata_usignolo_book.glb",0.345,-0.168,0.21,math.pi),("books/alba_mietitura_book.glb",0.345,-0.107,0.21,math.pi),
     ("desk_decorations/pen_holder.glb",0.40,-0.04,0.095,0),("desk_decorations/desk_globe.glb",0.31,0.05,0.122,0)]
def shelf_scene():
    acts=load("bookshelf.glb",lambda t:(t.Translate(0,0,0.675),t.RotateZ(90),t.Translate(0.40,0,0)),yup=False)
    for f,x,y,h,yw in ENT:
        a=load(f,lambda t,x=x,y=y,h=h,yw=yw:(t.RotateZ(math.degrees(yw)),t.Translate(x,y,SHELF_TOP+h/2)));brighten(a,"pen_holder" in f);acts+=a
    return acts
def table():
    return load("table.glb",lambda t:(t.Translate(0,0,0.25),t.Scale(1,1,1.5)),yup=False)   # glb vecchio a 0.50 m, come nell'URDF: scale z 1.5
def book(f,h): return load(f,lambda t:t.Translate(0,0,h/2))       # frame catalogo: x larghezza, y spessore, z altezza
def deco(f,h,pen=False): return brighten(load(f,lambda t:t.Translate(0,0,h/2)),pen)

S=int(__import__('sys').argv[1]) if len(__import__('sys').argv)>1 else 450
items=[("Robot AgiBot X2","robot",load_robot(),'+x'),
       ("Libreria","shelf",shelf_scene(),'-x'),
       ("Tavolo","table",table(),'-y')]
imgs=[(n,k,ortho(a,v,S)) for n,k,a,v in items]
small=[("Hunger Games",book("books/hunger_games_book.glb",0.205),'+y'),("IT",book("books/it_book.glb",0.235),'+y'),
       ("Ballata",book("books/ballata_usignolo_book.glb",0.21),'+y'),("Alba",book("books/alba_mietitura_book.glb",0.21),'+y'),
       ("Portapenne",deco("desk_decorations/pen_holder.glb",0.095,True),'-y'),("Mappamondo",deco("desk_decorations/desk_globe.glb",0.122),'-y')]
S2=S*3
simgs=[(n,ortho(a,v,S2)) for n,a,v in small]
import pickle
pickle.dump({"S":S,"S2":S2,"big":[(n,k,i.tobytes(),i.size) for n,k,i in imgs],"small":[(n,i.tobytes(),i.size) for n,i in simgs]},open("lineup.pkl","wb"))
print("ok",[(n,i.size) for n,k,i in imgs],[(n,i.size) for n,i in simgs])
