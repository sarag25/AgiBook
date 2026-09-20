"""Render VTK con sfondo trasparente: (1) pila dei 4 libri della scena
grasp_test, (2) ripiano alto della libreria come in Gazebo (book_placer.GRASP_TEST_ENTITIES)."""
import vtk, math
from PIL import Image
vtk.vtkObject.GlobalWarningDisplayOff()
M="/home/robot/SmartRobotics/src/agibot_x2_pkg/meshes/"
KEEP=[]
def load(path, T_fn, yup=True):
    """Importa un glb, centra il bbox sull'origine (base a z=0 dopo Y-up->Z-up) e applica T_fn(vtkTransform)."""
    imp=vtk.vtkGLTFImporter();imp.SetFileName(M+path);rw=vtk.vtkRenderWindow();rw.SetOffScreenRendering(1)
    imp.SetRenderWindow(rw);imp.Update();KEEP.append((imp,rw))
    r=rw.GetRenderers().GetFirstRenderer();b=r.ComputeVisiblePropBounds()
    c=[(b[0]+b[1])/2,(b[2]+b[3])/2,(b[4]+b[5])/2]
    t=vtk.vtkTransform();t.PostMultiply()
    t.Translate(-c[0],-c[1],-c[2])
    if yup: t.RotateX(90)
    T_fn(t)
    out=[];acts=r.GetActors();acts.InitTraversal()
    for _ in range(acts.GetNumberOfItems()):
        a=acts.GetNextActor();m=vtk.vtkMatrix4x4();m.DeepCopy(a.GetMatrix())
        f=vtk.vtkMatrix4x4();vtk.vtkMatrix4x4.Multiply4x4(t.GetMatrix(),m,f)
        a.SetPosition(0,0,0);a.SetOrientation(0,0,0);a.SetScale(1,1,1);a.SetUserMatrix(f);out.append(a)
    return out
def shoot(actors,out,pos,focal,W,H,angle=30):
    ren=vtk.vtkRenderer();[ren.AddActor(a) for a in actors]
    rw=vtk.vtkRenderWindow();rw.SetOffScreenRendering(1);rw.SetSize(W,H);rw.SetAlphaBitPlanes(1);rw.AddRenderer(ren)
    ren.SetBackground(1,1,1);ren.SetBackgroundAlpha(0)
    cam=ren.GetActiveCamera();cam.SetPosition(*pos);cam.SetFocalPoint(*focal);cam.SetViewUp(0,0,1);cam.SetViewAngle(angle)
    ren.RemoveAllLights()
    for p,it in [((pos[0]+1,pos[1]+1.5,pos[2]+2),1.4),((pos[0]+1,pos[1]-1.5,pos[2]+1),1.0),((focal[0],focal[1],focal[2]+3),0.6)]:
        l=vtk.vtkLight();l.SetPosition(*p);l.SetFocalPoint(*focal);l.SetIntensity(it);ren.AddLight(l)
    hl=vtk.vtkLight();hl.SetLightTypeToHeadlight();hl.SetIntensity(0.8);ren.AddLight(hl)
    ren.ResetCameraClippingRange();rw.Render();rw.Render()
    w=vtk.vtkWindowToImageFilter();w.SetInput(rw);w.ReadFrontBufferOff();w.SetInputBufferTypeToRGBA();w.Update()
    p=vtk.vtkPNGWriter();p.SetFileName(out);p.SetInputConnection(w.GetOutputPort());p.Write()
    im=Image.open(out);bb=im.getchannel('A').getbbox();im.crop((max(bb[0]-20,0),max(bb[1]-20,0),min(bb[2]+20,im.width),min(bb[3]+20,im.height))).save(out)

BOOKS=[("hunger_games_book",(0.2,0.07,0.205)),("it_book",(0.155,0.055,0.235)),
       ("ballata_usignolo_book",(0.15,0.045,0.21)),("alba_mietitura_book",(0.15,0.038,0.21))]
# ---- 1) pila: libri coricati (spessore in verticale), dorsi verso +x
acts=[];z=0.0;yaws=[0,6,-5,4]
for (k,s),yw in zip(BOOKS,yaws):
    th=s[1]
    def T(t,z=z,th=th,yw=yw):
        t.RotateX(90)            # coricato: lo spessore (y) diventa verticale
        t.RotateZ(yw);t.Translate(0,0,z+th/2)
    acts+=load(f"books/{k}.glb",T);z+=th+0.0005
shoot(acts,"books_stack.png",(0.75,0.35,0.42),(0,0,z/2),1600,1400,28)

# ---- 2) ripiano alto come in Gazebo
SHELF_TOP=0.993
ENT=[("books/hunger_games_book.glb",0.37,-0.32,0.205,math.pi),("books/it_book.glb",0.3475,-0.238,0.235,math.pi),
     ("books/ballata_usignolo_book.glb",0.345,-0.168,0.21,math.pi),("books/alba_mietitura_book.glb",0.345,-0.107,0.21,math.pi),
     ("desk_decorations/pen_holder.glb",0.40,-0.04,0.095,0),("desk_decorations/desk_globe.glb",0.31,0.05,0.122,0)]
acts=load("bookshelf.glb",lambda t:(t.Translate(0,0,0.675),t.RotateZ(90),t.Translate(0.40,0,0)),yup=False)
cut=vtk.vtkPlane();cut.SetOrigin(0,0,SHELF_TOP-0.022);cut.SetNormal(0,0,1)   # solo lo scomparto alto
for a in acts: a.GetMapper().AddClippingPlane(cut)
for f,x,y,h,yw in ENT:
    acts+=load(f,lambda t,x=x,y=y,h=h,yw=yw:(t.RotateZ(math.degrees(yw)),t.Translate(x,y,SHELF_TOP+h/2)))
shoot(acts,"top_shelf_full.png",(0.40-1.65,-0.55,1.50),(0.40,0.0,1.16),2000,1300,30)
