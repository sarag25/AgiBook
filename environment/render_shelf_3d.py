import vtk, math, sys
vtk.vtkObject.GlobalWarningDisplayOff()
def render(out, azim_deg, elev_deg, dist=3.2, W=1600, H=2000):
    imp=vtk.vtkGLTFImporter();imp.SetFileName("/home/robot/SmartRobotics/src/agibot_x2_pkg/meshes/full_scene.glb")
    rw=vtk.vtkRenderWindow();rw.SetOffScreenRendering(1);rw.SetSize(W,H);rw.SetAlphaBitPlanes(1)
    imp.SetRenderWindow(rw);imp.Update()
    ren=rw.GetRenderers().GetFirstRenderer()
    acts=ren.GetActors();acts.InitTraversal();B=[1e9,-1e9,1e9,-1e9,1e9,-1e9]
    for i in range(acts.GetNumberOfItems()):
        a=acts.GetNextActor();b=a.GetBounds()
        if b[1]<-0.44: a.VisibilityOff();continue      # tavolo
        for k in range(3): B[2*k]=min(B[2*k],b[2*k]);B[2*k+1]=max(B[2*k+1],b[2*k+1])
    c=[(B[0]+B[1])/2,(B[2]+B[3])/2,(B[4]+B[5])/2]
    az=math.radians(azim_deg);el=math.radians(elev_deg)
    cam=ren.GetActiveCamera();cam.SetFocalPoint(*c)
    cam.SetPosition(c[0]+dist*math.sin(az)*math.cos(el),c[1]+dist*math.cos(az)*math.cos(el),c[2]+dist*math.sin(el))
    cam.SetViewUp(0,0,1);cam.SetViewAngle(30)
    ren.SetBackground(1,1,1);ren.SetBackgroundAlpha(0.0);ren.GradientBackgroundOff()
    ren.RemoveAllLights()
    for pos,it in [((2,3,3),1.6),((-2,3,2),1.1),((0,2,0.3),0.7),((0,-3,2),0.4)]:
        l=vtk.vtkLight();l.SetPosition(*pos);l.SetFocalPoint(*c);l.SetIntensity(it);ren.AddLight(l)
    hl=vtk.vtkLight();hl.SetLightTypeToHeadlight();hl.SetIntensity(0.9);ren.AddLight(hl)
    ren.ResetCameraClippingRange()
    try: pass
    except: pass
    rw.Render()
    rw.Render();w=vtk.vtkWindowToImageFilter();w.SetInput(rw);w.ReadFrontBufferOff();w.SetInputBufferTypeToRGBA();w.Update()
    p=vtk.vtkPNGWriter();p.SetFileName(out);p.SetInputConnection(w.GetOutputPort());p.Write()
    print("B",[round(x,3) for x in B])
render("shelf_3d_right.png",35,12)
render("shelf_3d_left.png",-35,12)
