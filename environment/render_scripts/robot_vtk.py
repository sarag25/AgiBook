"""Carica x2_hand_gazebo.urdf in VTK (posa zero, giunti fissi/zero), restituisce gli attori."""
import vtk, re, math, xml.etree.ElementTree as ET
PKG="/home/robot/SmartRobotics/src/agibot_x2_pkg/"
def rpy_T(xyz,rpy):
    t=vtk.vtkTransform();t.PostMultiply()
    t.RotateX(math.degrees(rpy[0]));t.RotateY(math.degrees(rpy[1]));t.RotateZ(math.degrees(rpy[2]))
    t.Translate(*xyz);return t
def orig(el):
    o=el.find("origin") if el is not None else None
    f=lambda s:[float(v) for v in s.split()]
    return (f(o.get("xyz","0 0 0")),f(o.get("rpy","0 0 0"))) if o is not None else ([0,0,0],[0,0,0])
def load_robot(path=PKG+"urdf/x2_hand_gazebo.urdf"):
    txt=open(path,encoding="utf-8-sig").read()
    txt=re.sub(r"<!--.*?-->","",txt,flags=re.S);txt=re.sub(r"<xacro:[^>]*/>","",txt);txt=txt.replace("$(arg finger_mu)","1")
    root=ET.fromstring(txt)
    links={l.get("name"):l for l in root.findall("link")}
    child_joint={j.find("child").get("link"):j for j in root.findall("joint")}
    cache={}
    def world(name):
        if name in cache: return cache[name]
        j=child_joint.get(name)
        m=vtk.vtkMatrix4x4()
        if j is not None:
            xyz,rpy=orig(j);vtk.vtkMatrix4x4.Multiply4x4(world(j.find("parent").get("link")),rpy_T(xyz,rpy).GetMatrix(),m)
        cache[name]=m;return m
    acts=[]
    for n,l in links.items():
        for v in l.findall("visual"):
            g=v.find("geometry");xyz,rpy=orig(v)
            mesh=g.find("mesh");box=g.find("box")
            if mesh is not None:
                fn=mesh.get("filename").split("/meshes/")[-1];src=vtk.vtkSTLReader();src.SetFileName(PKG+"meshes/"+fn)
            elif box is not None:
                s=[float(x) for x in box.get("size").split()];src=vtk.vtkCubeSource();src.SetXLength(s[0]);src.SetYLength(s[1]);src.SetZLength(s[2])
            else: continue
            mp=vtk.vtkPolyDataMapper();mp.SetInputConnection(src.GetOutputPort());a=vtk.vtkActor();a.SetMapper(mp)
            M=vtk.vtkMatrix4x4();vtk.vtkMatrix4x4.Multiply4x4(world(n),rpy_T(xyz,rpy).GetMatrix(),M);a.SetUserMatrix(M)
            col=v.find("material/color")
            rgb=[float(c) for c in col.get("rgba").split()[:3]] if col is not None else None
            if rgb is not None and min(rgb)<0.9: a.GetProperty().SetColor(*rgb)   # colori veri (piedi, dita)
            elif "tcp" in n or "camera" in n: a.GetProperty().SetColor(1,0,0)
            else:
                dark=any(k in n for k in ("knee","ankle_pitch","hip_pitch","elbow","wrist","waist_yaw","head_pitch","shoulder_roll"))
                a.GetProperty().SetColor(*((0.16,0.16,0.18) if dark else (0.80,0.80,0.81)))
            if "tcp" in n: continue
            p=a.GetProperty();p.SetAmbient(0.12);p.SetDiffuse(0.75);p.SetSpecular(0.15);p.SetSpecularPower(20)
            acts.append(a)
    return acts
if __name__=="__main__":
    acts=load_robot();b=[1e9,-1e9]*3;B=[1e9,-1e9,1e9,-1e9,1e9,-1e9]
    for a in acts:
        bb=a.GetBounds()
        for k in range(3):B[2*k]=min(B[2*k],bb[2*k]);B[2*k+1]=max(B[2*k+1],bb[2*k+1])
    print(len(acts),"bounds",[round(x,3) for x in B],"ext",[round(B[2*k+1]-B[2*k],3) for k in range(3)])
    import xml.etree.ElementTree as ET2
    txt=re.sub(r"<!--.*?-->","",open(PKG+"urdf/x2_hand_gazebo.urdf",encoding="utf-8-sig").read(),flags=re.S)
    print("massa",round(sum(float(m) for m in re.findall(r'<mass value="([^"]+)"',txt)),2))
