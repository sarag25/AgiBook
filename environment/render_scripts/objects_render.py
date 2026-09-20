src=open('scene_render.py').read();exec(src[:src.index("BOOKS=[")])   # load()/shoot()
BOOKS=[("hunger_games_book",(0.2,0.07,0.205)),("it_book",(0.155,0.055,0.235)),
       ("ballata_usignolo_book",(0.15,0.045,0.21)),("alba_mietitura_book",(0.15,0.038,0.21))]
OBJ={"globe":("desk_decorations/desk_globe.glb",0.122),"pen":("desk_decorations/pen_holder.glb",0.095)}
exec(open('brighten.py').read())
def obj(k,x,y,yaw=0):
    f,h=OBJ[k];return brighten(load(f,lambda t:(t.RotateZ(yaw),t.Translate(x,y,h/2))),k=="pen")
def stack(x0=0,y0=0):
    a=[];z=0.0
    for (k,s),yw in zip(BOOKS,[0,6,-5,4]):
        th=s[1];a+=load(f"books/{k}.glb",lambda t,z=z,th=th,yw=yw:(t.RotateX(90),t.RotateZ(yw),t.Translate(x0,y0,z+th/2)));z+=th+0.0005
    return a,z
cam=lambda f,d,el,az: (f[0]+d*math.cos(el)*math.cos(az),f[1]+d*math.cos(el)*math.sin(az),f[2]+d*math.sin(el))
# singoli e coppia
shoot(obj("globe",0,0),"globe.png",cam((0,0,0.06),0.55,math.radians(15),math.radians(20)),(0,0,0.06),1200,1400,28)
shoot(obj("pen",0,0),"pen_holder.png",cam((0,0,0.05),0.5,math.radians(18),math.radians(20)),(0,0,0.047),1200,1400,28)
shoot(obj("globe",0,-0.055)+obj("pen",0,0.035),"objects_pair.png",cam((0,0,0.06),0.55,math.radians(15),math.radians(20)),(0,0,0.06),1600,1200,28)
# pila + oggetti: mappamondo a sinistra, portapenne a destra (visti dal lato dei dorsi, +x)
a,z=stack()
a+=obj("globe",0.03,-0.19)+obj("pen",0.03,0.15)
shoot(a,"books_stack_with_objects.png",cam((0,0,z/2),1.05,math.radians(18),math.radians(25)),(0.02,0,z/2),2000,1400,28)
