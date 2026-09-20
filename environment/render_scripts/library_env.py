src=open('scene_render.py').read();exec(src[:src.index("BOOKS=[")])
exec(open('brighten.py').read())
SHELF_TOP=0.993
ENT=[("books/hunger_games_book.glb",0.37,-0.32,0.205,math.pi),("books/it_book.glb",0.3475,-0.238,0.235,math.pi),
     ("books/ballata_usignolo_book.glb",0.345,-0.168,0.21,math.pi),("books/alba_mietitura_book.glb",0.345,-0.107,0.21,math.pi),
     ("desk_decorations/pen_holder.glb",0.40,-0.04,0.095,0),("desk_decorations/desk_globe.glb",0.31,0.05,0.122,0)]
def scene():
    acts=load("bookshelf.glb",lambda t:(t.Translate(0,0,0.675),t.RotateZ(90),t.Translate(0.40,0,0)),yup=False)
    for f,x,y,h,yw in ENT:
        a=load(f,lambda t,x=x,y=y,h=h,yw=yw:(t.RotateZ(math.degrees(yw)),t.Translate(x,y,SHELF_TOP+h/2)))
        brighten(a,"pen_holder" in f)
        acts+=a
    return acts
C=(0.40,0,0.675)
def cam(d,el,az): return (C[0]-d*math.cos(el)*math.cos(az),C[1]-d*math.cos(el)*math.sin(az),C[2]+d*math.sin(el))
shoot(scene(),"env_library_front.png",cam(3.2,math.radians(3),0),C,1400,2000,30)
shoot(scene(),"env_library_rot_right.png",cam(3.2,math.radians(12),math.radians(35)),C,1600,2000,30)
