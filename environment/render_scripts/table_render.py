src=open('scene_render.py').read();exec(src[:src.index("BOOKS=[")])
# table.glb e' ancora la geometria vecchia (0.50 m): come in table.urdf, scala z 1.5 -> 0.75 m
tab=lambda: load("table.glb",lambda t:(t.Translate(0,0,0.25),t.Scale(1,1,1.5)),yup=False)
f=(0,0,0.40)
cam=lambda d,el,az:(f[0]+d*math.cos(el)*math.cos(az),f[1]+d*math.cos(el)*math.sin(az),f[2]+d*math.sin(el))
shoot(tab(),"table_3d.png",cam(3.4,math.radians(22),math.radians(-55)),f,2000,1500,30)
shoot(tab(),"table_front.png",cam(3.4,math.radians(4),math.radians(-90)),f,2000,1300,30)
