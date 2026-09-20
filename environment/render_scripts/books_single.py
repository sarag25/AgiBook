src=open('scene_render.py').read();exec(src[:src.index("BOOKS=[")])
BOOKS=[("hunger_games_book",0.205),("it_book",0.235),("ballata_usignolo_book",0.21),("alba_mietitura_book",0.21)]
cam=lambda f,d,el,az:(f[0]+d*math.cos(el)*math.cos(az),f[1]+d*math.cos(el)*math.sin(az),f[2]+d*math.sin(el))
for k,h in BOOKS:
    a=load(f"books/{k}.glb",lambda t,h=h:t.Translate(0,0,h/2))     # in piedi, copertina verso +y
    f=(0,0,h/2)
    shoot(a,f"{k}.png",cam(f,0.85,math.radians(12),math.radians(70)),f,1200,1500,28)   # come gli oggetti: ~20 gradi di lato, un po' dall'alto
