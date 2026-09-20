import pickle
from PIL import Image, ImageDraw, ImageFont
D=pickle.load(open("lineup.pkl","rb"));S=D["S"];S2=D["S2"]
F="/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf";FB="/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
f=ImageFont.truetype(F,17);fs=ImageFont.truetype(F,15);fb=ImageFont.truetype(FB,20);ft=ImageFont.truetype(FB,26)
INK=(40,44,52);DIM=(200,60,40);BG=(250,250,251)
# (larghezza x profondita' x altezza in m) mostrate nelle quote
BIG={"robot":("Robot AgiBot X2 (x2_t2.5)",0.622,0.309,1.315),"shelf":("Libreria",0.800,0.300,1.350),"table":("Tavolo",1.200,0.900,0.750)}
SMALL={"Hunger Games":(0.200,0.070,0.205),"IT":(0.155,0.055,0.235),"Ballata":(0.150,0.045,0.210),"Alba":(0.150,0.038,0.210),
       "Portapenne":(0.056,0.056,0.095),"Mappamondo":(0.080,0.080,0.122)}
FULL={"Hunger Games":"Hunger Games (trilogia)","Ballata":"Ballata dell'usignolo","Alba":"Alba sulla mietitura","IT":"IT"}
def hdim(d,x0,x1,y,txt):
    d.line([(x0,y),(x1,y)],fill=DIM,width=2);[d.line([(x,y-7),(x,y+7)],fill=DIM,width=2) for x in (x0,x1)]
    w=d.textlength(txt,font=fs);d.rectangle([((x0+x1)/2-w/2-3,y+6),((x0+x1)/2+w/2+3,y+26)],fill=BG);d.text(((x0+x1)/2-w/2,y+7),txt,font=fs,fill=DIM)
def vdim(d,x,y0,y1,txt):
    d.line([(x,y0),(x,y1)],fill=DIM,width=2);[d.line([(x-7,y),(x+7,y)],fill=DIM,width=2) for y in (y0,y1)]
    d.text((x+8,(y0+y1)/2-9),txt,font=fs,fill=DIM)
m=lambda v:f"{v*100:.1f} cm" if v<1 else f"{v:.3f} m"
# ---- pannello 1: stessa scala
big=[(k,Image.frombytes("RGBA",sz,b)) for n,k,b,sz in D["big"]]
gap=150;left=80;ground=90+max(i.height for k,i in big)
W1=left+sum(i.width for k,i in big)+gap*len(big)+40;H1=ground+140
p1=Image.new("RGBA",(W1,H1),BG);d=ImageDraw.Draw(p1)
d.text((left,18),"Vista frontale – stessa scala",font=ft,fill=INK)
x=left
for k,im in big:
    p1.alpha_composite(im,(x,ground-im.height))
    name,w,dp,h=BIG[k]
    vdim(d,x+im.width+14,ground-im.height,ground,m(h))
    hdim(d,x,x+im.width,ground+14,m(w))
    d.text((x,ground+44),name,font=fb,fill=INK);d.text((x,ground+70),f"profondità {m(dp)}",font=fs,fill=(110,110,120))
    x+=im.width+gap
d.line([(left-30,ground),(W1-20,ground)],fill=(160,160,170),width=2)
# barra 50 cm
d.line([(W1-40-S/2,58),(W1-40,58)],fill=INK,width=4);d.text((W1-40-S/2,64),"50 cm",font=fs,fill=INK)
# ---- pannello 2: libri e oggetti, scala 3x
sm=[(n,Image.frombytes("RGBA",sz,b)) for n,b,sz in D["small"]]
gap2=120;ground2=80+max(i.height for n,i in sm)
W2=left+sum(i.width for n,i in sm)+gap2*len(sm);H2=ground2+130
p2=Image.new("RGBA",(W2,H2),BG);d=ImageDraw.Draw(p2)
d.text((left,18),"4 libri e 2 oggetti – ingranditi 3×",font=ft,fill=INK)
x=left
for n,im in sm:
    p2.alpha_composite(im,(x,ground2-im.height));w,dp,h=SMALL[n]
    vdim(d,x+im.width+12,ground2-im.height,ground2,m(h));hdim(d,x,x+im.width,ground2+14,m(w))
    d.text((x,ground2+44),FULL.get(n,n),font=ImageFont.truetype(FB,17),fill=INK)
    d.text((x,ground2+68),("spessore " if n in FULL else "profondità ")+m(dp),font=fs,fill=(110,110,120))
    x+=im.width+gap2
d.line([(left-30,ground2),(W2-20,ground2)],fill=(160,160,170),width=2)
d.line([(W2-40-S2*0.1,58),(W2-40,58)],fill=INK,width=4);d.text((W2-40-S2*0.1,64),"10 cm",font=fs,fill=INK)
W=max(W1,W2);out=Image.new("RGBA",(W,H1+H2),BG);out.alpha_composite(p1,(0,0));out.alpha_composite(p2,(0,H1))
out.convert("RGB").save("dimensions_front.png");print(out.size)
