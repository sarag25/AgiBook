"""Riordina gli oggetti nella foto: ritaglia dall'originale, incolla sullo scaffale vuoto.
Ogni oggetto resta alla SUA profondita' (stessa base y) e si sposta solo in orizzontale."""
import cv2, numpy as np
im=cv2.imread('/home/robot/SmartRobotics/x2_shelf_photo_1.jpg')
bg=cv2.imread('shelf_empty.jpg')                    # scaffale vuoto (stessa inquadratura)
H,W=im.shape[:2]
def rect_mask(x0,y0,x1,y1):
    m=np.zeros((H,W),np.uint8);cv2.rectangle(m,(x0,y0),(x1,y1),255,-1);return m
# ritagli: (nome, maschera, x_sinistra)
objs={}
objs["hunger"]=(rect_mask(1405,705,1550,1061),1405)
objs["ballata"]=(rect_mask(1160,697,1270,1062),1160)
objs["alba"]=(rect_mask(1076,697,1159,1062),1076)
objs["it"]=(rect_mask(1272,655,1402,1062),1272)
g=np.zeros((H,W),np.uint8)
cv2.circle(g,(857,912),67,255,-1);cv2.rectangle(g,(849,975),(866,1025),255,-1);cv2.ellipse(g,(857,1038),(53,22),0,0,360,255,-1)
objs["globe"]=(g,790)
p=rect_mask(961,882,1048,1029);cv2.ellipse(p,(1004,882),(43,8),0,0,360,255,-1)
objs["pen"]=(p,961)
# x sinistra di ogni oggetto nell'immagine finale.
# Libri: dalla sinistra dello scaffale (parete di fondo da x~424), 32 px (~2 cm a
# quella distanza, IT costa 5.5 cm = 95 px) fra uno e l'altro.
# Mappamondo e portapenne: stessa posizione della versione precedente.
BOOKS=["hunger","ballata","alba","it"];BOOK_X0=470;BOOK_GAP=32
POS={};x=BOOK_X0
for k in BOOKS:
    xs=np.where(objs[k][0]>0)[1];POS[k]=x;x+=xs.max()-xs.min()+1+BOOK_GAP
POS["globe"]=1189;POS["pen"]=1348
out=bg.copy()
for k in ["hunger","ballata","alba","it","globe","pen"]:
    m,_=objs[k];xs=np.where(m>0)[1];dx=POS[k]-xs.min()
    M=np.float32([[1,0,dx],[0,1,0]])
    obj=cv2.warpAffine(im,M,(W,H));mk=cv2.warpAffine(m,M,(W,H))
    a=cv2.GaussianBlur(mk,(3,3),0).astype(np.float32)[...,None]/255
    out=(obj*a+out*(1-a)).astype(np.uint8)
print("libri fino a x=",x-BOOK_GAP)
cv2.imwrite('shelf_reordered.png',out)

