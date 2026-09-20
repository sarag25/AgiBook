import cv2, numpy as np
im=cv2.imread('/home/robot/SmartRobotics/x2_shelf_photo_1.jpg');H,W=im.shape[:2]
mask=np.zeros((H,W),np.uint8)
cv2.ellipse(mask,(857,905),(72,72),0,0,360,255,-1)          # mappamondo
cv2.rectangle(mask,(798,955),(917,1066),255,-1)              # piedistallo
cv2.rectangle(mask,(953,862),(1057,1037),255,-1)             # portapenne
cv2.rectangle(mask,(1068,648),(1564,1070),255,-1)            # libri
cv2.rectangle(mask,(1470,648),(1604,1040),255,-1)            # parete dx intera (venatura uniforme)
WALL_X=1470
out=im.copy()
# parete di fondo + piano: venatura orizzontale -> riempio riga per riga con
# una striscia pulita della stessa riga, ripetuta a specchio (niente cuciture)
S0,S1=430,785
band=S1-S0
# profilo di luminosita' orizzontale della parete di fondo, misurato sulla
# fascia pulita sopra gli oggetti (righe 540-640), liscio
prof=im[540:640,:WALL_X].astype(np.float32).mean(0)
prof=cv2.GaussianBlur(prof.reshape(1,-1,3),(0,0),25).reshape(-1,3)
for y in range(H):
    xs=np.where(mask[y,:WALL_X]>0)[0]
    if len(xs)==0: continue
    k=(xs-S1)%(2*band)
    src=np.where(k<band, S1-1-k, S0+(k-band))
    out[y,xs]=np.clip(im[y,src].astype(np.float32)*prof[xs]/prof[src],0,255).astype(np.uint8)
# parete interna destra: riempio dall'alto (stessa colonna, stessa parete)
B=80
def y_junction(x): return 990+(x-WALL_X)*(1070-990)/(1610-WALL_X)
for x in range(WALL_X,1606):
    ys=np.where(mask[:,x]>0)[0]
    if len(ys)==0: continue
    top=ys.min();yj=y_junction(x)
    w=ys[ys<yj];f=ys[ys>=yj]
    if len(w):
        k=(w-top)%(2*B);out[w,x]=im[np.where(k<B, top-1-k, top-B+(k-B)),x]
    for y in f:
        kk=(x-S1)%(2*band);out[y,x]=im[y, S1-1-kk if kk<band else S0+(kk-band)]
# raccordo morbido lungo i bordi della maschera
# Poisson: tiene la texture riempita ma aggancia la luminosita' ai bordi originali
fill=out.copy()   # out = originale con il riempimento gia incollato: i bordi fra le due parti sono coerenti
for part in [(0,WALL_X),(WALL_X,W)]:
    m=np.zeros_like(mask);m[:,part[0]:part[1]]=mask[:,part[0]:part[1]]
    m=cv2.dilate(m,np.ones((9,9),np.uint8))
    if part[0]==WALL_X: m[:,:WALL_X]=0
    else: m[:,WALL_X:]=0
    x,y,w,h=cv2.boundingRect(m)
    out=cv2.seamlessClone(fill,out,m,(x+w//2,y+h//2),cv2.NORMAL_CLONE)
cv2.imwrite('shelf_empty.jpg',out,[cv2.IMWRITE_JPEG_QUALITY,95])
cv2.imwrite('shelf_empty_crop.jpg',out[430:1120,700:1680])
