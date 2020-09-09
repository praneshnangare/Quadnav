import cv2
import numpy as np
import time
import imutils
import math

def nothing(x):
    pass

def slopes(line):
    x1,y1,x2,y2 = line
    return (y2-y1)/(x2-x1)

def bandw():
        s = cv2.getTrackbarPos('switch','image2')
        m = cv2.getTrackbarPos('mode','image2')
        if m == 0 :
            lowerBound=np.array([0 , 0 , 0])
            upperBound=np.array([254 , 255, 243])
        else :
            h_high = cv2.getTrackbarPos('H_high','image1')
            h_low = cv2.getTrackbarPos('H_low','image1')
            s_high = cv2.getTrackbarPos('S_high','image1')
            s_low = cv2.getTrackbarPos('S_low','image1')
            v_high = cv2.getTrackbarPos('V_high','image1')
            v_low = cv2.getTrackbarPos('V_low','image1')
            lowerBound=np.array([h_low,s_low,v_low])
            upperBound=np.array([h_high,s_high,v_high])
            
        return lowerBound , upperBound , s

def find_intersection(line1, line2):
    x1, y1, x2, y2 = line1
    x3, y3, x4, y4 = line2
    g = ((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4))
    h = ((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4))
    if g != 0 and h != 0 :
        Px = ((x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4 - y3*x4))/  \
                g    
        Py = ((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4 - y3*x4))/  \
            h
        Px = np.round(Px).astype(int)
        Py = np.round(Py).astype(int) 
        return Px, Py
    else :
        return -1 , -1

    
def deswin(*argv):
    for arg in argv:
        cv2.destroyWindow(arg)

cv2.namedWindow('image1', cv2.WINDOW_AUTOSIZE |cv2.WINDOW_KEEPRATIO)
cv2.namedWindow('image2', cv2.WINDOW_AUTOSIZE |cv2.WINDOW_KEEPRATIO)

cv2.moveWindow('image1' , 1200 , 0)
cv2.moveWindow('image2' , 1200 , 500)


kernelOpen=np.ones((5,5))
kernelClose=np.ones((10,10))

cv2.createTrackbar('H_high','image1',0,255,nothing)
cv2.createTrackbar('H_low','image1',0,255,nothing)
cv2.createTrackbar('S_high','image1',0,255,nothing)
cv2.createTrackbar('S_low','image1',0,255,nothing)
cv2.createTrackbar('V_high','image1',0,255,nothing)
cv2.createTrackbar('V_low','image1',0,255,nothing)

cv2.createTrackbar('switch', 'image2',0,1,nothing)
cv2.createTrackbar('mode', 'image2',0,1,nothing)
cv2.setTrackbarPos('switch','image2',1)


fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('Navigation.avi',fourcc, 35.0, (320 , 240))

class app :
    def __init__(self) :
        self.slp = []
        self.d = 0
##        self.cam = cv2.VideoCapture(1)
        self.cam              = cv2.VideoCapture('linedetectv2.avi')
        self.font              = cv2.FONT_HERSHEY_SIMPLEX
        self.position        = (20,30)
        self.position1      = (50, 30)
        self.scale             = 0.5
        self.fcolor           = (0,0,0)
        self.ltype             = 2

    def minmax (self ):
        ls = []
        self.q = []
        self.qslp = []
        color = [200 , 100 , 100]
        ch = self.img.copy()
        x1t , y1t , x2t , y2t = [] , [] , [] , []
        sump = 0
        self.avls = []
        if len(self.slo) >0:
            for ind , sl in enumerate(self.slo):
                u=[]
                sump=0
                for mp in sl :
                    ls.append(mp[1])
                    x1 , y1 , x2 , y2 = mp[1]
                    x1t.append(x1)
                    y1t.append(y1)
                    x2t.append(x2)
                    y2t.append(y2)
                    sump= sump + mp[0]
                avgsl = sump / len(sl)
                self.avls.append(avgsl)
                if avgsl > 0 :
                    mx1 =  min(x1t)
                    my1 =  min(y1t)
                    mx2 =  max(x2t)
                    my2 =  max(y2t)
                elif avgsl < 0 :
                    mx1 =  min(x1t)
                    my1 =  max(y1t)
                    mx2 =  max(x2t)
                    my2 =  min(y2t)
                else :
                    mx1 =  min(x1t)
                    my1 =  min(y1t)
                    mx2 =  max(x2t)
                    my2 =  max(y2t)
                    
                self.q.append([mx1 , my1 , mx2  , my2])
                ls = []
                color = [100 , 255 , 200]
                x1t , y1t , x2t , y2t = [] , [] , [] , []
                slp = (my2-my1)/(mx2-mx1)
                slp = np.arctan(slp)
                slp = slp * 180/np.pi
                self.qslp.append(slp)

    
    def para(self , imgg):
        avgs = []
        ql = []
        slop = []
        self.slo = []
        flag = 0
        fl = 0
        f = 0
        self.lines = np.reshape(self.lines , (len(self.lines) , 4) , order = 'C')
        for line in self.lines:
            x1 , y1 , x2 , y2 = line
            slope = slopes(line)
            deg = np.arctan(slope)
            deg = deg * 180/np.pi
            if f==0 :
                f=1
                ql.append(deg)
                ql.append(line)
                slop.append(ql)
                self.slo.append(slop)
                ql=[]
                slop = []
                avgs.append(deg)
            else :
                for index , i in enumerate(self.slo):
                    if int(abs(deg)) in range(int(abs(avgs[index])-5) , int(abs(avgs[index])+5)):
                        ql.append(deg)
                        ql.append(line)
                        self.slo[index].append(ql)
                        avgs[index] = (abs(avgs[index]) + abs(deg))/2
                        ql = []
                        flag = 0
                        break
                    else :
                        flag = 1                    
                if flag == 1:
                    ql.append(deg)
                    ql.append(line)
                    slop.append(ql)
                    self.slo.append(slop)
                    ql = []
                    slop = []
                    avgs.append(deg)
                    flag = 0
        i=0
        j=0
        for ind , sl in enumerate(self.slo) :
            if ind%2 == 0  :
                color = [255 , 255 , i ]
                i=i+100
            else :
                color = [255,0 , j]
                j=j+100
            for s in sl :
                x1 , y1 , x2 , y2 = s[1]
                cv2.line(self.img , (x1 , y1) , (x2 , y2) , color , 3)

        self.minmax()

    def horver(self , px , py):
            x1 , y1 , x2 , y2 = self.q[0]
            x3 , y3 , x4 , y4 = self.q[1]
            l1 , r1 , u1 , d1 = 0 , 0 , 0 , 0
            l2 , r2 , u2 , d2 = 0 , 0 , 0 , 0
            self.horls = []
            self.verls = []
            if self.avls[0] > 0 :
                if abs(px-x1) < abs(px - x2):
                    li1 = a1 , b1 , a2 , b2 = px , py , x2, y2
                    deg1 = np.arctan(slopes(li1))
                    deg1 = deg1 * 180/np.pi
                    cdeg1 = 360 - deg1
                else :
                    li1 = a1 , b1 , a2 , b2 = x1 , y1 , px, py
                    deg1 = np.arctan(slopes(li1))
                    deg1 = deg1 * 180/np.pi
                    cdeg1 = 180 - deg1
            else :
                if abs(px-x1) < abs(px - x2):
                    li1 = a1 , b1 , a2 , b2 = px , py , x2, y2
                    deg1 = np.arctan(slopes(li1))
                    deg1 = deg1 * 180/np.pi
                    cdeg1 = -deg1
                    
                else :
                    li1 = a1 , b1 , a2 , b2 = x1 , y1 , px, py
                    deg1 = np.arctan(slopes(li1))
                    deg1 = deg1 * 180/np.pi
                    cdeg1 = 180 - deg1
                    
            if self.avls[1] > 0 :
                if abs(px-x3) < abs(px - x4):
                       li2 = a3 , b3 , a4 , b4 = px , py , x4, y4
                       deg2 = np.arctan(slopes(li2))
                       deg2 = deg2 * 180/np.pi
                       cdeg2 = 360 - deg2
                else :
                       li2 = a3 , b3 , a4 , b4 = x3 , y3 , px, py
                       deg2 = np.arctan(slopes(li2))
                       deg2 = deg2 * 180/np.pi
                       cdeg2 = 180 - deg2
            else :
                if abs(px-x3) < abs(px - x4):
                   li2 = a3 , b3 , a4 , b4 = px , py , x4, y4
                   deg2 = np.arctan(slopes(li2))
                   deg2 = deg2 * 180/np.pi
                   cdeg2 = -deg2

                else :
                       li2 = a3 , b3 , a4 , b4 = x3 , y3 , px, py                    
                       deg2 = np.arctan(slopes(li2))
                       deg2 = deg2 * 180/np.pi
                       cdeg2 = 180 - deg2
            ang = cdeg1 - cdeg2

            imgcopy = self.img.copy()
            diff1 = 90 - cdeg1
            diff2 = 90 - cdeg2

            if  abs(diff1) > abs(diff2):
                cv2.line(imgcopy , (a1 , b1) , (a2 , b2) , [0 , 0 , 255] , 2)
                cv2.line(imgcopy , (a3 , b3) , (a4 , b4) , [100 , 200, 255] , 2)

                if (diff2) > 5:
                    #print(abs(diff2) , ' to the right')
                    cv2.putText(self.img, str(int(abs(diff2))) , self.position, self.font, self.scale,self.fcolor,self.ltype)
                    cv2.putText(self.img, ' to the right' , self.position1, self.font, self.scale,self.fcolor,self.ltype) 
                elif diff2 < -5:
                    #print(abs(diff2) , ' to the left')
                    cv2.putText(self.img, str(int(abs(diff2))) , self.position, self.font, self.scale,self.fcolor,self.ltype)
                    cv2.putText(self.img, ' to the left' , self.position1, self.font, self.scale,self.fcolor,self.ltype) 

                else :
                    #print('go straight')
                    cv2.putText(self.img, ' Go straight' , self.position1, self.font, self.scale,self.fcolor,self.ltype) 

            elif abs(diff1) < abs(diff2):
                cv2.line(imgcopy , (a1 , b1) , (a2 , b2) , [100 , 200 , 255] , 2)
                cv2.line(imgcopy , (a3 , b3) , (a4 , b4) , [0 , 0 , 255] , 2)

                if (diff1) > 5:
                    #print(abs(diff1) , ' to the right')
                    cv2.putText(self.img, str(int(abs(diff1))) , self.position, self.font, self.scale,self.fcolor,self.ltype)
                    cv2.putText(self.img, ' to the right' , self.position1, self.font, self.scale,self.fcolor,self.ltype) 
                elif diff1 < -5:
                    #print(abs(diff1) , ' to the left')
                    cv2.putText(self.img, str(int(abs(diff1))) , self.position, self.font, self.scale,self.fcolor,self.ltype)
                    cv2.putText(self.img, ' to the right' , self.position1, self.font, self.scale,self.fcolor,self.ltype) 
                else :
                    #print('go straight')
                    cv2.putText(self.img, ' Go straight' , self.position1, self.font, self.scale,self.fcolor,self.ltype) 
            else:
                pass
                
    def run(self , flg) :
        mp = True
        count = 0
        while mp:
            mp  = True
            if flg == 0:
                ret , self.img = self.cam.read()
            else :
                self.img = cv2.imread('mailed-01.jpg' , 1)
                self.img = rot(self.img , 0)
                ret = True
                
            if ret == False:
##                self.cam = cv2.VideoCapture(1)
                self.cam = cv2.VideoCapture('linedetectv2.avi')
            else:
                self.img = cv2.resize(self.img , (int(self.img.shape[1]/2) , int(self.img.shape[0]/2)))
                imgHSV= cv2.cvtColor(self.img , cv2.COLOR_BGR2HSV)
                lowerBound , upperBound , s = bandw()
                mask=cv2.inRange(imgHSV,lowerBound,upperBound)
                maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
                maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)
                maskFinal = maskClose
                eros = cv2.erode(maskFinal , np.ones((3,3), dtype=np.uint8) , 1)
                edges = cv2.Canny(eros, 50, 150)
                self.dilated = cv2.dilate(edges , np.ones((2,2), dtype=np.uint8) )
                if s == 0:
                    cv2.imshow('dilated_edges' , self.dilated)
                    cv2.imshow('mask' , mask)
                    cv2.waitKey(1)

                else :
                    deswin('mask' , 'dilated_edges')
                    self.lines = cv2.HoughLinesP(self.dilated, rho=1, theta=np.pi/180, threshold=50, maxLineGap=15, minLineLength=10)
                    if self.lines is not None :
                        self.para(self.img)
                        b = len(self.q)
                        if b == 1:
                            slopp = self.qslp[0]
                            if abs(int(slopp)) in range(82 , 91):
                                cv2.putText(self.img, (' Go straight') , self.position1, self.font, self.scale,self.fcolor,self.ltype)
                            elif abs(int(slopp)) in range(0 , 45):    
                                #print('Its a horizontal Line')
                                pass
                            elif slopp < 0:
                                cv2.putText(self.img, str(90 - int(abs(slopp))) , self.position, self.font, self.scale,self.fcolor,self.ltype)
                                cv2.putText(self.img, '  To the right' , self.position1, self.font, self.scale,self.fcolor,self.ltype) 
                            elif slopp >0:
                                cv2.putText(self.img, str(90 - int(abs(slopp))) , self.position, self.font, self.scale,self.fcolor,self.ltype)
                                cv2.putText(self.img, '  To the left' , self.position1, self.font, self.scale,self.fcolor,self.ltype) 
                        elif b == 2:
                            px , py = find_intersection(self.q[0] , self.q[1])
                            self.horver(px , py)
                        else :
                            pass
                if count < 70 :
                    pass
                else :
                    #out.write(self.img)
                    cv2.imshow('init_img' , self.img)
                count = count +1
                if cv2.waitKey(5) & 0xFF == ord('q') or count >760:
                    self.cam.release()
                    out.release()
                    cv2.destroyAllWindows()
                    break  

app().run(0)



            

            
            
        
        

        

