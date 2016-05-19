#!usr/bin/python#coding=utf-8
import numpy as np
import matplotlib.pyplot as plt
from math import *
from scipy import interpolate
import math
import random
import string

x1=y1=y2=0
x2=300
x3=350
y3=400
x4=10
y4=480
sigma=10
avgerr=[]
orierr=[]
err=[]
naierr=[]
def fistcalcute(x1,x2,x3,y1,y2,y3,dist14,dist24,dist34):
    a = (x2-x1)*(x2-x1)+(x3-x2)*(x3-x2)+(x1-x3)*(x1-x3)
    d = (y2-y1)*(y2-y1)+(y3-y2)*(y3-y2)+(y1-y3)*(y1-y3)
    b = (x2-x1)*(y2-y1)+(x3-x2)*(y3-y2)+(x1-x3)*(y1-y3)
    c = b
    delta = a*d-b*c
    b1 = 0.5*(dist14**2 - dist24**2 + (x2-x1)*(x2-x1)+ (y2-y1)*(y2-y1))
    b2 = 0.5*(dist24**2 - dist34**2 + (x3-x2)*(x3-x2)+ (y3-y2)*(y3-y2))
    b3 = 0.5*(dist34**2 - dist14**2 + (x1-x3)*(x1-x3)+ (y1-y3)*(y1-y3))
    B1 = (x2-x1)*(b1+x1*x2+y1*y2-x1*x1-y1*y1)+(x3-x2)*(b2+x2*x3+y2*y3-x2*x2-y2*y2)+(x1-x3)*(b3+x1*x3+y1*y3-x3*x3-y3*y3);
    B2 = (y2-y1)*(b1+x1*x2+y1*y2-x1*x1-y1*y1)+(y3-y2)*(b2+x2*x3+y2*y3-x2*x2-y2*y2)+(y1-y3)*(b3+x1*x3+y1*y3-x3*x3-y3*y3);
    return round((d * B1 - b * B2) / delta), round((-c * B1 + a * B2) / delta)

listx2=[]
listy2=[]
listx3=[]
listy3=[]
listx4=[]
listy4=[]

lx2=[]
ly2=[]
lx3=[]
ly3=[]
lx4=[]
ly4=[]

for sigma in range(0,20):
    for i in range(10):
        x1=y1=y2=0
        x2=300
        x3=350
        y3=400
        x4=10
        y4=480

        dist12=0.5*(sqrt((x2-x1)**2+(y2-y1)**2)+random.gauss(0,sigma)+sqrt((x2-x1)**2+(y2-y1)**2)+random.gauss(0,sigma))
        dist13=0.5*(sqrt((x3-x1)**2+(y3-y1)**2)+random.gauss(0,sigma)+sqrt((x3-x1)**2+(y3-y1)**2)+random.gauss(0,sigma))
        dist14=0.5*(sqrt((x4-x1)**2+(y4-y1)**2)+random.gauss(0,sigma)+sqrt((x4-x1)**2+(y4-y1)**2)+random.gauss(0,sigma))
        dist23=0.5*(sqrt((x2-x3)**2+(y2-y3)**2)+random.gauss(0,sigma)+sqrt((x2-x3)**2+(y2-y3)**2)+random.gauss(0,sigma))
        dist24=0.5*(sqrt((x2-x4)**2+(y2-y4)**2)+random.gauss(0,sigma)+sqrt((x2-x4)**2+(y2-y4)**2)+random.gauss(0,sigma))
        dist34=0.5*(sqrt((x3-x4)**2+(y3-y4)**2)+random.gauss(0,sigma)+sqrt((x3-x4)**2+(y3-y4)**2)+random.gauss(0,sigma))


        x2=dist12
        pp = 1.0*(dist23 + dist13 + dist12) / 2.0
        S = sqrt(pp*(pp-dist23)*(pp-dist13)*(pp-dist12))

        y3 = round(2 * S/dist12)
        x3 = round(sqrt(dist13 * dist13 - 2 * S/dist12 * 2 * S/dist12))


        x4,y4 = fistcalcute(x1,x2,x3,y1,y2,y3,dist14,dist24,dist34)
        lx2.append(x2)
        ly2.append(y2)
        lx3.append(x3)
        ly3.append(y3)
        lx4.append(x4)
        ly4.append(y4)
        print x2,y2,x3,y3,x4,y4
        ss=sqrt((x2-300)**2+(x3-350)**2+(x4-10)**2+(y3-400)**2+(y4-480)**2)
        orierr.append(ss)
        rate = 0.5

        for i in range(10):

            dist_12=sqrt((x2-x1)**2+(y2-y1)**2)
            dist_13=sqrt((x3-x1)**2+(y3-y1)**2)
            dist_14=sqrt((x4-x1)**2+(y4-y1)**2)
            dist_23=sqrt((x2-x3)**2+(y2-y3)**2)
            dist_24=sqrt((x2-x4)**2+(y2-y4)**2)
            dist_34=sqrt((x3-x4)**2+(y3-y4)**2)
            tx2 = x2 - rate *((x2-x1)*(1-dist12/dist_12)+(x2-x3)*(1-dist23/dist_23)+(x2-x4)*(1-dist24/dist_24))
            tx3 = x3 - rate *((x3-x1)*(1-dist13/dist_13)+(x3-x2)*(1-dist23/dist_23)+(x3-x4)*(1-dist34/dist_34))
            tx4 = x4 - rate *((x4-x1)*(1-dist14/dist_14)+(x4-x2)*(1-dist24/dist_24)+(x4-x3)*(1-dist34/dist_34))
            ty3 = y3 - rate *((y3-y1)*(1-dist13/dist_13)+(y3-y2)*(1-dist23/dist_23)+(y3-y4)*(1-dist34/dist_34))
            ty4 = y4 - rate *((y4-y1)*(1-dist14/dist_14)+(y4-y2)*(1-dist24/dist_24)+(y4-y3)*(1-dist34/dist_34))
            x2 = tx2
            x3 = tx3
            x4 = tx4
            y3 = ty3
            y4 = ty4
            listx2.append(x2)
            listx3.append(x3)
            listx4.append(x4)
            listy2.append(y2)
            listy3.append(y3)
            listy4.append(y4)

        s=sqrt((x2-300)**2+(x3-350)**2+(x4-10)**2+(y3-400)**2+(y4-480)**2)

        err.append(s)
        print x2,y2,x3,y3,x4,y4
    avgerr.append(sum(err)/len(err))
    naierr.append(sum(orierr)/len(orierr))

plt.figure(1)
plt.scatter(lx2,ly2,color='m')
plt.scatter(lx3,ly3,color='c')
plt.scatter(lx4,ly4,color='r')
plt.scatter([0,300,350,10],[0,0,400,480],color='b')
plt.title("Naive calculation")
plt.grid(True)

plt.figure(2)
plt.scatter([0],[0],color='b')
plt.scatter(listx2,listy2,color='m')
plt.scatter(listx3,listy3,color='c')
plt.scatter(listx4,listy4,color='r')

plt.scatter([0,300,350,10],[0,0,400,480],color='b')
plt.title("Gradient calculation")
plt.grid(True)


plt.figure(3)
l=range(0,20)
plt.scatter(l,avgerr,color='g')
plt.plot(l,avgerr)
plt.scatter(l,naierr,color='m')
plt.plot(l,naierr)
plt.grid(True)
plt.title("Err verus Variance")
plt.legend(Loc='best')
plt.xlabel("Variance")
plt.ylabel("Err")
plt.show()
