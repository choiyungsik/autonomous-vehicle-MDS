from math import *
import numpy as np




def convert_degree_to_meter(lat,lon):

    alpha = lat*pi/180
    beta = lon*pi/180

    a = 6377397.155
    f = 1/299.1528128
    b = a*(1-f)
    k = 1
    x_plus = 500000
    y_plus = 200000

    e1 = (a**2-b**2)/a**2
    e2 = (a**2-b**2)/b**2
    alpha0 = 38 *pi/180
    beta0 = (125+0.002890277)*pi/180

    T = pow(tan(alpha),2)
    C = e1/(1-e1)*pow(cos(alpha),2)
    AA = (beta-beta0)*cos(alpha)  #both radian

    N = a/sqrt( 1-e1*sin(alpha)**2 )
    M = a*(alpha*(1-e1/4-3*e1**2/64-5*e1**3/256)-sin(2*alpha)*(3*e1/8+3*e1**2/32+45*e1**3/1024)
        +sin(4*alpha)*(15*e1**2/256+45*e1**3/1024)-35*e1**3*sin(6*alpha)/3072)

    M0 = a*(alpha0*(1-e1/4-3*e1**2/64-5*e1**3/256)-sin(2*alpha0)*(3*e1/8+3*e1**2/32+45*e1**3/1024)
         +sin(4*alpha0)*(15*e1**2/256+45*e1**3/1024)-35*e1**3*sin(6*alpha0)/3072)

    Y = y_plus + k*N*(AA+AA**3*(1-T+C)/6+pow(AA,5)*(5-18*T+T*T+72*C-58*e2)/120)
    X = x_plus + k*(M-M0+N*tan(alpha)*(AA*AA/2 + pow(AA,4)*(5-T+9*C+4*C*C)/24 +
        pow(AA,6)*(61-58*T+T*T+600*C-330*e2)/720))

    return [X,Y]


a=[37.6312796,127.0766479]
b=[37.6309665,127.0765384]

c=[37.630978,127.0760636]
d=[37.6310089,127.0759147]


gps_n = convert_degree_to_meter(a[0],a[1])
gps_n_1 = convert_degree_to_meter(b[0],b[1])
gps_n[0] = (gps_n[0] - 460000)/100
gps_n[1] = (gps_n[1] - 383000)/100
gps_n_1[0] = (gps_n_1[0] - 460000)/100
gps_n_1[1] = (gps_n_1[1] - 383000)/100

gps_nn = convert_degree_to_meter(c[0],c[1])
gps_nn_1 = convert_degree_to_meter(d[0],d[1])
gps_nn[0] = (gps_nn[0] - 460000)/100
gps_nn[1] = (gps_nn[1] - 383000)/100
gps_nn_1[0] = (gps_nn_1[0] - 460000)/100
gps_nn_1[1] = (gps_nn_1[1] - 383000)/100

print(atan2(gps_n_1[1]-gps_n[1], gps_n_1[0]-gps_n[0])*180/np.pi)
print(atan2(gps_nn_1[1]-gps_nn[1], gps_nn_1[0]-gps_nn[0])*180/np.pi)

##1>0
print(atan2(b[1]-a[1],b[0]-a[0])*180/np.pi)
print(atan2(d[1]-c[1],d[0]-c[0])*180/np.pi)

##0>1
print(atan2(b[0]-a[0],b[1]-a[1])*180/np.pi)
print(atan2(d[0]-c[0],d[1]-c[1])*180/np.pi)
