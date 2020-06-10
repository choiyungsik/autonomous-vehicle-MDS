from math import *
import numpy as np

from pyproj import Proj
from pyproj import transform

WGS84 = { 'proj':'latlong', 'datum':'WGS84', 'ellps':'WGS84', }

# conamul
TM127 = { 'proj':'tmerc', 'lat_0':'38N', 'lon_0':'127.0028902777777777776E',
   'ellps':'bessel', 'x_0':'200000', 'y_0':'500000', 'k':'1.0',
   'towgs84':'-146.43,507.89,681.46'}

# naver
TM128 = { 'proj':'tmerc', 'lat_0':'38N', 'lon_0':'128E', 'ellps':'bessel',
   'x_0':'400000', 'y_0':'600000', 'k':'0.9999',
   'towgs84':'-146.43,507.89,681.46'}


GRS80 = { 'proj':'tmerc', 'lat_0':'38', 'lon_0':'127', 'k':1, 'x_0':200000,
    'y_0':600000, 'ellps':'GRS80', 'units':'m' }

def wgs84_to_tm128(longitude, latitude):
   return transform( Proj(**WGS84), Proj(**TM128), longitude, latitude )

def tm128_to_wgs84(x, y):
   return transform( Proj(**TM128), Proj(**WGS84), x, y )

def wgs84_to_tm127(longitude, latitude):
   return map(lambda x:2.5*x,
        transform( Proj(**WGS84), Proj(**TM127), longitude, latitude ))

def tm127_to_wgs84(x, y):
   return transform( Proj(**TM127), Proj(**WGS84), x/2.5, y/2.5 )

def grs80_to_wgs84(x, y):
   return transform( Proj(**GRS80), Proj(**WGS84), x, y )

def wgs84_to_grs80(x, y):
   return transform( Proj(**WGS84), Proj(**GRS80), y, x )

def wgs84_to_cyworld(longitude, latitude):
   x_min = 4456260.72
   y_min = 1161720.00
   long_min = 123.78323
   lat_min = 32.27345
   max_grid_length = 112721.92
   x = (longitude-long_min)*max_grid_length/3.1308 + x_min
   y = (latitude-lat_min)*max_grid_length/3.1308 + y_min
   return x, y

def cyworld_to_wgs84(x, y):
   x_min = 4456260.72;
   y_min = 1161720.00;
   long_min = 123.78323;
   lat_min = 32.27345;
   max_grid_length = 112721.92;
   longitude = long_min + (x-x_min)*3.1308 / max_grid_length;
   latitude = lat_min + (y-y_min)*3.1308 / max_grid_length;
   return longitude, latitude



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
<<<<<<< HEAD
print (convert_degree_to_meter(a[0],a[1]))
=======
>>>>>>> 022a4736f007735b494d99267f54bd8395857c07

gps_n = convert_degree_to_meter(a[0],a[1])
print(gps_n)
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

theta1=atan2(gps_n_1[1]-gps_n[1], gps_n_1[0]-gps_n[0])*180/np.pi
print(theta1)
theta2=atan2(gps_nn_1[1]-gps_nn[1], gps_nn_1[0]-gps_nn[0])*180/np.pi
print(theta2)
print(theta1-theta2)


grs80_n=wgs84_to_grs80(a[0],a[1])
grs80_n_1=wgs84_to_grs80(b[0],b[1])
print(grs80_n)
print(grs80_n_1)

print(1)
print(grs80_to_wgs84(grs80_n[0],grs80_n[1]))
print(grs80_to_wgs84(grs80_n_1[0],grs80_n_1[1]))
print(1)
grs80_nn=wgs84_to_grs80(c[0],c[1])
grs80_nn_1=wgs84_to_grs80(d[0],d[1])
print(grs80_nn)
print(grs80_nn_1)

theta11=atan2(grs80_n_1[0]-grs80_n[0],grs80_n_1[1]-grs80_n[1])*180/np.pi
theta22=atan2(grs80_nn_1[0]-grs80_nn[0],grs80_nn_1[1]-grs80_nn[1])*180/np.pi
print(theta11)
print(theta22)
print(theta11-theta22)
print()
##1>0
#print(atan2(b[1]-a[1],b[0]-a[0])*180/np.pi)
#print(atan2(d[1]-c[1],d[0]-c[0])*180/np.pi)

##0>1
#print(atan2(b[0]-a[0],b[1]-a[1])*180/np.pi)
#print(atan2(d[0]-c[0],d[1]-c[1])*180/np.pi)
