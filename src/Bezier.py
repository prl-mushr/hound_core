#!/usr/bin/env python3

import numpy as np
import math as m

RAD2DEG = 57.3
DEG2METER = 111392.84
DEG2RAD = 1/57.3

track_width = 2
max_curvature = 0.3
ratio = 0.35

def set_track_width(tw):
    global track_width
    track_width = (tw)/ratio

def distancecalcy(y1,y2,x1,x2):
    delX = (x2-x1);
    delY = (y2-y1);
    delX *= delX;
    delY *= delY;
    return m.sqrt(delX + delY);   

def anglecalcy(x1,x2,y1,y2):
    angle = RAD2DEG*m.atan2((y2-y1),(x2-x1));
    if(angle<0):
        angle += 360;
    return angle;

def angle_difference(x1,x2,x3,y1,y2,y3):
    angle1 = anglecalcy(x1,x2,y1,y2)
    angle2 = anglecalcy(x2,x3,y2,y3)
    angle_diff = m.fabs(angle1-angle2)
    if(angle_diff>360):
        angle_diff -= 360
    return angle_diff

def wrap_360(angle):
    if(angle>360):
        angle -= 360
    if(angle<0):
        angle += 360
    return angle

def generate_slopes(X,Y):
    circuit = False
    if(distancecalcy(Y[0],Y[-1],X[0],X[-1])<1):
        circuit = True
    slope = np.empty_like(X)
    for i in range(1,len(X)-1):
        angle1 = anglecalcy( X[i-1], X[i], Y[i-1], Y[i] )
        angle2 = anglecalcy( X[i], X[i+1], Y[i], Y[i+1] )
        if(m.fabs(angle1 - angle2) > 180):
            angle1 -= 360
        # if((i-1)%3==0 and i>=1):
        #   slope[i]=angle1
        # elif((i-3)%3==0 and i>=3):
        #   slope[i]=angle2
        # else:
        slope[i] = ( angle1 + angle2 )*0.5

    if(circuit):
        angle1 = anglecalcy( X[-2], X[-1], Y[-2], Y[-1] )
        angle2 = anglecalcy( X[0], X[1], Y[0], Y[1] )
        if(m.fabs(angle1 - angle2) > 180):
            angle1 -= 360
        slope[0]  =  ( angle1 + angle2 )*0.5;
        slope[-1] = slope[0]
    else:
        slope[0] = anglecalcy( X[0], X[1], Y[0], Y[1] );
        slope[-1] = anglecalcy( X[-2], X[-1], Y[-2], Y[-1] )

    return slope

def generate_angle_diff(X,Y):
    circuit = False
    if(distancecalcy(Y[0],Y[-1],X[0],X[-1])<1):
        circuit = True
    slope = np.empty_like(X)
    for i in range(1,len(X)-1):
        angle1 = anglecalcy( X[i-1], X[i], Y[i-1], Y[i] )
        angle2 = anglecalcy( X[i], X[i+1], Y[i], Y[i+1] )
        if(m.fabs(angle1 - angle2) > 180):
            angle1 -= 360
        # if((i-1)%3==0 and i>=1):
        #   slope[i]=angle1
        # elif((i-3)%3==0 and i>=3):
        #   slope[i]=angle2
        # else:
        slope[i] = wrap_360( m.fabs( angle1 - angle2 ) )

    if(circuit):
        angle1 = anglecalcy( X[-2], X[-1], Y[-2], Y[-1] )
        angle2 = anglecalcy( X[0], X[1], Y[0], Y[1] )
        if(m.fabs(angle1 - angle2) > 180):
            angle1 -= 360
        slope[0]  =  wrap_360(m.fabs(angle1 - angle2))
        slope[-1] = slope[0]
    else:
        slope[0] = 0
        slope[-1] = 0

    return slope

def acute_angle(A,B):
    a = m.fabs(A-B)
    while(a>180):
        a -= 180
    return a


def area(x1, y1, angle1, x2, y2, angle2):
    X = distancecalcy(y1,y2,x1,x2)
    base = anglecalcy(x1,x2,y1,y2)
    B = acute_angle(angle1,base)*DEG2RAD
    C = acute_angle(angle2,base)*DEG2RAD
    A = m.pi - (B+C)
    return m.fabs(((m.sin(B)*m.sin(C)/m.sin(A))))*X**2

def get_Intermediate_Points(slope1, slope2, X1, X2, Y1, Y2):
    global track_width
    global ratio
    int1 = np.zeros(2)
    int2 = np.zeros(2)
    d = distancecalcy(Y2,Y1,X2,X1)
    # ratio = 0.4 - 0.06*(d/(track_width+d))
    # if(d>track_width/ratio):
    #   d = track_width/ratio
    int1[0] = X1 + ratio*m.cos(slope1*DEG2RAD)*d
    int1[1] = Y1 + ratio*m.sin(slope1*DEG2RAD)*d
    int2[0] = X2 - ratio*m.cos(slope2*DEG2RAD)*d
    int2[1] = Y2 - ratio*m.sin(slope2*DEG2RAD)*d
    return int1,int2

def jerk_optimal(P0,P3, a, b):
    P = P3 - P0
    L = np.linalg.norm(P)
    P /= L
    A = a/np.linalg.norm(a)
    B = b/np.linalg.norm(b)
    k1 = 0.35
    k2 = 0.35
    A_B = np.dot(A - B, A - B)
    if(A_B != 0):
        k = min((2.0/3.0)*np.dot(P, A - B)/A_B, 0.5)
    else:
        k = 0.33
    k1 = k2 = k
    return k1,k2


def get_Intermediate_Points_generic(P0,P3,Vhat0,Vhat3,speed,compliment=False, jerk_opt=False):
    global ratio
    d = np.linalg.norm(P3-P0)
    if(not compliment):
        if(jerk_opt):
            k1, k2 = jerk_optimal(P0, P3, Vhat0, -Vhat3)
            P1 = P0 + k1*d*Vhat0
            P2 = P3 - k2*d*Vhat3
        else:
            if(d > speed*100):
                d = speed*100
            P1 = P0 + ratio*d*Vhat0
            P2 = P3 - ratio*d*Vhat3
    else:
        P1 = P0 + 0.1*d*Vhat0
        P2 = P3 - 0.9*d*Vhat3
    return P1,P2

def get_bezier(P0,P1,P2,P3,N):
    t = np.arange(0,1,1/N)
    T = np.array([(1-t)**3,3*t*(1-t)**2,3*t*t*(1-t),t**3])
    Bx = T[0]*P0[0] + T[1]*P1[0] + T[2]*P2[0] + T[3]*P3[0]
    By = T[0]*P0[1] + T[1]*P1[1] + T[2]*P2[1] + T[3]*P3[1]
    Bz = T[0]*P0[2] + T[1]*P1[2] + T[2]*P2[2] + T[3]*P3[2]
    return Bx,By,Bz

def get_bezier_coeffs(X1,X2,Y1,Y2,slope1,slope2):
    int1,int2 = get_Intermediate_Points(slope1,slope2,X1,X2,Y1,Y2)
    C = np.array([int2[0]*2.5,X2*2,int1[1]*2,int2[1],Y2])
    return C

def plot_bezier_coeffs(C):
    Px = np.array([0,0,C[0]/2.5,C[1]/2])
    Py = np.array([0,C[2]/2,C[3],C[4]])
    t = np.arange(0,0.2,0.01)
    T = np.array([(1-t)**3,3*t*(1-t)**2,3*t*t*(1-t),t**3])
    Bx = T[0]*Px[0] + T[1]*Px[1] + T[2]*Px[2] + T[3]*Px[3]
    By = T[0]*Py[0] + T[1]*Py[1] + T[2]*Py[2] + T[3]*Py[3]
    return Bx,By

def find_bezier(x,y):
    slope2 = RAD2DEG*m.atan2(y[-1]-y[-2],x[-1]-x[-2])
    slope1 = (m.pi/2)*RAD2DEG
    C = get_bezier_coeffs(x[0],x[-1],y[0],y[-1],slope1,slope2)
    Bx,By = get_bezier(x[0],x[-1],y[0],y[-1],slope1,slope2)
    return C, Bx,By

def arc_length(X1,Y1,X2,Y2,X3,Y3,X4,Y4):
    L1 = distancecalcy(Y1,Y2,X1,X2)
    L2 = distancecalcy(Y2,Y3,X2,X3)
    L3 = distancecalcy(Y3,Y4,X3,X4)
    L4 = distancecalcy(Y4,Y1,X4,X1)
    L = L1+L2+L3
    L = 0.5*(L+L4)
    return L

def arc_length_generic(P0,P1,P2,P3):
    L1 = np.linalg.norm(P1-P0)
    L2 = np.linalg.norm(P2-P1)
    L3 = np.linalg.norm(P3-P2)
    L4 = np.linalg.norm(P3-P0)
    L = (L1+L2+L3+L4)*0.5
    return L

def get_T(X1,Y1,X2,Y2,X3,Y3,X4,Y4):
    L1 = distancecalcy(Y1,Y2,X1,X2)
    L2 = distancecalcy(Y2,Y3,X2,X3)
    L3 = distancecalcy(Y3,Y4,X3,X4)
    L4 = distancecalcy(Y4,Y1,X4,X1)
    L = L1+L2+L3
    L = 0.5*(L+L4)
    t1 = 0.5*(L1/(L1+L2))
    t2 = 1 - 0.5*(L3/(L3+L2))
    return np.array([t1,t2])

def get_T_generic(P0,P1,P2,P3,V,future_time):
    L1 = np.linalg.norm(P1-P0)
    L2 = np.linalg.norm(P2-P1)
    L3 = np.linalg.norm(P3-P2)
    L4 = np.linalg.norm(P3-P0)
    L = (L1+L2+L3+L4)*0.5
    t1 = 0.5*(L1/(L1+L2))
    t2 = 1 - 0.5*(L3/(L3+L2))
    t = V*future_time/L
    t = max(t,0.001) # 0.1 % lookahead minimum
    return np.array([t,t1,t2])

def Curv(t,KX1,KX2,KX3,KY1,KY2,KY3):
    delX = t*t*KX1 + t*KX2 + KX3
    delY = t*t*KY1 + t*KY2 + KY3
    del2X = 2*t*KX1 + KX2
    del2Y = 2*t*KY1 + KY2
    denominator = delX*delX + delY*delY
    dummy = denominator
    denominator *= denominator*denominator
    denominator = m.sqrt(denominator)
    del3Y = 2*KY1
    del3X = 2*KX1
    second_denominator = denominator*dummy 
    dK = ((del3Y*delX - del3X*delY)/denominator) - (3*(delX*del2Y - del2X*delY)*(delX*del2X + delY*del2Y)/second_denominator)
    sub_term_1 = (delX*del2Y - del2X*delY)
    sub_term_2 = 2*(delX*del2X + delY*del2Y)
    third_denominator = m.fabs(second_denominator*dummy)
    sub_term_3 = (del3Y*delX - del3X*delY)
    sub_term_4 = 2*(del2X**2 + del2Y**2 + del3X*delX+del3Y*delY)
    sub_term_5 = - del3X*del2Y + del3Y*del2X
    term_1 = 3.75*(sub_term_1*(sub_term_2**2))/third_denominator
    term_2 = -3*(sub_term_3*sub_term_2)/second_denominator
    term_3 = -1.5*(sub_term_1*sub_term_4)/second_denominator
    term_4 = sub_term_5/denominator
    d2K = term_1 + term_2 + term_3 + term_4
    return dK,d2K

def check_range(x,i):
    if(i):
        if(x>1):
            return 1
        if(x<0.5):
            return 0.5
        return x
    if(x<0):
        return 0
    if(x>0.5):
        return 0.5
    return x

def C_from_K_t(t,KX1,KX2,KX3,KY1,KY2,KY3):
    delX = t*t*KX1 + t*KX2 + KX3
    delY = t*t*KY1 + t*KY2 + KY3
    del2X = 2*t*KX1 + KX2
    del2Y = 2*t*KY1 + KY2
    denominator = delX*delX + delY*delY
    dummy = denominator
    denominator *= denominator*denominator
    denominator = m.sqrt(denominator)
    Curvature = ((delX*del2Y) - (delY*del2X))
    Curvature /= denominator
    return Curvature

def cmp(a,b):
    return (a > b) ^ (a < b)

def get_bezier_point(P0,P1,P2,P3,t):
    T = np.array([(1-t)**3,3*t*(1-t)**2,3*t*t*(1-t),t**3])
    return T[0]*P0 + T[1]*P1 + T[2]*P2+ T[3]*P3

def get_CTN(P1,P2,P3,P4,t):
    KP1 = 9*P2 + 3*P4 - 3*P1 - 9*P3 # see how much shorter everything is when we use vector notation?
    KP2 = 6*P1 - 12*P2 + 6*P3
    KP3 = 3*(P2 - P1)
    B1 = t*t*KP1 + t*KP2 + KP3 # first derivative (also tangent) . Up to this line is the get-tangent function
    B2 = 2*t*KP1 + KP2 # second derivative 
    BNormal = np.cross(B1,B2) # binormal
    k_num = np.linalg.norm(BNormal) # numerator for curvature 
    BNormal /= k_num # please don't be zero?
    k_den = np.linalg.norm(B1)**3 # denominator for curvature
    C = k_num/k_den # curvature 
    T = B1/np.linalg.norm(B1) # unit tangent vector
    N = np.cross(BNormal,T) # unit normal vector
    return C,T,N

def get_Curvature_generic(P1,P2,P3,P4,t):
    KP1 = 9*P2 + 3*P4 - 3*P1 - 9*P3 # see how much shorter everything is when we use vector notation?
    KP2 = 6*P1 - 12*P2 + 6*P3
    KP3 = 3*(P2 - P1)
    B1 = t*t*KP1 + t*KP2 + KP3 # first derivative (also tangent) . Up to this line is the get-tangent function
    B2 = 2*t*KP1 + KP2 # second derivative 
    BNormal = np.cross(B1,B2) # binormal
    k_num = np.linalg.norm(BNormal) # numerator for curvature 
    k_den = np.linalg.norm(B1)**3 # denominator for curvature
    C = k_num/k_den # curvature
    return C

def get_Curvature(X1,Y1,X2,Y2,X3,Y3,X4,Y4,t):
    Px = np.array([X1,X2,X3,X4])
    Py = np.array([Y1,Y2,Y3,Y4])
    KX1 = 9*X2 + 3*X4 - 3*X1 - 9*X3
    KY1 = 9*Y2 + 3*Y4 - 3*Y1 - 9*Y3
    KX2 = 6*X1 - 12*X2 + 6*X3
    KY2 = 6*Y1 - 12*Y2 + 6*Y3
    KX3 = 3*(X2 - X1)
    KY3 = 3*(Y2 - Y1)
    Curvature = C_from_K_t(t,KX1,KX2,KX3,KY1,KY2,KY3)
    return Curvature

def s_k(X, Y, slope1, destX, destY, slope2):
    int1,int2 = get_Intermediate_Points( slope1, slope2, X, destX, Y, destY)
    t = get_T(X, Y, int1[0], int1[1], int2[0], int2[1], destX, destY)
    Curvature = np.max(np.fabs(get_Curvature(X, Y, int1[0], int1[1], int2[0], int2[1], destX, destY, t)))
    Curvature *= max((Curvature/max_curvature)**2,1)
    s = arc_length(X, Y, int1[0], int1[1], int2[0], int2[1], destX, destY)
    return Curvature/s

def get_bezier_track(X,Y,slope):
    bx = np.zeros(len(X)*10)
    by = np.zeros(len(X)*10)
    for i in range(len(X)-1):
        k = i*10
        bx[k:k+10],by[k:k+10] = get_bezier(X[i],X[i+1],Y[i],Y[i+1],slope[i],slope[i+1])
    return bx,by