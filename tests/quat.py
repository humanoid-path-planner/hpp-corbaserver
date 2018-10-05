from hpp import Quaternion
from math import sqrt, pi, sin, cos
import numpy as np

ok=True
for q in (
        [ 0.5,-0.5,-0.5,-0.5],
        [-0.5,-0.5, 0.5,-0.5],
        [0,0, 1/sqrt(2),1/sqrt(2)],
        [0,0,-1/sqrt(2),1/sqrt(2)],
        [0,-1/sqrt(2),0,1/sqrt(2)],
        ):

    q1 = Quaternion(q)
    r,p,y = q1.toRPY()

    q2 = Quaternion()
    q2.fromRPY(r,p,y)

    print ("q1",str(q1))
    print ("q2",str(q2))
    print ("rpy", r,p,y)
    qdiff = q1*q2.inv()
    if qdiff.array[3] < 0: qdiff.array *= -1
    if not (np.abs(qdiff.array - np.array([0,0,0,1])) < 1e-6).all():
        print ("ERROR")
        ok=False
    print ("")

assert ok

for i in range (50):
    r,p,y =  np.random.uniform (low = 0, high = pi/2, size=3)
    # Roll
    qr = Quaternion (sin (r/2),0,0,cos (r/2))
    # Pitch
    qp = Quaternion (0,sin (p/2),0,cos (p/2))
    # Yaw
    qy = Quaternion (0,0,sin (y/2),cos (y/2))
    # Roll, pitch, yaw
    q = qy * qp * qr
    r_exp, p_exp, y_exp = q.toRPY ()
    if not (np.abs(np.array ([r, p, y]) - np.array ([r_exp, p_exp, y_exp])) \
            < 1e-6).all () :
        print ("ERROR")
        ok = False

assert ok
