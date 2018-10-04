from hpp import Quaternion
from math import sqrt
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

    print "q1",str(q1)
    print "q2",str(q2)
    print "rpy", r,p,y
    qdiff = q1*q2.inv()
    if qdiff.array[3] < 0: qdiff.array *= -1
    if not (np.abs(qdiff.array - np.array([0,0,0,1])) < 1e-6).all():
        print "ERROR"
        ok=False
    print ""

assert ok
