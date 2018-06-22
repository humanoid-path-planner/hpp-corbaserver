from hpp import Quaternion
import numpy as np

for q in (
        [ 0.5,-0.5,-0.5,-0.5],
        [-0.5,-0.5, 0.5,-0.5],
        ):

    q = Quaternion(q)
    r,p,y = q.toRPY()

    qq = Quaternion()
    qq.fromRPY(r,p,y)

    assert (q.array - qq.array < 1e-6).all()
