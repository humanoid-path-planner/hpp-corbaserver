#/usr/bin/env python
import time
from projected_volume import ProjectedVolume

pv = ProjectedVolume()
pv.setConfig(pv.q2)
pv.compute()
pv.displayRobot()
pv.computeConvexHull()
