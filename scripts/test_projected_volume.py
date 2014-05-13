#/usr/bin/env python
import time
from projected_volume import ProjectedVolume

pv = ProjectedVolume()
pv.setConfig(pv.q1)
pv.compute()
pv.display()
pv.computeConvexHull()
