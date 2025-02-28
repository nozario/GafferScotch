import GafferUI

import GafferScotch
import GafferScotchUI

nodeMenu = GafferUI.NodeMenu.acquire(application)
nodeMenu.append(
    "/GafferScotch/CaptureWeights",
    GafferScotch.CaptureWeights,
    searchText="CaptureWeights",
)
nodeMenu.append(
    "/GafferScotch/PointDeform",
    GafferScotch.PointDeform,
    searchText="PointDeform",
)

nodeMenu.append(
    "/GafferScotch/RigidAttachCurves",
    GafferScotch.RigidAttachCurves,
    searchText="RigidAttachCurves",
)

nodeMenu.append(
    "/GafferScotch/RigidDeformCurves",
    GafferScotch.RigidDeformCurves,
    searchText="RigidDeformCurves",
)
