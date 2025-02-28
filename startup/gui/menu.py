import GafferUI

import GafferScotch
import GafferScotchUI

nodeMenu = GafferUI.NodeMenu.acquire(application)
nodeMenu.append(
    "/GafferScotch/CaptureWeight",
    GafferScotch.CaptureWeight,
    searchText="CaptureWeight",
)
nodeMenu.append(
    "/GafferScotch/AttachCurves",
    GafferScotch.AttachCurves,
    searchText="AttachCurves",
)
nodeMenu.append(
    "/GafferScotch/PointDeform",
    GafferScotch.PointDeform,
    searchText="PointDeform",
)
