import GafferUI

import GafferScotch

nodeMenu = GafferUI.NodeMenu.acquire(application)
nodeMenu.append(
    "/GafferScotch/CaptureWeights",
    GafferScotch.CaptureWeights,
    searchText="CaptureWeights",
)
nodeMenu.append(
    "/GafferScotch/CaptureMatrixWeights",
    GafferScotch.CaptureMatrixWeights,
    searchText="CaptureMatrixWeights",
)
nodeMenu.append(
    "/GafferScotch/PointDeform",
    GafferScotch.PointDeform,
    searchText="PointDeform",
)
nodeMenu.append(
    "/GafferScotch/MatrixDeform",
    GafferScotch.MatrixDeform,
    searchText="MatrixDeform",
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
nodeMenu.append(
    "/GafferScotch/CurvesPostProcess",
    GafferScotch.CurvesPostProcess,
    searchText="CurvesPostProcess",
)
nodeMenu.append(
    "/GafferScotch/FeatherAttachBarbs",
    GafferScotch.FeatherAttachBarbs,
    searchText="FeatherAttachBarbs",
)
nodeMenu.append(
    "/GafferScotch/FeatherDeformBarbs",
    GafferScotch.FeatherDeformBarbs,
    searchText="FeatherDeformBarbs",
)
nodeMenu.append(
    "/GafferScotch/CurvesToCurvesAttach",
    GafferScotch.CurvesToCurvesAttach,
    searchText="CurvesToCurvesAttach",
)

nodeMenu.append(
    "/GafferScotch/CurvesToCurvesDeform",
    GafferScotch.CurvesToCurvesDeform,
    searchText="CurvesToCurvesDeform",
)
