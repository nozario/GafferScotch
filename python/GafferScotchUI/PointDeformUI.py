import Gaffer
import IECore
import imath

import GafferScotch

from ._IconPath import ICON_PATH

Gaffer.Metadata.registerNode(
    GafferScotch.PointDeform,
    "description",
    """
    Deforms geometry based on the weights created by CaptureWeights node.
    Takes three inputs:
    1. The mesh/points/curve to deform (input)
    2. The static reference deformer (staticDeformer)
    3. The animated deformer (animatedDeformer)

    The vertices of both deformers should match the indices baked into the capture attributes,
    otherwise the node will error.
    """,
    "icon",
    ICON_PATH,
    "nodeGadget:color",
    imath.Color3f(0.42, 0.27, 0.23),
    plugs={
        "staticDeformer": [
            "description",
            """
            The static reference deformer geometry whose points will be used as the base state.
            The number of points must match the indices stored in the capture attributes.
            """,
        ],
        "animatedDeformer": [
            "description",
            """
            The animated deformer geometry whose points will drive the deformation.
            The number of points must match the indices stored in the capture attributes.
            """,
        ],
        "deformerPath": [
            "description",
            """
            The path to both the static and animated deformer objects in their respective scenes.
            Both deformers must exist at the same path in their respective inputs.
            """,
        ],
        "cleanupAttributes": [
            "description",
            """
            When enabled, removes all capture attributes (captureInfluences, captureIndex*, captureWeight*)
            from the output geometry after deformation is applied.
            """,
        ],
    },
)
