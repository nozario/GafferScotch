import Gaffer
import IECore
import imath

import GafferScotch

from ._IconPath import ICON_PATH

Gaffer.Metadata.registerNode(
    GafferScotch.MatrixDeform,
    "description",
    """
    Advanced deformation using matrix transformations between static and animated geometry.
    
    Takes input geometry that has been prepared with CaptureMatrixWeights and applies 
    deformation using local coordinate frame transformations. This provides more natural
    results than PointDeform, especially when influence points are outside the
    target geometry.
    
    Requires:
    1. The mesh/points/curve to deform (input)
    2. The static reference deformer (staticDeformer)
    3. The animated deformer (animatedDeformer)
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
            Should be the same as the input to CaptureMatrixWeights.
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
        "rigidProjection": [
            "description",
            """
            When enabled, applies polar decomposition to the transformation matrices
            to ensure they remain rigid (preserving local shape). This can help avoid
            unwanted stretching or shearing effects and produces more natural results.
            """,
        ],
        "cleanupAttributes": [
            "description",
            """
            When enabled, removes all capture attributes (captureInfluences, captureIndex*,
            captureWeight*, captureMatrices, etc.) from the output geometry after
            deformation is applied.
            """,
        ],
    },
)
