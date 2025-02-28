import IECore
import imath

import Gaffer
import GafferScotch
from . import ICON_PATH

Gaffer.Metadata.registerNode(
    GafferScotch.RigidAttachCurves,
    "description",
    """
    Computes and stores binding data for curves to a mesh, using a rigid frame-based approach.
    This node focuses on the binding computation and caches the results as primitive variables
    for later use by the RigidDeformCurves node.
    """,
    "icon",
    ICON_PATH,
    "nodeGadget:color",
    imath.Color3f(0.42, 0.27, 0.23),
    plugs={
        "staticDeformer": [
            "description",
            """
            The static mesh to bind the curves to. This mesh defines the initial
            binding positions and frames.
            """,
            "nodule:type",
            "GafferUI::StandardNodule",
        ],
        "curveRootAttr": [
            "description",
            """
            The name of a vertex float attribute that defines the root point
            of each curve. Values should be between 0 and 1, with 1 indicating
            the root point. If empty, the first point of each curve is used.
            """,
        ],
        "useBindAttr": [
            "description",
            """
            When enabled, uses a string attribute on the curves to determine
            which mesh to bind to. When disabled, uses a single mesh path
            for all curves.
            """,
        ],
        "deformerPath": [
            "description",
            """
            The path to the mesh to bind to when useBindAttr is disabled.
            """,
        ],
        "bindAttr": [
            "description",
            """
            The name of the string attribute that specifies which mesh to
            bind to when useBindAttr is enabled.
            """,
        ],
    },
)
