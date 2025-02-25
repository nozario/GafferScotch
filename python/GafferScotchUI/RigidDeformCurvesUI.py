import IECore
import imath

import Gaffer
import GafferScotch
from . import ICON_PATH

Gaffer.Metadata.registerNode(
    GafferScotch.RigidDeformCurves,
    "description",
    """
    Deforms curves using binding data computed by the RigidAttachCurves node.
    Takes both rest and animated meshes as input and uses a rigid frame-based
    approach to maintain curve shape during deformation.
    """,
    "icon",
    ICON_PATH,
    "nodeGadget:color",
    imath.Color3f(0.42, 0.27, 0.23),
    plugs={
        "restMesh": [
            "description",
            """
            The rest mesh that was used for binding. This is used as a reference
            to ensure proper deformation.
            """,
            "nodule:type",
            "GafferUI::StandardNodule",
        ],
        "animatedMesh": [
            "description",
            """
            The animated version of the mesh. The curves will follow the
            deformation of this mesh.
            """,
            "nodule:type",
            "GafferUI::StandardNodule",
        ],
        "useBindAttr": [
            "description",
            """
            When enabled, uses a string attribute on the curves to determine
            which mesh to deform with. When disabled, uses a single mesh path
            for all curves.
            """,
        ],
        "bindPath": [
            "description",
            """
            The path to the mesh to use when useBindAttr is disabled.
            """,
        ],
        "bindAttr": [
            "description",
            """
            The name of the string attribute that specifies which mesh to
            use when useBindAttr is enabled.
            """,
        ],
    },
)
