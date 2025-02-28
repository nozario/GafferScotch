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
    Takes both static and animated meshes as input and uses a rigid frame-based
    approach to maintain curve shape during deformation.
    """,
    "icon",
    ICON_PATH,
    "nodeGadget:color",
    imath.Color3f(0.42, 0.27, 0.23),
    # Section collapse states
    "layout:section:Settings.Deformer:collapsed",
    False,
    "layout:section:Settings.Binding:collapsed",
    False,
    "layout:section:Settings.Cleanup:collapsed",
    False,
    # Activators for conditional visibility
    "layout:activator:bindAttrVisible",
    lambda plug: plug["useBindAttr"].getValue(),
    "layout:activator:deformerPathVisible",
    lambda plug: not plug["useBindAttr"].getValue(),
    plugs={
        "staticDeformer": [
            "description",
            """
            The static mesh that was used for binding. This is used as a reference
            to ensure proper deformation.
            """,
            "nodule:type",
            "GafferUI::StandardNodule",
            "layout:section",
            "Settings.Deformer",
        ],
        "animatedDeformer": [
            "description",
            """
            The animated version of the mesh. The curves will follow the
            deformation of this mesh.
            """,
            "nodule:type",
            "GafferUI::StandardNodule",
            "layout:section",
            "Settings.Deformer",
        ],
        "useBindAttr": [
            "description",
            """
            Determines how the target mesh is specified for deformation.
            
            Direct Path : Use a single mesh path for all curves.
            Attribute : Use a string attribute on each curve to specify its target mesh.
            """,
            "plugValueWidget:type",
            "GafferUI.PresetsPlugValueWidget",
            "preset:Direct Path",
            False,
            "preset:Attribute",
            True,
            "layout:section",
            "Settings.Binding",
        ],
        "deformerPath": [
            "description",
            """
            The path to the mesh to use when useBindAttr is disabled.
            """,
            "plugValueWidget:type",
            "GafferUI.PathPlugValueWidget",
            "layout:section",
            "Settings.Binding",
            "layout:visibilityActivator",
            "deformerPathVisible",
        ],
        "bindAttr": [
            "description",
            """
            The name of the string attribute that specifies which mesh to
            use when useBindAttr is enabled.
            """,
            "layout:section",
            "Settings.Binding",
            "layout:visibilityActivator",
            "bindAttrVisible",
        ],
        "cleanupBindAttributes": [
            "description",
            """
            When enabled, removes all binding attributes from the output geometry.
            This includes restPosition, restNormal, restTangent, restBitangent,
            triangleIndex, barycentricCoords, uvCoords, rootPoint, and any custom
            bind path attribute.
            """,
            "layout:section",
            "Settings.Cleanup",
        ],
    },
)
