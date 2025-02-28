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
    # Section collapse states
    "layout:section:Settings.Deformer:collapsed",
    False,
    "layout:section:Settings.Binding:collapsed",
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
            The static mesh to bind the curves to. This mesh defines the initial
            binding positions and frames.
            """,
            "nodule:type",
            "GafferUI::StandardNodule",
            "layout:section",
            "Settings.Deformer",
        ],
        "curveRootAttr": [
            "description",
            """
            The name of a vertex float attribute that defines the root point
            of each curve. Values should be between 0 and 1, with 1 indicating
            the root point. If empty, the first point of each curve is used.
            """,
            "layout:section",
            "Settings.Binding",
        ],
        "useBindAttr": [
            "description",
            """
            Determines how the target mesh is specified for binding.
            
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
            The path to the mesh to bind to when useBindAttr is disabled.
            """,
            "plugValueWidget:type",
            "GafferUI.PathPlugValueWidget",
            "path:valid",
            True,
            "path:leaf",
            True,
            "path:bookmarks",
            "meshes",
            "layout:section",
            "Settings.Binding",
            "layout:visibilityActivator",
            "deformerPathVisible",
        ],
        "bindAttr": [
            "description",
            """
            The name of the string attribute that specifies which mesh to
            bind to when useBindAttr is enabled.
            """,
            "layout:section",
            "Settings.Binding",
            "layout:visibilityActivator",
            "bindAttrVisible",
        ],
    },
)
