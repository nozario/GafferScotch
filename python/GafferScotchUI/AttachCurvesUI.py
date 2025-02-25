import Gaffer
import GafferUI
import IECore
import imath

import GafferScotch
from . import ICON_PATH

Gaffer.Metadata.registerNode(
    GafferScotch.AttachCurves,
    "description",
    """
    Attaches curves to a mesh, deforming them as the mesh animates.
    The curves can be attached either using a single target path for all curves,
    or using a per-curve attribute to specify different target paths.
    """,
    "icon",
    ICON_PATH,
    "nodeGadget:color",
    imath.Color3f(0.42, 0.27, 0.23),
    # Add activators for conditional visibility
    "layout:activator:useBindPath",
    lambda node: not node["useBindRootAttribute"].getValue(),
    "layout:activator:useBindRootAttr",
    lambda node: node["useBindRootAttribute"].getValue(),
    plugs={
        # Input plugs
        "curves": [
            "description",
            """
            The input curves to be attached to the mesh.
            """,
            "nodule:type",
            "GafferUI::StandardNodule",
        ],
        "restMesh": [
            "description",
            """
            The rest pose mesh that the curves will be bound to.
            """,
            "nodule:type",
            "GafferUI::StandardNodule",
        ],
        "animatedMesh": [
            "description",
            """
            The animated mesh that will drive the curve deformation.
            """,
            "nodule:type",
            "GafferUI::StandardNodule",
        ],
        # Binding mode
        "useBindRootAttribute": [
            "description",
            """
            Choose how to specify target paths for curve attachment.
            """,
            "plugValueWidget:type",
            "GafferUI.PresetsPlugValueWidget",
            "preset:Bind Path",
            False,
            "preset:Use bindRoot attribute",
            True,
        ],
        "rootPath": [
            "description",
            """
            The target path where the curves will be attached.
            Only used when binding mode is set to 'Bind Path'.
            """,
            "layout:activator",
            "useBindPath",
            "layout:section",
            "Settings",
        ],
        "bindRootAttribute": [
            "description",
            """
            The name of the uniform attribute containing target paths.
            Only used when binding mode is set to 'Use bindRoot attribute'.
            Should be a string attribute with one value per curve.
            """,
            "layout:activator",
            "useBindRootAttr",
            "layout:section",
            "Settings",
        ],
        # Root point finding - moved to bottom
        "rootAttributeName": [
            "description",
            """
            The name of the vertex attribute that marks root points on the curves.
            This should be an integer attribute where 1 indicates a root point.
            If not found, the first point of each curve is used as the root.
            """,
            "layout:section",
            "Settings",
            "layout:index",
            100,  # Force to bottom
            "default",
            "",  # Set empty default
        ],
    },
)
