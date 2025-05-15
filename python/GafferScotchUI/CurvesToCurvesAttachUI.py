import Gaffer
import IECore
import imath

import GafferScotch

from ._IconPath import ICON_PATH

Gaffer.Metadata.registerNode(
    GafferScotch.CurvesToCurvesAttach,
    "description",
    """
    Computes and stores binding data for attaching child curves to parent (deformer) curves.
    This node establishes an initial spatial relationship by finding the closest point on a parent
    curve to the root of each child curve, and calculates a rest frame (position, tangent, normal, bitangent)
    on the parent curve at that point. Binding data is stored as primitive variables on the output child curves.
    """,
    "icon",
    ICON_PATH,
    "nodeGadget:color",
    imath.Color3f(0.47, 0.27, 0.23),  # Adjusted color slightly
    "layout:section:Settings.Parent Deformer:collapsed",
    False,
    "layout:section:Settings.Child Curve Root:collapsed",
    False,
    "layout:section:Settings.Binding Path:collapsed",
    False,
    "layout:section:Settings.Up Vector:collapsed",
    False,
    "layout:activator:bindAttrVisible",
    lambda plug: plug["useBindAttr"].getValue(),
    "layout:activator:deformerPathVisible",
    lambda plug: not plug["useBindAttr"].getValue(),
    "layout:activator:upVectorAttrVisible",
    lambda plug: plug["useUpVectorAttr"].getValue(),
    "layout:activator:upVectorPlugVisible",
    lambda plug: not plug["useUpVectorAttr"].getValue(),
    plugs={
        "in": [
            "description",
            "The child curves to be attached.",
        ],
        "parentDeformer": [
            "description",
            """
            The parent (deformer) curves that the child curves will be attached to.
            This defines the target geometry for binding.
            """,
            "nodule:type",
            "GafferUI::StandardNodule",
            "layout:section",
            "Settings.Parent Deformer",
        ],
        "curveRootAttr": [
            "description",
            """
            The name of a vertex float or int attribute on the child curves that defines the root point
            of each curve. The vertex with the highest positive value is considered the root.
            If empty or no positive value found, the first point of each curve is used.
            """,
            "layout:section",
            "Settings.Child Curve Root",
        ],
        "useBindAttr": [
            "description",
            """
            Determines how the parent deformer curves are specified.

            - Direct Path: Use a single scene path for all child curves.
            - Attribute: Use a string attribute on each child curve to specify its parent deformer path.
            """,
            "plugValueWidget:type",
            "GafferUI.PresetsPlugValueWidget",
            "preset:Direct Path",
            False,
            "preset:Attribute",
            True,
            "layout:section",
            "Settings.Binding Path",
        ],
        "deformerPath": [
            "description",
            "The scene path to the parent deformer curves when 'Use Bind Attr' is disabled.",
            "plugValueWidget:type",
            "GafferUI.StringPlugValueWidget",
            "layout:section",
            "Settings.Binding Path",
            "layout:visibilityActivator",
            "deformerPathVisible",
        ],
        "bindAttr": [
            "description",
            """
            The name of a string primitive variable (Uniform or Vertex) on the child curves that 
            specifies the scene path to its parent deformer curves when 'Use Bind Attr' is enabled.
            If Vertex, the value from the first vertex of the curve is used.
            """,
            "layout:section",
            "Settings.Binding Path",
            "layout:visibilityActivator",
            "bindAttrVisible",
        ],
        "upVector": [
            "description",
            """
            An explicit up-vector used to help define a consistent orientation frame on the parent curve.
            Used if 'Use Up Vector Attr' is disabled.
            """,
            "layout:section",
            "Settings.Up Vector",
            "layout:visibilityActivator",
            "upVectorPlugVisible",
        ],
        "useUpVectorAttr": [
            "description",
            """
            Determines how the up-vector for frame orientation is specified.

            - Plug Value: Use the 'Up Vector' plug's value for all curves.
            - Attribute: Use a V3f attribute on each child curve as its up-vector.
            """,
            "plugValueWidget:type",
            "GafferUI.PresetsPlugValueWidget",
            "preset:Plug Value",
            False,
            "preset:Attribute",
            True,
            "layout:section",
            "Settings.Up Vector",
        ],
        "upVectorAttr": [
            "description",
            """
            The name of a V3f primitive variable (Uniform or Vertex/Varying) on the child curves to be used as 
            the up-vector for its attachment frame. If Vertex/Varying, the value from the root point is used.
            Used if 'Use Up Vector Attr' is enabled.
            """,
            "layout:section",
            "Settings.Up Vector",
            "layout:visibilityActivator",
            "upVectorAttrVisible",
        ],
    },
)
