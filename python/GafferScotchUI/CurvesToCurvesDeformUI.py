import Gaffer
import IECore
import imath

import GafferScotch

from ._IconPath import ICON_PATH

Gaffer.Metadata.registerNode(
    GafferScotch.CurvesToCurvesDeform,
    "description",
    """
    Deforms child curves based on binding data from CurvesToCurvesAttach and an animated parent (deformer) curve.
    It reads the stored rest frame and parent curve attachment point (index and U parameter), evaluates the
    equivalent frame on the animated parent curve, and transforms the child curve points accordingly.
    """,
    "icon",
    ICON_PATH,
    "nodeGadget:color",
    imath.Color3f(0.47, 0.27, 0.23),
    "layout:section:Settings.Parent Deformers:collapsed",
    False,
    "layout:section:Settings.Binding Path:collapsed",
    False,
    "layout:section:Settings.Up Vector:collapsed",
    False,
    "layout:section:Settings.Cleanup:collapsed",
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
            "The child curves (with cfx:* binding attributes) to be deformed.",
        ],
        "staticParentDeformer": [
            "description",
            """
            The parent (deformer) curves in their static/rest pose. This input is optional
            if the rest frame information is fully self-contained and trusted on the child curves'
            cfx:* attributes, but can be used for consistency or future enhancements.
            """,
            "nodule:type",
            "GafferUI::StandardNodule",
            "layout:section",
            "Settings.Parent Deformers",
        ],
        "animatedParentDeformer": [
            "description",
            """
            The parent (deformer) curves in their animated/current pose. The child curves will follow
            the deformation of this input.
            """,
            "nodule:type",
            "GafferUI::StandardNodule",
            "layout:section",
            "Settings.Parent Deformers",
        ],
        "useBindAttr": [
            "description",
            """
            Determines how the parent deformer curves (both static and animated) are specified.

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
            """
            The scene path to the parent deformer curves (used for both static and animated inputs)
            when 'Use Bind Attr' is disabled.
            """,
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
            The name of a string primitive variable on the child curves that specifies the scene path
            to its parent deformer curves when 'Use Bind Attr' is enabled.
            """,
            "layout:section",
            "Settings.Binding Path",
            "layout:visibilityActivator",
            "bindAttrVisible",
        ],
        "upVector": [
            "description",
            """
            An explicit up-vector used to help define a consistent orientation frame on the animated parent curve.
            Should match the method used in the attach node. Used if 'Use Up Vector Attr' is disabled.
            """,
            "layout:section",
            "Settings.Up Vector",
            "layout:visibilityActivator",
            "upVectorPlugVisible",
        ],
        "useUpVectorAttr": [
            "description",
            """
            Determines how the up-vector for frame orientation on the animated parent is specified.

            - Plug Value: Use the 'Up Vector' plug's value.
            - Attribute: Use a V3f attribute on the child curve (typically uniform, set by attach).
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
            The name of a V3f primitive variable on the child curves to be used as the up-vector for 
            orienting the frame on the animated parent curve. Used if 'Use Up Vector Attr' is enabled.
            """,
            "layout:section",
            "Settings.Up Vector",
            "layout:visibilityActivator",
            "upVectorAttrVisible",
        ],
        "cleanupBindAttributes": [
            "description",
            """
            When enabled, removes all cfx:* binding attributes from the output child curves after deformation.
            Also removes the bind path attribute if one was used.
            """,
            "layout:section",
            "Settings.Cleanup",
        ],
    },
)
