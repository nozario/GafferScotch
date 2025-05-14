import Gaffer
import imath

import GafferScotch

from ._IconPath import ICON_PATH

Gaffer.Metadata.registerNode(
    GafferScotch.FeatherDeformBarbs,
    "description",
    """
    Deforms barb geometry based on the motion of animated shaft curves.
    Uses binding data computed by the FeatherAttachBarbs node to create
    natural-looking feather deformation.
    """,
    "icon",
    ICON_PATH,
    "nodeGadget:color",
    imath.Color3f(0.42, 0.27, 0.23),
    # Section collapse states
    "layout:section:Input:collapsed",
    False,
    "layout:section:Shaft Attributes:collapsed",
    False,
    "layout:section:Output Controls:collapsed",
    False,
    plugs={
        "in": [
            "description",
            """
            The input barb geometry (CurvesPrimitive with bind attributes) to be deformed.
            This should contain binding data created by the FeatherAttachBarbs node.
            """,
            "nodule:type",
            "GafferUI::StandardNodule",
            "layout:section",
            "Input",
        ],
        "animatedShafts": [
            "description",
            """
            The input animated shaft geometry (CurvesPrimitive) that will drive the barb deformation.
            This is the deformed/animated version of the shafts.
            """,
            "nodule:type",
            "GafferUI::StandardNodule",
            "plugValueWidget:type",
            "GafferUI.ScenePlugValueWidget",
            "layout:section",
            "Input",
        ],
        "restShafts": [
            "description",
            """
            The input rest shaft geometry (CurvesPrimitive) in their bind pose.
            This should match the shafts used in FeatherAttachBarbs.
            """,
            "nodule:type",
            "GafferUI::StandardNodule",
            "plugValueWidget:type",
            "GafferUI.ScenePlugValueWidget",
            "layout:section",
            "Input",
        ],
        "hairIdAttrName": [
            "description",
            """
            Name of the primitive variable (int or string, Uniform) on shafts that
            identifies matching hair/feather groups. Must match the attribute on barbs.
            """,
            "layout:section",
            "Shaft Attributes",
        ],
        "shaftPointIdAttrName": [
            "description",
            """
            Name of the primitive variable (int, Uniform) on barbs that
            identifies which point along the shaft curve each barb is attached to.
            """,
            "layout:section",
            "Shaft Attributes",
        ],
        "barbParamAttrName": [
            "description",
            """
            Name of the primitive variable (float, Uniform) on barbs that
            stores the parametric position of each barb along the shaft curve segment.
            """,
            "layout:section",
            "Shaft Attributes",
        ],
        "shaftUpVectorPrimVarName": [
            "description",
            """
            Optional name of the primitive variable (V3f, Uniform) on shafts
            defining their up-vector for orientation calculation.
            """,
            "layout:section",
            "Shaft Attributes",
        ],
        "shaftPointOrientAttrName": [
            "description",
            """
            Optional name of the primitive variable (Quaternionf, Uniform or Vertex) on shafts
            defining their explicit orientation.
            """,
            "layout:section",
            "Shaft Attributes",
        ],
        "cleanupBindAttributes": [
            "description",
            """
            If true, the binding attributes created by FeatherAttachBarbs will be
            removed from the output barb geometry.
            """,
            "layout:section",
            "Output Controls",
            "plugValueWidget:type",
            "GafferUI.BoolPlugValueWidget",
            "boolPlugValueWidget:displayMode",
            "checkBox",
        ],
    },
)

# Example of how you might add metadata for plugs later:
# GafferUI.PlugValueWidget.registerCreator(
# GafferScotch.FeatherDeformBarbs.staticTypeId(),
# "animatedShafts", # Example plug name
# lambda plug : GafferUI.StandardPlugValueWidget( plug, GafferUI.ColorSwatchPlugValueWidget ), # Example widget
# )
