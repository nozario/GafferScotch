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
    "layout:section:Animated Shaft Attributes:collapsed",
    False,
    "layout:section:Output Controls:collapsed",
    False,
    plugs={
        "in": [
            "description",
            """
            The input barb geometry (CurvesPrimitive with bind attributes) to be deformed.
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
            Name of the primitive variable (int or string, Uniform) on animated shafts that
            identifies matching hair/feather groups (must match 'bind_shaftHairId' on barbs).
            """,
            "layout:section",
            "Animated Shaft Attributes",
        ],
        "shaftUpVectorPrimVarName": [
            "description",
            """
            Optional name of the primitive variable (V3f, Uniform) on animated shafts
            defining their up-vector for orientation calculation.
            """,
            "layout:section",
            "Animated Shaft Attributes",
        ],
        "shaftPointOrientAttrName": [
            "description",
            """
            Optional name of the primitive variable (Quaternionf, Uniform or Vertex) on animated
            shafts defining their explicit orientation.
            """,
            "layout:section",
            "Animated Shaft Attributes",
        ],
        "cleanupBindAttributes": [
            "description",
            """
            If true, the 'bind_*' attributes created by FeatherAttachBarbs will be
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
