import Gaffer
import GafferUI

import GafferScotch

# For now, we'll use the default UI generation based on plug metadata.
# If custom UI elements or layouts are needed, a specific creation function
# would be provided here.
GafferUI.NodeUI.registerNodeUI(GafferScotch.FeatherDeformBarbs, None)

# Metadata for Plugs

# --- Input Sections ---
Gaffer.Metadata.registerValue(
    GafferScotch.FeatherDeformBarbs,
    "inPlug",
    "description",
    "The input barb geometry (CurvesPrimitive with bind attributes) to be deformed.",
)

Gaffer.Metadata.registerValue(
    GafferScotch.FeatherDeformBarbs,
    "animatedShaftsPlug",
    "description",
    "The input animated shaft geometry (CurvesPrimitive) that will drive the barb deformation.",
)
Gaffer.Metadata.registerValue(
    GafferScotch.FeatherDeformBarbs,
    "animatedShaftsPlug",
    "plugValueWidget:type",
    "GafferUI.ScenePlugValueWidget",
)

# --- Attribute Names Section ---
Gaffer.Metadata.registerValue(
    GafferScotch.FeatherDeformBarbs,
    "hairIdAttrNamePlug",
    "layout:section",
    "Animated Shaft Attributes",
)
Gaffer.Metadata.registerValue(
    GafferScotch.FeatherDeformBarbs,
    "hairIdAttrNamePlug",
    "description",
    "Name of the primitive variable (int or string, Uniform) on animated shafts that identifies matching hair/feather groups (must match 'bind_shaftHairId' on barbs).",
)

Gaffer.Metadata.registerValue(
    GafferScotch.FeatherDeformBarbs,
    "shaftUpVectorPrimVarNamePlug",
    "layout:section",
    "Animated Shaft Attributes",
)
Gaffer.Metadata.registerValue(
    GafferScotch.FeatherDeformBarbs,
    "shaftUpVectorPrimVarNamePlug",
    "description",
    "Optional name of the primitive variable (V3f, Uniform) on animated shafts defining their up-vector for orientation calculation.",
)

Gaffer.Metadata.registerValue(
    GafferScotch.FeatherDeformBarbs,
    "shaftPointOrientAttrNamePlug",
    "layout:section",
    "Animated Shaft Attributes",
)
Gaffer.Metadata.registerValue(
    GafferScotch.FeatherDeformBarbs,
    "shaftPointOrientAttrNamePlug",
    "description",
    "Optional name of the primitive variable (Quaternionf, Uniform or Vertex) on animated shafts defining their explicit orientation.",
)

# --- Output Controls Section ---
Gaffer.Metadata.registerValue(
    GafferScotch.FeatherDeformBarbs,
    "cleanupBindAttributesPlug",
    "layout:section",
    "Output Controls",
)
Gaffer.Metadata.registerValue(
    GafferScotch.FeatherDeformBarbs,
    "cleanupBindAttributesPlug",
    "description",
    "If true, the 'bind_*' attributes created by FeatherAttachBarbs will be removed from the output barb geometry.",
)
Gaffer.Metadata.registerValue(
    GafferScotch.FeatherDeformBarbs,
    "cleanupBindAttributesPlug",
    "plugValueWidget:type",
    "GafferUI.BoolPlugValueWidget",
)
Gaffer.Metadata.registerValue(
    GafferScotch.FeatherDeformBarbs,
    "cleanupBindAttributesPlug",
    "boolPlugValueWidget:displayMode",
    "checkBox",
)

# Example of how you might add metadata for plugs later:
# GafferUI.PlugValueWidget.registerCreator(
# GafferScotch.FeatherDeformBarbs.staticTypeId(),
# "animatedShafts", # Example plug name
# lambda plug : GafferUI.StandardPlugValueWidget( plug, GafferUI.ColorSwatchPlugValueWidget ), # Example widget
# )
