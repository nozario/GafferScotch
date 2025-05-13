import Gaffer
import GafferUI

import GafferScotch

# For now, we'll use the default UI generation based on plug metadata.
# If custom UI elements or layouts are needed, a specific creation function
# would be provided here.
GafferUI.NodeUI.registerNodeUI(GafferScotch.FeatherAttachBarbs, None)

# Metadata for Plugs

# --- Input Sections ---
Gaffer.Metadata.registerValue(
    GafferScotch.FeatherAttachBarbs,
    "inPlug",
    "description",
    "The input barb geometry (CurvesPrimitive) to attach to shafts.",
)

Gaffer.Metadata.registerValue(
    GafferScotch.FeatherAttachBarbs,
    "shaftsPlug",
    "description",
    "The input shaft geometry (CurvesPrimitive) to attach to.",
)
Gaffer.Metadata.registerValue(
    GafferScotch.FeatherAttachBarbs,
    "shaftsPlug",
    "plugValueWidget:type",
    "GafferUI.ScenePlugValueWidget",
)

# --- Attribute Names Section ---
Gaffer.Metadata.registerValue(
    GafferScotch.FeatherAttachBarbs,
    "hairIdAttrNamePlug",
    "layout:section",
    "Attribute Names",
)
Gaffer.Metadata.registerValue(
    GafferScotch.FeatherAttachBarbs,
    "hairIdAttrNamePlug",
    "description",
    "Name of the primitive variable (int or string, Uniform) on shafts and barbs that identifies matching hair/feather groups.",
)

Gaffer.Metadata.registerValue(
    GafferScotch.FeatherAttachBarbs,
    "shaftPointIdAttrNamePlug",
    "layout:section",
    "Attribute Names",
)
Gaffer.Metadata.registerValue(
    GafferScotch.FeatherAttachBarbs,
    "shaftPointIdAttrNamePlug",
    "description",
    "Name of the primitive variable (int, Uniform) on barbs specifying the target point index on the parent shaft curve.",
)

Gaffer.Metadata.registerValue(
    GafferScotch.FeatherAttachBarbs,
    "barbParamAttrNamePlug",
    "layout:section",
    "Attribute Names",
)
Gaffer.Metadata.registerValue(
    GafferScotch.FeatherAttachBarbs,
    "barbParamAttrNamePlug",
    "description",
    "Name of the vertex primitive variable (float, Vertex) on barbs (e.g., 'curveu') used to identify the barb's root point (minimum value).",
)

Gaffer.Metadata.registerValue(
    GafferScotch.FeatherAttachBarbs,
    "shaftUpVectorPrimVarNamePlug",
    "layout:section",
    "Attribute Names",
)
Gaffer.Metadata.registerValue(
    GafferScotch.FeatherAttachBarbs,
    "shaftUpVectorPrimVarNamePlug",
    "description",
    "Optional name of the primitive variable (V3f, Uniform) on shafts defining their up-vector for orientation calculation.",
)

Gaffer.Metadata.registerValue(
    GafferScotch.FeatherAttachBarbs,
    "shaftPointOrientAttrNamePlug",
    "layout:section",
    "Attribute Names",
)
Gaffer.Metadata.registerValue(
    GafferScotch.FeatherAttachBarbs,
    "shaftPointOrientAttrNamePlug",
    "description",
    "Optional name of the primitive variable (Quaternionf, Uniform or Vertex) on shafts defining their explicit orientation.",
)

# --- Binding Controls Section ---
Gaffer.Metadata.registerValue(
    GafferScotch.FeatherAttachBarbs,
    "defaultUpVectorPlug",
    "layout:section",
    "Binding Controls",
)
Gaffer.Metadata.registerValue(
    GafferScotch.FeatherAttachBarbs,
    "defaultUpVectorPlug",
    "description",
    "Default up-vector to use for shaft orientation if no up-vector primvar or explicit orientation is found.",
)

# Example of how you might add metadata for plugs if needed:
# GafferUI.PlugValueWidget.registerCreator(
# GafferScotch.FeatherAttachBarbs.staticTypeId(),
# "somePlugName",
# lambda plug : GafferUI.StandardPlugValueWidget( plug, GafferUI.LabelPlugValueWidget ),
# )

# Or using plug metadata directly in the node's constructor (C++):
# Gaffer::MetadataAlgo::setPlugValue( somePlug, "plugValueWidget:type", "GafferUI.PresetsPlugValueWidget" );
# Gaffer::MetadataAlgo::setPlugValue( somePlug, "description", "This is a helpful description." );
