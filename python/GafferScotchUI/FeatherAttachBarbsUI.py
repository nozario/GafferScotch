import Gaffer
import GafferUI
import imath

import GafferScotch

from ._IconPath import ICON_PATH

# For now, we'll use the default UI generation based on plug metadata.
# If custom UI elements or layouts are needed, a specific creation function
# would be provided here.
GafferUI.NodeUI.registerNodeUI(GafferScotch.FeatherAttachBarbs, None)

Gaffer.Metadata.registerNode(
    GafferScotch.FeatherAttachBarbs,
    "description",
    """
    Attaches barb geometry to shaft curves to create complete feather structures.
    Computes and stores the necessary binding data for later deformation by
    the FeatherDeformBarbs node.
    """,
    "icon",
    ICON_PATH,
    "nodeGadget:color",
    imath.Color3f(0.42, 0.27, 0.23),
    # Section collapse states
    "layout:section:Input:collapsed",
    False,
    "layout:section:Attribute Names:collapsed",
    False,
    "layout:section:Binding Controls:collapsed",
    False,
    plugs={
        "in": [
            "description",
            """
            The input barb geometry (CurvesPrimitive) to attach to shafts.
            """,
            "nodule:type",
            "GafferUI::StandardNodule",
            "layout:section",
            "Input",
        ],
        "shafts": [
            "description",
            """
            The input shaft geometry (CurvesPrimitive) to attach to.
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
            Name of the primitive variable (int or string, Uniform) on shafts and barbs 
            that identifies matching hair/feather groups.
            """,
            "layout:section",
            "Attribute Names",
        ],
        "shaftPointIdAttrName": [
            "description",
            """
            Name of the primitive variable (int, Uniform) on barbs specifying 
            the target point index on the parent shaft curve.
            """,
            "layout:section",
            "Attribute Names",
        ],
        "barbParamAttrName": [
            "description",
            """
            Name of the vertex primitive variable (float, Vertex) on barbs (e.g., 'curveu') 
            used to identify the barb's root point (minimum value).
            """,
            "layout:section",
            "Attribute Names",
        ],
        "shaftUpVectorPrimVarName": [
            "description",
            """
            Optional name of the primitive variable (V3f, Uniform) on shafts 
            defining their up-vector for orientation calculation.
            """,
            "layout:section",
            "Attribute Names",
        ],
        "shaftPointOrientAttrName": [
            "description",
            """
            Optional name of the primitive variable (Quaternionf, Uniform or Vertex) 
            on shafts defining their explicit orientation.
            """,
            "layout:section",
            "Attribute Names",
        ],
        "defaultUpVector": [
            "description",
            """
            Default up-vector to use for shaft orientation if no up-vector primvar 
            or explicit orientation is found.
            """,
            "layout:section",
            "Binding Controls",
        ],
    },
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
