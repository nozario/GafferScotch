import Gaffer
import imath

import GafferScotch

from ._IconPath import ICON_PATH

Gaffer.Metadata.registerNode(
    GafferScotch.FeatherAttachBarbs,
    "description",
    """
    Attaches barb geometry to shaft curves to create complete feather structures.
    Computes and stores the necessary binding data for later deformation by
    the FeatherDeformBarbs node.
    
    For each barb, the node:
    1. Finds the matching shaft curve using the hair ID attribute
    2. Identifies the barb's root point using the parametric curve value
    3. Finds the closest point on the matching shaft curve
    4. Computes the local reference frame for rigid deformation
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
        "inShafts": [
            "description",
            """
            The input shaft geometry (CurvesPrimitive) to attach barbs to.
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
        "curveParamAttrName": [
            "description",
            """
            Name of the vertex primitive variable (float, Vertex) on barbs that represents
            the parametric position along the curve (e.g., 'curveu'). The point with the
            minimum value is considered to be the barb's root point.
            """,
            "layout:section",
            "Attribute Names",
        ],
        "shaftPointOrientAttrName": [
            "description",
            """
            Name of the primitive variable (Quaternionf, Uniform or Vertex)
            on shafts defining their explicit orientation. This is required
            for computing the correct normal and tangent frame.
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
