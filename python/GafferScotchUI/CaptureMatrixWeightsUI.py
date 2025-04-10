import Gaffer
import IECore
import imath

import GafferScotch

from ._IconPath import ICON_PATH

Gaffer.Metadata.registerNode(
    GafferScotch.CaptureMatrixWeights,
    "description",
    """
    Computes influence weights and local transformation matrices for each point/vertex.
    
    Similar to CaptureWeights, but also computes local coordinate frames
    for each source point. These frames enable more natural deformations
    with the MatrixDeform node, especially when influence points are 
    outside the target mesh.

    Outputs both regular influence weights and transformation matrices.
    """,
    "icon",
    ICON_PATH,
    "nodeGadget:color",
    imath.Color3f(0.42, 0.27, 0.23),
    # Section collapse states
    "layout:section:Settings.General:collapsed",
    False,
    "layout:section:Settings.Search:collapsed",
    False,
    plugs={
        "staticDeformer": [
            "description",
            """
            The source points to compute influences from. These points will
            influence the points in the primary input based on proximity.
            Local transformation matrices will also be computed for these points.
            """,
            "nodule:type",
            "GafferUI::StandardNodule",
            "layout:section",
            "Settings.General",
        ],
        "deformerPath": [
            "description",
            """
            Path to the source/deformer object within the staticDeformer scene.
            """,
            "layout:section",
            "Settings.General",
        ],
        "pieceAttribute": [
            "description",
            """
            Optional attribute name for filtering points by piece/group.
            When specified, only points with matching attribute values
            will influence each other.
            """,
            "layout:section",
            "Settings.General",
        ],
        "radius": [
            "description",
            """
            The search radius for finding influencing points. Points outside
            this radius will not influence the target point. If fewer than
            minPoints are found, the radius is automatically increased.
            """,
            "layout:section",
            "Settings.Search",
        ],
        "minPoints": [
            "description",
            """
            The minimum number of source points required to influence each
            target point. If fewer points are found within the radius, the
            radius is automatically increased to include at least this many points.
            """,
            "layout:section",
            "Settings.Search",
        ],
        "maxPoints": [
            "description",
            """
            The maximum number of source points that can influence each
            target point. The closest points within the radius are kept.
            """,
            "layout:section",
            "Settings.Search",
        ],
        "neighborPoints": [
            "description",
            """
            The number of neighboring points to use when building local
            coordinate frames for each source point. More points generally
            create more stable frames but can be slower to compute.
            """,
            "layout:section",
            "Settings.Search",
        ],
    },
)
