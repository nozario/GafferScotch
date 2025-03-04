import IECore
import imath

import Gaffer
import GafferScotch
from . import ICON_PATH

Gaffer.Metadata.registerNode(
    GafferScotch.CaptureWeights,
    "description",
    """
    Computes influence weights for each point/vertex based on proximity to source points.
    For each target point, finds the closest source points within a specified radius and
    computes weights based on distance. The weights use a quadratic falloff function.
    
    The node outputs two array primitive variables:
    - captureIndices: Array of influencing point indices
    - captureWeights: Array of corresponding weights
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
        "source": [
            "description",
            """
            The source points to compute influences from. These points will
            influence the points in the primary input based on proximity.
            """,
            "nodule:type",
            "GafferUI::StandardNodule",
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
    },
)
