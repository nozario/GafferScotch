import Gaffer
import IECore
import imath

import GafferScotch

from ._IconPath import ICON_PATH

Gaffer.Metadata.registerNode(
    GafferScotch.CurvesPostProcess,
    "description",
    """
    A node for post-processing curves after deformation.
    Provides various techniques for refining and improving deformed curves.
    """,
    "nodeGadget:color",
    imath.Color3f(0.42, 0.27, 0.23),
    "icon",
    ICON_PATH,
    # Taubin Smoothing
    "layout:section:Taubin Smoothing:summary",
    "Options for applying Taubin smoothing to curves",
    plugs={
        "enableTaubinSmoothing": [
            "description",
            """
            Enables Taubin smoothing. This is a non-shrinking smoothing algorithm that helps
            reduce high-frequency noise while preserving the overall shape of the curves.
            """,
            "layout:section",
            "Taubin Smoothing",
            "nodule:type",
            "",
            "plugValueWidget:type",
            "GafferUI.BoolPlugValueWidget",
        ],
        "enableEndPointsFix": [
            "description",
            """
            Enables end points fix. This post-processing step realigns end points of curves
            that deviate significantly from the median distance between points, helping to
            maintain the curve's shape.
            """,
            "layout:section",
            "End Points Fix",
            "nodule:type",
            "",
            "plugValueWidget:type",
            "GafferUI.BoolPlugValueWidget",
        ],
        "lambda": [
            "description",
            """
            Positive smoothing weight for the first pass of Taubin smoothing. Larger values
            result in more aggressive smoothing. Typical values range from 0.0 to 1.0.
            """,
            "layout:section",
            "Taubin Smoothing",
            "nodule:type",
            "",
            "plugValueWidget:type",
            "GafferUI.NumericPlugValueWidget",
        ],
        "mu": [
            "description",
            """
            Negative smoothing weight for the second pass of Taubin smoothing.
            This helps prevent shrinkage. This value is typically negative, with
            common values around -0.53 (slightly larger magnitude than lambda).
            """,
            "layout:section",
            "Taubin Smoothing",
            "nodule:type",
            "",
            "plugValueWidget:type",
            "GafferUI.NumericPlugValueWidget",
        ],
        "iterations": [
            "description",
            """
            Number of smoothing iterations to perform. More iterations result in
            smoother curves, but may lose desirable detail. Typically 5-15 iterations
            are sufficient.
            """,
            "layout:section",
            "Taubin Smoothing",
            "nodule:type",
            "",
            "plugValueWidget:type",
            "GafferUI.NumericPlugValueWidget",
        ],
    },
)
