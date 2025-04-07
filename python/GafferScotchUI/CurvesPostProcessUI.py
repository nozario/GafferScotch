##########################################################################
#
#  Copyright (c) 2023, Your Name. All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are
#  met:
#
#      * Redistributions of source code must retain the above
#        copyright notice, this list of conditions and the following
#        disclaimer.
#
#      * Redistributions in binary form must reproduce the above
#        copyright notice, this list of conditions and the following
#        disclaimer in the documentation and/or other materials provided with
#        the distribution.
#
#      * Neither the name of Your Name nor the names of
#        any other contributors to this software may be used to endorse or
#        promote products derived from this software without specific prior
#        written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
#  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
#  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
#  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
#  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
#  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
#  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
#  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
##########################################################################

import Gaffer
import IECore
import imath

import GafferScotch

from . import ICON_PATH

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
    "plugs",
    {
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
