import os
import sys

__import__("IECore")
__import__("Gaffer")
__import__("GafferScene")

try:
    # Import all symbols from our C++ extension module
    from .GafferScotchModule import (
        CaptureMatrixWeights,
        CaptureWeights,
        CurvesPostProcess,
        MatrixDeform,
        PointDeform,
        RigidAttachCurves,
        RigidDeformCurves,
    )

except ImportError as e:
    print(f"Error importing GafferScotch: {e}")
    print(f"Python path: {sys.path}")
    print(f"Current directory: {os.path.dirname(os.path.abspath(__file__))}")
    print(
        f"Directory contents: {os.listdir(os.path.dirname(os.path.abspath(__file__)))}"
    )
    raise
