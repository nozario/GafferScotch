#ifndef GAFFERSCOTCH_ATTACHCURVESDATASTRUCTURES_H
#define GAFFERSCOTCH_ATTACHCURVESDATASTRUCTURES_H

#include "GafferScotch/Export.h"

#include "IECore/MurmurHash.h"
#include "IECore/VectorTypedData.h"

#include "IECoreScene/CurvesPrimitive.h"
#include "IECoreScene/MeshPrimitive.h"
#include "IECoreScene/PrimitiveVariable.h"

#include "Imath/ImathMatrix.h"
#include "Imath/ImathVec.h"

namespace GafferScotch
{

/// Utility functions for the AttachCurves node
namespace AttachCurvesDataStructures
{

/// Create a transformation matrix from position, normal, tangent, and bitangent vectors
GAFFERSCOTCH_API Imath::M44f createTransformMatrix(
    const Imath::V3f &position,
    const Imath::V3f &normal,
    const Imath::V3f &tangentU,
    const Imath::V3f &tangentV
);

/// Apply a transformation matrix to a curve
GAFFERSCOTCH_API void transformCurve(
    const Imath::M44f &transformation,
    const std::vector<Imath::V3f> &inputPositions,
    std::vector<Imath::V3f> &outputPositions,
    int startIndex,
    int numVerts
);

/// Find the closest point on a mesh to a given point and return its UV coordinates
GAFFERSCOTCH_API bool findClosestPointUV(
    const IECoreScene::MeshPrimitive *mesh,
    const Imath::V3f &point,
    Imath::V2f &uv
);

/// Find a point on a mesh with given UV coordinates
GAFFERSCOTCH_API bool findPointAtUV(
    const IECoreScene::MeshPrimitive *mesh,
    const Imath::V2f &uv,
    Imath::V3f &point,
    Imath::V3f &normal,
    Imath::V3f &tangentU,
    Imath::V3f &tangentV
);

/// Compute vertex offsets for a curves primitive
GAFFERSCOTCH_API IECore::IntVectorDataPtr computeVertexOffsets(
    const IECoreScene::CurvesPrimitive *curves
);

/// Ensure vectors form an orthonormal basis
GAFFERSCOTCH_API void orthonormalizeBasis(
    Imath::V3f &normal,
    Imath::V3f &tangentU,
    Imath::V3f &tangentV
);

} // namespace AttachCurvesDataStructures

} // namespace GafferScotch

#endif // GAFFERSCOTCH_ATTACHCURVESDATASTRUCTURES_H
