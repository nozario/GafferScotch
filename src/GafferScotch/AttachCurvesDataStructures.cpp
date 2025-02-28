#include "GafferScotch/AttachCurvesDataStructures.h"

#include "IECoreScene/MeshAlgo.h"
#include "IECoreScene/MeshPrimitiveEvaluator.h"
#include "IECoreScene/PrimitiveVariable.h"

using namespace Imath;
using namespace IECore;
using namespace IECoreScene;
using namespace GafferScotch;

namespace GafferScotch
{

namespace AttachCurvesDataStructures
{

M44f createTransformMatrix(
    const V3f &position,
    const V3f &normal,
    const V3f &tangentU,
    const V3f &tangentV
)
{
    M44f matrix;
    matrix.makeIdentity();
    
    // Set the rotation part of the matrix (3x3 upper-left)
    matrix[0][0] = tangentU.x;
    matrix[0][1] = tangentU.y;
    matrix[0][2] = tangentU.z;
    
    matrix[1][0] = tangentV.x;
    matrix[1][1] = tangentV.y;
    matrix[1][2] = tangentV.z;
    
    matrix[2][0] = normal.x;
    matrix[2][1] = normal.y;
    matrix[2][2] = normal.z;
    
    // Set the translation part of the matrix (bottom row)
    matrix[3][0] = position.x;
    matrix[3][1] = position.y;
    matrix[3][2] = position.z;
    
    return matrix;
}

void transformCurve(
    const M44f &transformation,
    const std::vector<V3f> &inputPositions,
    std::vector<V3f> &outputPositions,
    int startIndex,
    int numVerts
)
{
    for( int i = 0; i < numVerts; ++i )
    {
        V3f point = inputPositions[startIndex + i];
        V3f transformedPoint = point * transformation;
        outputPositions[startIndex + i] = transformedPoint;
    }
}

bool findClosestPointUV(
    const MeshPrimitive *mesh,
    const V3f &point,
    V2f &uv
)
{
    // Create a mesh evaluator
    MeshPrimitiveEvaluatorPtr evaluator = new MeshPrimitiveEvaluator( mesh );
    PrimitiveEvaluator::ResultPtr result = evaluator->createResult();
    
    // Find the closest point on the mesh
    if( !evaluator->closestPoint( point, result.get() ) )
    {
        return false;
    }
    
    // Get the UV coordinates at this point
    uv = result->uv();
    
    return true;
}

bool findPointAtUV(
    const MeshPrimitive *mesh,
    const V2f &uv,
    V3f &point,
    V3f &normal,
    V3f &tangentU,
    V3f &tangentV
)
{
    // Check if the mesh has the necessary UV coordinates
    if( !mesh->variables.count( "uv" ) )
    {
        return false;
    }
    
    // Create a mesh evaluator
    MeshPrimitiveEvaluatorPtr evaluator = new MeshPrimitiveEvaluator( mesh );
    PrimitiveEvaluator::ResultPtr result = evaluator->createResult();
    
    // Find the point on the mesh with the given UV coordinates
    if( !evaluator->pointAtUV( uv, result.get() ) )
    {
        return false;
    }
    
    // Get the position at this point
    point = result->point();
    
    // Calculate normals if they don't exist
    PrimitiveVariable normals;
    if( !mesh->variables.count( "N" ) )
    {
        normals = MeshAlgo::calculateVertexNormals( mesh, MeshAlgo::NormalWeighting::Angle );
    }
    else
    {
        normals = mesh->variables.at( "N" );
    }
    
    // Calculate tangents
    std::pair<PrimitiveVariable, PrimitiveVariable> tangents = MeshAlgo::calculateTangentsFromUV( mesh, "uv", "P", true, false );
    PrimitiveVariable tangentUVar = tangents.first;
    PrimitiveVariable tangentVVar = tangents.second;
    
    // Get the normal and tangents at this point
    normal = result->vectorPrimVar( normals );
    tangentU = result->vectorPrimVar( tangentUVar );
    tangentV = result->vectorPrimVar( tangentVVar );
    
    // Normalize the vectors
    normal.normalize();
    tangentU.normalize();
    tangentV.normalize();
    
    // Ensure the vectors form an orthonormal basis
    orthonormalizeBasis( normal, tangentU, tangentV );
    
    return true;
}

IntVectorDataPtr computeVertexOffsets(
    const CurvesPrimitive *curves
)
{
    const IntVectorData *vertsPerCurve = curves->verticesPerCurve();
    const std::vector<int> &vertsPerCurveData = vertsPerCurve->readable();
    
    IntVectorDataPtr offsetsData = new IntVectorData();
    std::vector<int> &offsets = offsetsData->writable();
    offsets.reserve( vertsPerCurveData.size() );
    
    int offset = 0;
    for( size_t i = 0; i < vertsPerCurveData.size(); ++i )
    {
        offsets.push_back( offset );
        offset += vertsPerCurveData[i];
    }
    
    return offsetsData;
}

void orthonormalizeBasis(
    V3f &normal,
    V3f &tangentU,
    V3f &tangentV
)
{
    // Ensure the normal is normalized
    normal.normalize();
    
    // Ensure the tangent vectors are orthogonal to the normal
    tangentU = tangentU - normal * tangentU.dot( normal );
    tangentU.normalize();
    
    tangentV = tangentV - normal * tangentV.dot( normal );
    tangentV.normalize();
    
    // Ensure the tangent vectors are orthogonal to each other
    tangentV = tangentV - tangentU * tangentV.dot( tangentU );
    tangentV.normalize();
    
    // Ensure the tangent vectors form a right-handed coordinate system
    if( tangentU.cross( tangentV ).dot( normal ) < 0 )
    {
        tangentV = -tangentV;
    }
}

} // namespace AttachCurvesDataStructures

} // namespace GafferScotch
