#include <math.h>
#include <Eigen/Dense>
#include "GafferScotch/MatrixDeform.h"
#include "GafferScotch/ScenePathUtil.h"
#include "GafferScotch/nanoflann.hpp"

#include "IECoreScene/Primitive.h"
#include "IECoreScene/MeshPrimitive.h"
#include "IECoreScene/MeshPrimitiveEvaluator.h"
#include "IECoreScene/PrimitiveEvaluator.h"
#include "IECoreScene/MeshAlgo.h"
#include "IECoreScene/PrimitiveVariable.inl"

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/cache_aligned_allocator.h>

#include <iostream>
#include <iomanip>

using namespace Gaffer;
using namespace GafferScene;
using namespace GafferScotch;
using namespace IECore;
using namespace IECoreScene;
using namespace Imath;
using namespace tbb;

namespace
{
    template <typename T>
    using AlignedVector = std::vector<T, tbb::cache_aligned_allocator<T>>;
    
    // Local coordinate frame structure
    struct LocalFrame
    {
        V3f position;
        V3f normal;
        V3f tangent;
        V3f bitangent;

        void orthonormalize()
        {
            // Normalize normal vector
            float normalLength = normal.length();
            if (normalLength > 0)
                normal /= normalLength;
            else
                normal = V3f(0, 1, 0); // Fallback

            // Make tangent orthogonal to normal
            tangent = tangent - normal * tangent.dot(normal);
            float tangentLength = tangent.length();
            if (tangentLength > 0)
                tangent /= tangentLength;
            else
            {
                // Find a suitable tangent direction
                if (std::abs(normal.y) < 0.9f)
                    tangent = V3f(0, 1, 0).cross(normal).normalized();
                else
                    tangent = V3f(1, 0, 0).cross(normal).normalized();
            }

            // Compute bitangent to complete orthogonal frame
            bitangent = normal.cross(tangent).normalized();
        }

        M44f toMatrix() const
        {
            M44f matrix;
            
            // Set rotation part (column-major order)
            matrix[0][0] = normal.x;
            matrix[1][0] = normal.y;
            matrix[2][0] = normal.z;
            matrix[3][0] = 0;
            
            matrix[0][1] = bitangent.x;
            matrix[1][1] = bitangent.y;
            matrix[2][1] = bitangent.z;
            matrix[3][1] = 0;
            
            matrix[0][2] = tangent.x;
            matrix[1][2] = tangent.y;
            matrix[2][2] = tangent.z;
            matrix[3][2] = 0;
            
            // Set translation part
            matrix[0][3] = position.x;
            matrix[1][3] = position.y;
            matrix[2][3] = position.z;
            matrix[3][3] = 1;
            
            return matrix;
        }
    };

    // Build a local coordinate frame for mesh points using MeshPrimitiveEvaluator
    bool buildLocalFrameFromMesh(const IECoreScene::MeshPrimitive *mesh, const V3f &point, LocalFrame &frame)
    {
        // Create mesh evaluator
        IECoreScene::PrimitiveEvaluatorPtr evaluator = IECoreScene::MeshPrimitiveEvaluator::create(mesh);
        if (!evaluator)
            return false;
            
        // Create result object
        IECoreScene::PrimitiveEvaluator::ResultPtr result = evaluator->createResult();
        
        // Find closest point on the mesh
        if (!evaluator->closestPoint(point, result.get()))
            return false;
            
        // Get position and normal
        frame.position = result->point();
        frame.normal = result->normal();
        
        // Get triangle index and barycentric coordinates for proper tangent lookup
        int triangleIndex = static_cast<MeshPrimitiveEvaluator::Result*>(result.get())->triangleIndex();
        V3f baryCoords = static_cast<MeshPrimitiveEvaluator::Result*>(result.get())->barycentricCoordinates();
        
        // Get the vertex indices for this triangle
        const std::vector<int> &vertexIds = mesh->vertexIds()->readable();
        const int *triangleVertices = &vertexIds[triangleIndex * 3];
        V3i triVerts(triangleVertices[0], triangleVertices[1], triangleVertices[2]);
        
        // Try to calculate tangents from UVs first
        auto uvIt = mesh->variables.find("uv");
        if (uvIt == mesh->variables.end())
            uvIt = mesh->variables.find("st");
        if (uvIt == mesh->variables.end())
            uvIt = mesh->variables.find("UV");
            
        if (uvIt != mesh->variables.end())
        {
            // Calculate UV-based tangents for the whole mesh
            auto tangentResult = MeshAlgo::calculateTangents(mesh, uvIt->first, true, "P");
            
            // Interpolate tangent at our specific point using barycentric coordinates
            frame.tangent = GafferScotch::Detail::primVar<V3f>(tangentResult.first, &baryCoords[0], triangleIndex, triVerts);
            frame.bitangent = GafferScotch::Detail::primVar<V3f>(tangentResult.second, &baryCoords[0], triangleIndex, triVerts);
            
            // Ensure we have a properly orthonormal frame
            frame.orthonormalize();
            return true;
        }
        
        // If no UVs, try to calculate tangents from edges
        auto normalIt = mesh->variables.find("N");
        if (normalIt == mesh->variables.end())
        {
            // Calculate vertex normals if not present
            PrimitiveVariable normals = MeshAlgo::calculateNormals(mesh);
            MeshPrimitivePtr meshCopy = mesh->copy();
            meshCopy->variables["N"] = normals;
            
            // Calculate edge-based tangents
            auto tangentResult = MeshAlgo::calculateTangentsFromTwoEdges(meshCopy.get(), "P", "N", true, false);
            
            // Interpolate tangent at our specific point using barycentric coordinates
            frame.tangent = GafferScotch::Detail::primVar<V3f>(tangentResult.first, &baryCoords[0], triangleIndex, triVerts);
            frame.bitangent = GafferScotch::Detail::primVar<V3f>(tangentResult.second, &baryCoords[0], triangleIndex, triVerts);
            
            // Ensure we have a properly orthonormal frame
            frame.orthonormalize();
            return true;
        }
        else
        {
            // Calculate edge-based tangents using existing normals
            auto tangentResult = MeshAlgo::calculateTangentsFromTwoEdges(mesh, "P", normalIt->first, true, false);
            
            // Interpolate tangent at our specific point using barycentric coordinates
            frame.tangent = GafferScotch::Detail::primVar<V3f>(tangentResult.first, &baryCoords[0], triangleIndex, triVerts);
            frame.bitangent = GafferScotch::Detail::primVar<V3f>(tangentResult.second, &baryCoords[0], triangleIndex, triVerts);
            
            // Ensure we have a properly orthonormal frame
            frame.orthonormalize();
            return true;
        }
        
        // Fallback to simple cross-product approach (should rarely happen)
        V3f normal = frame.normal.normalized();
        V3f tangent;
        
        if (std::abs(normal.y) < 0.9f)
        {
            // Use up vector to create tangent
            tangent = V3f(0, 1, 0).cross(normal).normalized();
        }
        else
        {
            // Near-vertical normal, use x axis instead
            tangent = V3f(1, 0, 0).cross(normal).normalized();
        }
        
        // Create orthogonal frame
        frame.tangent = tangent;
        frame.bitangent = normal.cross(tangent).normalized();
        
        // Ensure we have a properly orthonormal frame
        frame.orthonormalize();
        return true;
    }

    // Point cloud adaptor for nanoflann
    struct PointCloudAdaptor
    {
        const std::vector<V3f> &points;

        PointCloudAdaptor(const std::vector<V3f> &pts) : points(pts) {}

        inline size_t kdtree_get_point_count() const { return points.size(); }

        inline float kdtree_get_pt(const size_t idx, const size_t dim) const
        {
            return dim == 0 ? points[idx].x : (dim == 1 ? points[idx].y : points[idx].z);
        }

        template <class BBOX>
        bool kdtree_get_bbox(BBOX &) const { return false; }
    };

    using KDTreeType = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>,
        PointCloudAdaptor,
        3 /* dimensionality */
        >;

    // Find neighboring points for a given point
    std::vector<int> findNeighbors(
        const std::vector<V3f> &points, 
        int pointIndex, 
        int numNeighbors, 
        KDTreeType &kdtree)
    {
        if (pointIndex >= points.size() || numNeighbors <= 0)
            return std::vector<int>();

        const V3f &queryPoint = points[pointIndex];
        const float queryPt[3] = {queryPoint.x, queryPoint.y, queryPoint.z};

        std::vector<uint32_t> indices(numNeighbors + 1); // +1 to include the point itself
        std::vector<float> distances(numNeighbors + 1);

        // Query with more points than needed as the point itself will be included
        size_t found = kdtree.knnSearch(queryPt, numNeighbors + 1, &indices[0], &distances[0]);

        // Filter out the point itself
        std::vector<int> neighbors;
        neighbors.reserve(numNeighbors);

        for (size_t i = 0; i < found; ++i)
        {
            if (indices[i] != pointIndex) // Skip the query point
            {
                neighbors.push_back(indices[i]);
                if (neighbors.size() >= numNeighbors)
                    break;
            }
        }

        return neighbors;
    }

    // Build a local coordinate frame from a point and its neighbors using PCA
    void buildLocalFrameFromNeighbors(const std::vector<V3f> &points, int centerIdx, 
                                      const std::vector<int> &neighbors, LocalFrame &frame)
    {
        frame.position = points[centerIdx];
        
        // Handle edge case with not enough neighbors
        if (neighbors.size() < 3)
        {
            // Fallback to identity-like frame
            frame.normal = V3f(0, 1, 0);
            frame.tangent = V3f(1, 0, 0);
            frame.bitangent = V3f(0, 0, 1);
            return;
        }

        // 1. Calculate a covariance matrix from neighboring points
        Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
        V3f centroid(0, 0, 0);
        
        // Compute local centroid
        for (int idx : neighbors)
        {
            centroid += points[idx];
        }
        centroid /= neighbors.size();
        
        // Build covariance matrix
        for (int idx : neighbors)
        {
            V3f p = points[idx] - centroid;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    covariance(i, j) += p[i] * p[j];
                }
            }
        }
        
        // 2. Perform eigen decomposition to get principal axes
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
        
        // 3. Extract principal directions (eigenvectors)
        // The eigenvectors are sorted by eigenvalue (smallest to largest)
        frame.normal = V3f(solver.eigenvectors().col(0)[0], 
                          solver.eigenvectors().col(0)[1],
                          solver.eigenvectors().col(0)[2]);
                        
        frame.bitangent = V3f(solver.eigenvectors().col(1)[0],
                             solver.eigenvectors().col(1)[1],
                             solver.eigenvectors().col(1)[2]);
                        
        frame.tangent = V3f(solver.eigenvectors().col(2)[0],
                           solver.eigenvectors().col(2)[1],
                           solver.eigenvectors().col(2)[2]);
        
        // 4. Ensure we have a right-handed coordinate system
        if (frame.normal.cross(frame.bitangent).dot(frame.tangent) < 0)
        {
            frame.tangent = -frame.tangent;
        }
        
        // 5. Final orthonormalization for numerical stability
        frame.orthonormalize();
    }

    struct InfluenceData
    {
        struct Entry
        {
            const int *indices;       // Points to the array of indices for this influence level
            const float *weights;     // Points to the array of weights for this influence level
            size_t count;
        };
        std::vector<Entry> influences;

        // Add a method to get all influences for a specific point
        void getPointInfluences(size_t pointIndex, std::vector<std::pair<int, float>> &pointInfluences) const
        {
            pointInfluences.clear();
            for (const auto &influence : influences)
            {
                if (pointIndex >= influence.count)
                    continue;

                const int idx = influence.indices[pointIndex];
                const float weight = influence.weights[pointIndex];

                if (idx >= 0 && weight > 0.0f)
                {
                    pointInfluences.emplace_back(idx, weight);
                }
            }
        }
    };

    bool getInfluenceData(const Primitive *primitive, int maxInfluences, InfluenceData &data)
    {
        data.influences.clear();
        data.influences.reserve(maxInfluences);

        for (int i = 1; i <= maxInfluences; ++i)
        {
            std::string indexName = "captureIndex" + std::to_string(i);
            std::string weightName = "captureWeight" + std::to_string(i);

            auto indexIt = primitive->variables.find(indexName);
            auto weightIt = primitive->variables.find(weightName);

            if (indexIt == primitive->variables.end() || weightIt == primitive->variables.end())
                continue;

            const IntVectorData *indices = runTimeCast<const IntVectorData>(indexIt->second.data.get());
            const FloatVectorData *weights = runTimeCast<const FloatVectorData>(weightIt->second.data.get());

            if (!indices || !weights)
                continue;

            InfluenceData::Entry entry;
            entry.indices = indices->readable().data();
            entry.weights = weights->readable().data();
            entry.count = indices->readable().size();

            data.influences.push_back(std::move(entry));
        }

        return !data.influences.empty();
    }

    M44f polarDecomposition(const M44f &m)
    {
        // Extract the 3x3 rotation+scale component
        Eigen::Matrix3f A;
        A << m[0][0], m[0][1], m[0][2],
             m[1][0], m[1][1], m[1][2],
             m[2][0], m[2][1], m[2][2];
        
        // Compute SVD
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        
        // Reconstruct rotation matrix
        Eigen::Matrix3f R = svd.matrixU() * svd.matrixV().transpose();
        
        // Ensure proper rotation (determinant should be 1)
        if (R.determinant() < 0) {
            Eigen::Matrix3f correction = Eigen::Matrix3f::Identity();
            correction(2, 2) = -1;
            R = svd.matrixU() * correction * svd.matrixV().transpose();
        }
        
        // Reconstruct as 4x4 matrix with original translation
        M44f result;
        result[0][0] = R(0, 0); result[0][1] = R(0, 1); result[0][2] = R(0, 2); result[0][3] = m[0][3];
        result[1][0] = R(1, 0); result[1][1] = R(1, 1); result[1][2] = R(1, 2); result[1][3] = m[1][3];
        result[2][0] = R(2, 0); result[2][1] = R(2, 1); result[2][2] = R(2, 2); result[2][3] = m[2][3];
        result[3][0] = m[3][0]; result[3][1] = m[3][1]; result[3][2] = m[3][2]; result[3][3] = m[3][3];
        
        return result;
    }

    // Compute local frames for each deformer point (rest pose)
    void computeLocalFrames(
        const std::vector<V3f> &points,
        const Primitive *primitive,
        int neighborPoints,
        std::vector<LocalFrame> &frames)
    {
        frames.resize(points.size());
        
        // Check if we have a mesh primitive
        const MeshPrimitive *mesh = runTimeCast<const MeshPrimitive>(primitive);
        
        // Setup KD tree for neighbor queries if needed
        PointCloudAdaptor adaptor(points);
        KDTreeType kdtree(3, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
        kdtree.buildIndex();
        
        // Calculate frames in parallel
        parallel_for(blocked_range<size_t>(0, points.size(), 1024),
            [&](const blocked_range<size_t> &range)
            {
                for (size_t i = range.begin(); i != range.end(); ++i)
                {
                    LocalFrame &frame = frames[i];
                    
                    // Try mesh evaluator first if available
                    bool usedMeshEval = false;
                    if (mesh)
                    {
                        usedMeshEval = buildLocalFrameFromMesh(mesh, points[i], frame);
                    }
                    
                    // Fall back to neighbor-based frame calculation if needed
                    if (!usedMeshEval)
                    {
                        // Find neighbors for local frame construction
                        std::vector<int> neighbors = findNeighbors(points, i, neighborPoints, kdtree);
                        
                        // Build local frame using PCA on neighbors
                        buildLocalFrameFromNeighbors(points, i, neighbors, frame);
                    }
                }
            }
        );
    }

    // Compute a weighted average transformation matrix from all influences
    M44f computeAverageMatrix(
        const V3f &currentPos,
        const std::vector<V3f> &staticPos,
        const std::vector<V3f> &animatedPos,
        const std::vector<M44f> &sourceMatrices,
        const std::vector<std::pair<int, float>> &influences,
        bool applyRigidProjection)
    {
        M44f avgMatrix;
        avgMatrix.makeIdentity();
        float totalWeight = 0.0f;
        
        // Calculate weighted average matrix and translation
        for (const auto &influence : influences)
        {
            const int sourceIndex = influence.first;
            const float weight = influence.second;
            
            if (weight <= 0.0f || sourceIndex < 0 || 
                sourceIndex >= staticPos.size() || 
                sourceIndex >= sourceMatrices.size())
                continue;
                
            const V3f &staticPoint = staticPos[sourceIndex];
            const V3f &animatedPoint = animatedPos[sourceIndex];
            const M44f &sourceMatrix = sourceMatrices[sourceIndex];
            
            // Create a transform from static to animated
            M44f staticToWorld = sourceMatrix;
            
            // Add the difference between static and animated positions
            V3f translation = animatedPoint - staticPoint;
            M44f animatedMatrix = staticToWorld;
            animatedMatrix[0][3] += translation.x;
            animatedMatrix[1][3] += translation.y;
            animatedMatrix[2][3] += translation.z;
            
            // Calculate full transform between rest and deformed state
            M44f transform = staticToWorld.inverse() * animatedMatrix;
            
            // Accumulate weighted transform
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    avgMatrix[i][j] += transform[i][j] * weight;
                }
            }
            
            totalWeight += weight;
        }
        
        // Normalize by total weight
        if (totalWeight > 0.0f)
        {
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    avgMatrix[i][j] /= totalWeight;
                }
            }
            
            // Apply polar decomposition for rigid projection if requested
            if (applyRigidProjection)
            {
                avgMatrix = polarDecomposition(avgMatrix);
            }
        }
        else
        {
            avgMatrix.makeIdentity();
        }
        
        return avgMatrix;
    }
    
    // Apply weighted average matrix to a point (used in parallel processing)
    V3f applyAverageMatrix(
        const V3f &currentPos,
        const std::vector<V3f> &staticPos,
        const std::vector<V3f> &animatedPos,
        const M44f &averageMatrix,
        const std::vector<std::pair<int, float>> &influences)
    {
        if (influences.empty())
            return currentPos;
            
        // Find center of influence points (weighted)
        V3f staticCenter(0, 0, 0);
        float totalWeight = 0.0f;
        
        for (const auto &influence : influences)
        {
            const int sourceIndex = influence.first;
            const float weight = influence.second;
            
            if (weight <= 0.0f || sourceIndex < 0 || sourceIndex >= staticPos.size())
                continue;
                
            staticCenter += staticPos[sourceIndex] * weight;
            totalWeight += weight;
        }
        
        if (totalWeight <= 0.0f)
            return currentPos;
            
        staticCenter /= totalWeight;
        
        // Transform point around the center
        V3f localPoint = currentPos - staticCenter;
        V3f transformedPoint;
        averageMatrix.multVecMatrix(localPoint, transformedPoint);
        
        // Add back to static center transformed by average matrix
        V3f transformedCenter;
        averageMatrix.multVecMatrix(V3f(0, 0, 0), transformedCenter);
        
        return transformedPoint + staticCenter + transformedCenter;
    }

    void hashPositions(const IECoreScene::Primitive *primitive, IECore::MurmurHash &h)
    {
        if (!primitive)
            return;

        auto pIt = primitive->variables.find("P");
        if (pIt == primitive->variables.end())
            return;

        const V3fVectorData *positions = runTimeCast<const V3fVectorData>(pIt->second.data.get());
        if (!positions)
            return;

        const std::vector<V3f> &pos = positions->readable();
        h.append(pos.size());
        if (!pos.empty())
        {
            h.append(&pos[0], pos.size());
        }
    }
}

IE_CORE_DEFINERUNTIMETYPED(MatrixDeform);

size_t MatrixDeform::g_firstPlugIndex = 0;

MatrixDeform::MatrixDeform(const std::string &name)
    : Deformer(name)
{
    storeIndexOfNextChild(g_firstPlugIndex);

    addChild(new ScenePlug("staticDeformer"));
    addChild(new ScenePlug("animatedDeformer"));
    addChild(new StringPlug("deformerPath", Plug::In, ""));
    addChild(new IntPlug("neighborPoints", Plug::In, 8, 3));
    addChild(new BoolPlug("rigidProjection", Plug::In, true));
    addChild(new BoolPlug("cleanupAttributes", Plug::In, true));
}

ScenePlug *MatrixDeform::staticDeformerPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

const ScenePlug *MatrixDeform::staticDeformerPlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

ScenePlug *MatrixDeform::animatedDeformerPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex + 1);
}

const ScenePlug *MatrixDeform::animatedDeformerPlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex + 1);
}

StringPlug *MatrixDeform::deformerPathPlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 2);
}

const StringPlug *MatrixDeform::deformerPathPlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 2);
}

IntPlug *MatrixDeform::neighborPointsPlug()
{
    return getChild<IntPlug>(g_firstPlugIndex + 3);
}

const IntPlug *MatrixDeform::neighborPointsPlug() const
{
    return getChild<IntPlug>(g_firstPlugIndex + 3);
}

BoolPlug *MatrixDeform::rigidProjectionPlug()
{
    return getChild<BoolPlug>(g_firstPlugIndex + 4);
}

const BoolPlug *MatrixDeform::rigidProjectionPlug() const
{
    return getChild<BoolPlug>(g_firstPlugIndex + 4);
}

BoolPlug *MatrixDeform::cleanupAttributesPlug()
{
    return getChild<BoolPlug>(g_firstPlugIndex + 5);
}

const BoolPlug *MatrixDeform::cleanupAttributesPlug() const
{
    return getChild<BoolPlug>(g_firstPlugIndex + 5);
}

void MatrixDeform::affects(const Plug *input, AffectedPlugsContainer &outputs) const
{
    Deformer::affects(input, outputs);

    if (input == staticDeformerPlug()->objectPlug() ||
        input == animatedDeformerPlug()->objectPlug() ||
        input == deformerPathPlug() ||
        input == neighborPointsPlug() ||
        input == rigidProjectionPlug() ||
        input == cleanupAttributesPlug())
    {
        outputs.push_back(outPlug()->objectPlug());
    }
}

bool MatrixDeform::affectsProcessedObjectBound(const Plug *input) const
{
    return input == staticDeformerPlug()->objectPlug() ||
           input == animatedDeformerPlug()->objectPlug() ||
           input == deformerPathPlug() ||
           input == neighborPointsPlug() ||
           input == rigidProjectionPlug();
}

bool MatrixDeform::affectsProcessedObject(const Plug *input) const
{
    return input == staticDeformerPlug()->objectPlug() ||
           input == animatedDeformerPlug()->objectPlug() ||
           input == deformerPathPlug() ||
           input == neighborPointsPlug() ||
           input == rigidProjectionPlug() ||
           input == cleanupAttributesPlug();
}

void MatrixDeform::hashProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    Deformer::hashProcessedObject(path, context, h);

    const ScenePath deformerPath = GafferScotch::makeScenePath(deformerPathPlug()->getValue());
    ConstObjectPtr inputObject = inPlug()->object(path);

    const Primitive *inputPrimitive = runTimeCast<const Primitive>(inputObject.get());
    if (!inputPrimitive)
    {
        h.append(inPlug()->objectHash(path));
        return;
    }

    ConstObjectPtr staticDeformerObject = staticDeformerPlug()->object(deformerPath);
    ConstObjectPtr animatedDeformerObject = animatedDeformerPlug()->object(deformerPath);

    const Primitive *staticDeformerPrimitive = runTimeCast<const Primitive>(staticDeformerObject.get());
    const Primitive *animatedDeformerPrimitive = runTimeCast<const Primitive>(animatedDeformerObject.get());

    if (inputPrimitive && staticDeformerPrimitive && animatedDeformerPrimitive)
    {
        hashPositions(staticDeformerPrimitive, h);
        hashPositions(animatedDeformerPrimitive, h);
        h.append(inPlug()->objectHash(path));
        neighborPointsPlug()->hash(h);
        rigidProjectionPlug()->hash(h);
        cleanupAttributesPlug()->hash(h);
    }
    else
    {
        h.append(inPlug()->objectHash(path));
        h.append(staticDeformerPlug()->objectHash(deformerPath));
        h.append(animatedDeformerPlug()->objectHash(deformerPath));
        deformerPathPlug()->hash(h);
        neighborPointsPlug()->hash(h);
        rigidProjectionPlug()->hash(h);
        cleanupAttributesPlug()->hash(h);
    }
}

void MatrixDeform::hashProcessedObjectBound(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    hashProcessedObject(path, context, h);
}

Imath::Box3f MatrixDeform::computeProcessedObjectBound(const ScenePath &path, const Gaffer::Context *context) const
{
    const Box3f inputBound = inPlug()->boundPlug()->getValue();
    if (inputBound.isEmpty())
    {
        return inputBound;
    }

    ConstObjectPtr inputObject = inPlug()->objectPlug()->getValue();
    const Primitive *inputPrimitive = runTimeCast<const Primitive>(inputObject.get());
    if (!inputPrimitive)
    {
        return inputBound;
    }

    const ScenePath deformerPath = GafferScotch::makeScenePath(deformerPathPlug()->getValue());

    ConstObjectPtr staticObj = staticDeformerPlug()->object(deformerPath);
    ConstObjectPtr animatedObj = animatedDeformerPlug()->object(deformerPath);

    const Primitive *staticDeformerPrimitive = runTimeCast<const Primitive>(staticObj.get());
    const Primitive *animatedDeformerPrimitive = runTimeCast<const Primitive>(animatedObj.get());

    if (!staticDeformerPrimitive || !animatedDeformerPrimitive)
    {
        return inputBound;
    }

    // Get the positions from both deformers
    auto staticPIt = staticDeformerPrimitive->variables.find("P");
    auto animatedPIt = animatedDeformerPrimitive->variables.find("P");

    if (staticPIt == staticDeformerPrimitive->variables.end() ||
        animatedPIt == animatedDeformerPrimitive->variables.end())
    {
        return inputBound;
    }

    const V3fVectorData *staticPositions = runTimeCast<const V3fVectorData>(staticPIt->second.data.get());
    const V3fVectorData *animatedPositions = runTimeCast<const V3fVectorData>(animatedPIt->second.data.get());

    if (!staticPositions || !animatedPositions)
    {
        return inputBound;
    }

    const std::vector<V3f> &staticPos = staticPositions->readable();
    const std::vector<V3f> &animatedPos = animatedPositions->readable();

    if (staticPos.size() != animatedPos.size() || staticPos.empty())
    {
        return inputBound;
    }

    // Calculate a conservative bounding box using maximum displacement
    float maxDistance = 0;
    
    // Find maximum distance from any input point to bounds center
    V3f center = inputBound.center();
    for (auto p : staticPos)
    {
        maxDistance = std::max(maxDistance, (p - center).length());
    }
    
    // Add a generous padding factor to account for rotation
    maxDistance *= 1.5f;
    
    Box3f result = inputBound;
    V3f padding(maxDistance, maxDistance, maxDistance);
    result.min -= padding;
    result.max += padding;
    
    return result;
}

IECore::ConstObjectPtr MatrixDeform::computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject) const
{
    const Primitive *inputPrimitive = runTimeCast<const Primitive>(inputObject);
    if (!inputPrimitive)
        return inputObject;

    const ScenePath deformerPath = GafferScotch::makeScenePath(deformerPathPlug()->getValue());

    // Get static and animated deformer objects
    ConstObjectPtr staticDeformerObject = staticDeformerPlug()->object(deformerPath);
    ConstObjectPtr animatedDeformerObject = animatedDeformerPlug()->object(deformerPath);

    const Primitive *staticDeformerPrimitive = runTimeCast<const Primitive>(staticDeformerObject.get());
    const Primitive *animatedDeformerPrimitive = runTimeCast<const Primitive>(animatedDeformerObject.get());

    if (!staticDeformerPrimitive || !animatedDeformerPrimitive)
        return inputObject;

    // Get all position data
    PrimitivePtr result = inputPrimitive->copy();
    auto pIt = result->variables.find("P");
    if (pIt == result->variables.end())
        return result;

    V3fVectorDataPtr positionData = runTimeCast<V3fVectorData>(pIt->second.data);
    if (!positionData)
        return result;

    // Get static and animated positions
    auto staticPIt = staticDeformerPrimitive->variables.find("P");
    auto animatedPIt = animatedDeformerPrimitive->variables.find("P");

    if (staticPIt == staticDeformerPrimitive->variables.end() ||
        animatedPIt == animatedDeformerPrimitive->variables.end())
        return result;

    const V3fVectorData *staticPositions = runTimeCast<const V3fVectorData>(staticPIt->second.data.get());
    const V3fVectorData *animatedPositions = runTimeCast<const V3fVectorData>(animatedPIt->second.data.get());

    if (!staticPositions || !animatedPositions)
        return result;

    const std::vector<V3f> &staticPos = staticPositions->readable();
    const std::vector<V3f> &animatedPos = animatedPositions->readable();
    std::vector<V3f> &positions = positionData->writable();

    // Safety check for positions size
    if (staticPos.size() != animatedPos.size() || staticPos.empty())
        return result;

    // Get influence data
    auto captureInfluencesIt = inputPrimitive->variables.find("captureInfluences");
    if (captureInfluencesIt == inputPrimitive->variables.end())
        return result;

    const std::vector<int> &numInfluences = runTimeCast<const IntVectorData>(captureInfluencesIt->second.data.get())->readable();

    // Count max influences
    int maxInfluences = 0;
    for (int count : numInfluences)
    {
        maxInfluences = std::max(maxInfluences, count);
    }

    // Get influence data from CaptureWeights
    InfluenceData influenceData;
    if (!getInfluenceData(inputPrimitive, maxInfluences, influenceData))
        return result;
        
    // Get the rigid projection flag and neighbor points
    bool applyRigidProjection = rigidProjectionPlug()->getValue();
    int neighborPoints = neighborPointsPlug()->getValue();
    
    // Compute local frames for static deformer points
    std::vector<LocalFrame> sourceFrames;
    computeLocalFrames(staticPos, staticDeformerPrimitive, neighborPoints, sourceFrames);
    
    // Convert local frames to matrices
    std::vector<M44f> sourceMatrices(sourceFrames.size());
    for (size_t i = 0; i < sourceFrames.size(); ++i)
    {
        sourceMatrices[i] = sourceFrames[i].toMatrix();
    }

    // Process each point in parallel
    parallel_for(blocked_range<size_t>(0, positions.size()),
                 [&](const blocked_range<size_t> &range)
                 {
                     // Thread-local storage for influences
                     std::vector<std::pair<int, float>> influences;
                     
                     for (size_t i = range.begin(); i != range.end(); ++i)
                     {
                         const int maxInfluencesForPoint = (i < numInfluences.size()) ? numInfluences[i] : 0;
                         if (maxInfluencesForPoint <= 0)
                             continue;

                         // Get all influences for this point
                         influenceData.getPointInfluences(i, influences);
                         if (influences.empty())
                             continue;
                             
                         // Calculate weighted average matrix (Houdini-style)
                         M44f averageMatrix = computeAverageMatrix(
                             positions[i], 
                             staticPos, 
                             animatedPos, 
                             sourceMatrices, 
                             influences,
                             applyRigidProjection
                         );
                         
                         // Apply the average matrix to transform the point
                         positions[i] = applyAverageMatrix(
                             positions[i],
                             staticPos,
                             animatedPos,
                             averageMatrix,
                             influences
                         );
                     }
                 });

    // If requested, clean up the capture attributes
    if (cleanupAttributesPlug()->getValue())
    {
        result->variables.erase("captureInfluences");
        for (int i = 1; i <= maxInfluences; ++i)
        {
            result->variables.erase("captureIndex" + std::to_string(i));
            result->variables.erase("captureWeight" + std::to_string(i));
        }
        
        // Also clean up any matrix attributes if they exist
        result->variables.erase("captureMatrices");
        result->variables.erase("captureFrameNormals");
        result->variables.erase("captureFrameTangents");
        result->variables.erase("captureFrameBitangents");
    }

    return result;
} 