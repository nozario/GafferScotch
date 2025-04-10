#include <math.h>
#include <Eigen/Dense>
#include "GafferScotch/MatrixDeform.h"
#include "GafferScotch/ScenePathUtil.h"
#include "GafferScotch/nanoflann.hpp"
#include "GafferScotch/CurvesDataStructures.h"

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
            // 1. Normalize normal vector
            float normalLength = normal.length();
            if (normalLength > 0.001f)
            {
                normal /= normalLength;
            }
            else
            {
                // Fallback to world up if normal is too small
                normal = V3f(0, 1, 0);
            }
            
            // 2. Make sure the tangent is not parallel to the normal
            float dotNT = normal.dot(tangent);
            if (std::abs(dotNT) > 0.999f || tangent.length() < 0.001f)
            {
                // Find a suitable tangent direction perpendicular to normal
                if (std::abs(normal.y) < 0.9f)
                {
                    // Use world up vector to create tangent
                    tangent = V3f(0, 1, 0).cross(normal).normalized();
                }
                else
                {
                    // Near-vertical normal, use world x instead
                    tangent = V3f(1, 0, 0).cross(normal).normalized();
                }
            }
            else
            {
                // Gram-Schmidt to make tangent orthogonal to normal
                tangent -= normal * dotNT;
                float tangentLength = tangent.length();
                if (tangentLength > 0.001f)
                {
                    tangent /= tangentLength;
                }
            }
            
            // 3. Compute bitangent from normal and tangent to ensure a right-handed frame
            bitangent = normal.cross(tangent);
            
            // 4. Normalize the bitangent (should be already normalized from cross product)
            float bitangentLength = bitangent.length();
            if (bitangentLength > 0.001f)
            {
                bitangent /= bitangentLength;
            }
            
            // 5. Final validation checks
            if (std::abs(normal.dot(tangent)) > 0.01f || 
                std::abs(normal.dot(bitangent)) > 0.01f ||
                std::abs(tangent.dot(bitangent)) > 0.01f)
            {
                // Frame is not properly orthogonal, rebuild with standard approach
                if (std::abs(normal.y) < 0.9f)
                {
                    tangent = V3f(0, 1, 0).cross(normal).normalized();
                }
                else
                {
                    tangent = V3f(1, 0, 0).cross(normal).normalized();
                }
                bitangent = normal.cross(tangent).normalized();
            }
        }

        M44f toMatrix() const
        {
            // Create matrix directly based on basis vectors and position
            // This creates a coordinate frame as expected by the deformer
            M44f matrix;
            
            // First row - tangent
            matrix[0][0] = tangent.x;
            matrix[0][1] = tangent.y; 
            matrix[0][2] = tangent.z;
            matrix[0][3] = 0;
            
            // Second row - bitangent
            matrix[1][0] = bitangent.x;
            matrix[1][1] = bitangent.y;
            matrix[1][2] = bitangent.z;
            matrix[1][3] = 0;
            
            // Third row - normal
            matrix[2][0] = normal.x;
            matrix[2][1] = normal.y;
            matrix[2][2] = normal.z;
            matrix[2][3] = 0;
            
            // Last row - position
            matrix[3][0] = position.x;
            matrix[3][1] = position.y;
            matrix[3][2] = position.z;
            matrix[3][3] = 1;
            
            return matrix;
        }

        V3f toLocalSpace(const V3f &point) const
        {
            V3f localPoint = point - position;
            float dotNT = localPoint.dot(tangent);
            float dotNB = localPoint.dot(bitangent);
            float dotNN = localPoint.dot(normal);
            return V3f(dotNT, dotNB, dotNN);
        }

        V3f toWorldSpace(const V3f &point) const
        {
            return position + point.x * tangent + point.y * bitangent + point.z * normal;
        }
    };

    // Build a local coordinate frame for mesh points using MeshPrimitiveEvaluator
    bool buildLocalFrameFromMesh(const IECoreScene::MeshPrimitive *mesh, const V3f &point, LocalFrame &frame)
    {
        // Triangulate the mesh if needed - MeshPrimitiveEvaluator requires triangulated meshes
        IECoreScene::MeshPrimitivePtr triangulatedMesh;
        const IECoreScene::MeshPrimitive *meshToUse = mesh;
        
        // Check if the mesh is already triangulated
        bool isTriangulated = true;
        const std::vector<int> &verticesPerFace = mesh->verticesPerFace()->readable();
        for (int count : verticesPerFace)
        {
            if (count != 3)
            {
                isTriangulated = false;
                break;
            }
        }
        
        // Triangulate if needed
        if (!isTriangulated)
        {
            try
            {
                triangulatedMesh = IECoreScene::MeshAlgo::triangulate(mesh);
                meshToUse = triangulatedMesh.get();
            }
            catch (const std::exception &e)
            {
                // If triangulation fails, we'll fall back to neighbor-based frame calculation
                return false;
            }
        }
        
        // Create mesh evaluator
        IECoreScene::PrimitiveEvaluatorPtr evaluator = IECoreScene::MeshPrimitiveEvaluator::create(meshToUse);
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
        const std::vector<int> &vertexIds = meshToUse->vertexIds()->readable();
        const int *triangleVertices = &vertexIds[triangleIndex * 3];
        V3i triVerts(triangleVertices[0], triangleVertices[1], triangleVertices[2]);
        
        // Skip UV-based tangent calculation since UVs might overlap and cause issues
        
        // Use edge-based tangent calculation instead
        auto normalIt = meshToUse->variables.find("N");
        if (normalIt == meshToUse->variables.end())
        {
            // Calculate vertex normals if not present
            PrimitiveVariable normals = MeshAlgo::calculateNormals(meshToUse);
            MeshPrimitivePtr meshCopy = meshToUse->copy();
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
            auto tangentResult = MeshAlgo::calculateTangentsFromTwoEdges(meshToUse, "P", normalIt->first, true, false);
            
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
        
        // Add the center point to ensure it influences the frame
        centroid = (centroid + points[centerIdx]) * 0.5f;
        
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
        
        // Weight the center point more heavily
        V3f centerDiff = points[centerIdx] - centroid;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                covariance(i, j) += centerDiff[i] * centerDiff[j] * 2.0f;
            }
        }
        
        // 2. Perform eigen decomposition to get principal axes
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
        
        // 3. Extract principal directions (eigenvectors)
        // The eigenvectors are sorted by eigenvalue (smallest to largest)
        // Use the smallest eigenvalue's direction as normal (typically represents the normal to the surface)
        frame.normal = V3f(solver.eigenvectors().col(0)[0], 
                          solver.eigenvectors().col(0)[1],
                          solver.eigenvectors().col(0)[2]);
                        
        // Ensure consistent orientation - point normal toward global Y if possible
        if (frame.normal.y < 0)
        {
            frame.normal = -frame.normal;
        }
        
        // 4. Calculate tangent and bitangent based on normal
        // Use our improved orthonormalize method instead of directly using eigenvectors
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
        // Start with identity matrix
        M44f totalXform;
        totalXform.makeIdentity();
        float totalWeight = 0.0f;
        
        // Accumulate weighted matrices directly (like Houdini's totalxform)
        for (const auto &influence : influences)
        {
            const int sourceIndex = influence.first;
            const float weight = influence.second;
            
            if (weight <= 0.0f || sourceIndex < 0 || 
                sourceIndex >= staticPos.size() || 
                sourceIndex >= sourceMatrices.size())
                continue;
                
            // Get the source matrix directly
            const M44f &sourceMatrix = sourceMatrices[sourceIndex];
            
            // Accumulate weighted matrix (just like Houdini)
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    totalXform[i][j] += sourceMatrix[i][j] * weight;
                }
            }
            
            totalWeight += weight;
        }
        
        // Normalize the accumulated matrix
        if (totalWeight > 0.0f)
        {
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    totalXform[i][j] /= totalWeight;
                }
            }
            
            // Apply polar decomposition for rigid projection if requested
            if (applyRigidProjection)
            {
                totalXform = polarDecomposition(totalXform);
            }
        }
        
        return totalXform;
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
        
        // Calculate weighted delta following Houdini's approach
        V3f delta(0, 0, 0);
        float totalWeight = 0.0f;
        
        for (const auto &influence : influences)
        {
            const int sourceIndex = influence.first;
            const float weight = influence.second;
            
            if (weight <= 0.0f || sourceIndex < 0 || sourceIndex >= staticPos.size())
                continue;
            
            // Get static point position (oldcenter in Houdini)
            const V3f &staticPoint = staticPos[sourceIndex];
            
            // Calculate difference between animated and static (diff in Houdini)
            const V3f &animatedPoint = animatedPos[sourceIndex];
            V3f translation = animatedPoint - staticPoint;
            
            // Move point to local space
            V3f localPoint = currentPos - staticPoint;
            
            // Apply matrix transformation
            V3f transformedPoint;
            averageMatrix.multVecMatrix(localPoint, transformedPoint);
            
            // Move back to world space and add translation
            transformedPoint += staticPoint + translation;
            
            // Calculate difference and apply weight
            V3f pointDelta = transformedPoint - currentPos;
            delta += pointDelta * weight;
            totalWeight += weight;
        }
        
        // Apply normalized delta to current position
        if (totalWeight > 0.0f)
        {
            delta /= totalWeight;
            return currentPos + delta;
        }
        
        return currentPos;
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

    void computeLocalFrame(LocalFrame& frame, const V3fVectorData* points, const std::vector<int>& pointIndices, const V3f* N)
    {
        // Use the first point as the position
        frame.position = points->readable()[pointIndices[0]];
        
        // If we have a single point and a normal, use the normal
        if (pointIndices.size() == 1 && N)
        {
            frame.normal = N[pointIndices[0]];
            frame.orthonormalize();
            return;
        }
        
        // With two points, use the vector between them as tangent
        if (pointIndices.size() == 2)
        {
            if (N)
            {
                frame.normal = N[pointIndices[0]];
                
                // Use direction between points for tangent
                V3f direction = points->readable()[pointIndices[1]] - frame.position;
                float length = direction.length();
                
                if (length > 0.0001f)
                {
                    // Project direction onto normal plane
                    direction /= length;
                    float dot = direction.dot(frame.normal);
                    frame.tangent = direction - frame.normal * dot;
                    
                    float tangentLength = frame.tangent.length();
                    if (tangentLength > 0.0001f)
                    {
                        frame.tangent /= tangentLength;
                    }
                    else
                    {
                        // If tangent is close to parallel with normal, choose arbitrary tangent
                        frame.orthonormalize();
                        return;
                    }
                    
                    // Create bitangent from normal and tangent
                    frame.bitangent = frame.normal.cross(frame.tangent);
                }
                else
                {
                    frame.orthonormalize();
                }
            }
            else
            {
                // No normal provided, use direction as tangent and find suitable normal
                V3f direction = points->readable()[pointIndices[1]] - frame.position;
                float length = direction.length();
                
                if (length > 0.0001f)
                {
                    frame.tangent = direction / length;
                    
                    // Find best normal direction
                    if (std::abs(frame.tangent.y) < 0.9f)
                    {
                        // Use world up to create normal
                        frame.normal = frame.tangent.cross(V3f(0, 1, 0)).normalized();
                    }
                    else
                    {
                        // Use world x for near-vertical tangents
                        frame.normal = frame.tangent.cross(V3f(1, 0, 0)).normalized();
                    }
                    
                    frame.bitangent = frame.normal.cross(frame.tangent);
                }
                else
                {
                    // Fallback to default orientation
                    frame.normal = V3f(0, 1, 0);
                    frame.orthonormalize();
                }
            }
            
            return;
        }
        
        // For 3+ points, use PCA to determine the frame
        
        // 1. Calculate centroid with a higher weight for the center point
        V3f centroid(0, 0, 0);
        float totalWeight = 0.0f;
        const float centerWeight = 3.0f; // Higher weight for center point
        
        // Add the center point with higher weight
        centroid += frame.position * centerWeight;
        totalWeight += centerWeight;
        
        // Add other points with normal weight
        for (size_t i = 1; i < pointIndices.size(); ++i)
        {
            centroid += points->readable()[pointIndices[i]];
            totalWeight += 1.0f;
        }
        
        centroid /= totalWeight;
        
        // 2. Build covariance matrix
        float cov[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
        
        // Add contribution from center point with higher weight
        V3f delta = frame.position - centroid;
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                cov[i][j] += centerWeight * delta[i] * delta[j];
            }
        }
        
        // Add contribution from other points
        for (size_t p = 1; p < pointIndices.size(); ++p)
        {
            delta = points->readable()[pointIndices[p]] - centroid;
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    cov[i][j] += delta[i] * delta[j];
                }
            }
        }
        
        // 3. Find eigenvectors - simple power iteration for smallest eigenvector (normal)
        V3f ev(1, 1, 1);
        V3f lastEv;
        
        // Normalize covariance matrix for numerical stability
        float maxCoeff = 0.0f;
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                maxCoeff = std::max(maxCoeff, std::abs(cov[i][j]));
            }
        }
        
        if (maxCoeff > 0.0001f)
        {
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    cov[i][j] /= maxCoeff;
                }
            }
        }
        
        // Initialize ev with a vector not aligned with any of the eigenvectors
        // for better convergence
        ev = V3f(1.0f, 0.7f, 0.3f).normalized();
        
        // Power iteration to find smallest eigenvector (normal direction)
        const int maxIter = 10;
        const float convergenceThreshold = 0.0001f;
        
        for (int iter = 0; iter < maxIter; ++iter)
        {
            lastEv = ev;
            
            // Multiply by covariance matrix
            V3f tmp(0, 0, 0);
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    tmp[i] += cov[i][j] * ev[j];
                }
            }
            
            ev = tmp;
            
            // Inverse power iteration - divide by largest component
            float maxComp = 0.0f;
            for (int i = 0; i < 3; ++i)
            {
                maxComp = std::max(maxComp, std::abs(ev[i]));
            }
            
            if (maxComp > 0.0001f)
            {
                ev /= maxComp;
            }
            else
            {
                // Random restart if we hit a zero vector
                ev = V3f(0.1f * (iter + 1), 0.7f, 0.3f).normalized();
            }
            
            // Check for convergence
            float dot = ev.dot(lastEv);
            if (std::abs(std::abs(dot) - 1.0f) < convergenceThreshold)
            {
                break;
            }
        }
        
        // Make sure sign is consistent - align with provided normal if available
        if (N)
        {
            if (ev.dot(N[pointIndices[0]]) < 0)
            {
                ev = -ev;
            }
            
            // If eigenvector is unreliable, use provided normal
            if (ev.length() < 0.9f)
            {
                ev = N[pointIndices[0]];
            }
        }
        
        // Normalize the final result
        frame.normal = ev.normalized();
        
        // Complete the frame
        frame.orthonormalize();
    }

    void smoothFrames(std::vector<LocalFrame>& frames, const std::vector<std::vector<int>>& neighbors, int iterations = 2)
    {
        std::vector<LocalFrame> originalFrames = frames;
        
        // Store original positions
        std::vector<V3f> positions;
        for (const auto& frame : frames)
        {
            positions.push_back(frame.position);
        }
        
        // Perform multiple smoothing iterations
        for (int iter = 0; iter < iterations; ++iter)
        {
            std::vector<LocalFrame> smoothedFrames = frames;
            
            // For each frame
            for (size_t i = 0; i < frames.size(); ++i)
            {
                if (neighbors[i].empty())
                    continue;
                    
                // Get weighted average of neighboring frames
                V3f avgNormal(0, 0, 0);
                V3f avgTangent(0, 0, 0);
                V3f avgBitangent(0, 0, 0);
                float totalWeight = 0.0f;
                
                // Add contribution from original frame (higher weight)
                float selfWeight = 2.0f;
                avgNormal += originalFrames[i].normal * selfWeight;
                avgTangent += originalFrames[i].tangent * selfWeight;
                avgBitangent += originalFrames[i].bitangent * selfWeight;
                totalWeight += selfWeight;
                
                // Add contribution from neighbors
                for (int neighborIdx : neighbors[i])
                {
                    // Compute distance-based weight
                    float dist = (positions[i] - positions[neighborIdx]).length();
                    float weight = 1.0f / (0.1f + dist * dist);
                    
                    // Ensure consistent orientation relative to original frame
                    V3f neighborNormal = frames[neighborIdx].normal;
                    V3f neighborTangent = frames[neighborIdx].tangent;
                    V3f neighborBitangent = frames[neighborIdx].bitangent;
                    
                    // Check if frames have opposite orientation
                    if (originalFrames[i].normal.dot(neighborNormal) < 0)
                    {
                        // Flip orientation of neighbor's frame
                        neighborNormal = -neighborNormal;
                        neighborTangent = -neighborTangent;
                        neighborBitangent = -neighborBitangent;
                    }
                    
                    avgNormal += neighborNormal * weight;
                    avgTangent += neighborTangent * weight;
                    avgBitangent += neighborBitangent * weight;
                    totalWeight += weight;
                }
                
                // Normalize and update smoothed frame
                if (totalWeight > 0.0f)
                {
                    smoothedFrames[i].normal = (avgNormal / totalWeight).normalized();
                    smoothedFrames[i].tangent = (avgTangent / totalWeight).normalized();
                    smoothedFrames[i].bitangent = (avgBitangent / totalWeight).normalized();
                    
                    // Re-orthonormalize to ensure frame consistency
                    smoothedFrames[i].orthonormalize();
                }
            }
            
            frames = smoothedFrames;
        }
        
        // Ensure positions remain unchanged
        for (size_t i = 0; i < frames.size(); ++i)
        {
            frames[i].position = positions[i];
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
                         
                         // Initialize new position and weight sum
                         V3f newPosition(0, 0, 0);
                         float totalWeight = 0.0f;
                         
                         // This is true cage-based deformation - preservation of shape 
                         // through correct transformation of rest offset
                         for (const auto &influence : influences)
                         {
                             const int sourceIndex = influence.first;
                             const float weight = influence.second;
                             
                             if (weight <= 0.0f || sourceIndex < 0 || 
                                 sourceIndex >= staticPos.size() || 
                                 sourceIndex >= sourceMatrices.size())
                                 continue;
                                 
                             // Get rest and deformed positions for this cage/influence point
                             const V3f &restInfluencePos = staticPos[sourceIndex];
                             const V3f &deformedInfluencePos = animatedPos[sourceIndex];
                             
                             // Calculate the original offset from influence point to the target point in rest pose
                             V3f restOffset = positions[i] - restInfluencePos;
                             
                             // Transform the offset into the local space defined by the influence point's local frame
                             V3f localRestOffset = sourceFrames[sourceIndex].toLocalSpace(restOffset);
                             
                             // Transform the local offset back to global space using the influence point's local frame
                             // This ensures the relative offset is properly preserved in the deformed space
                             V3f transformedOffset = sourceFrames[sourceIndex].toWorldSpace(localRestOffset);
                             
                             // Apply the transformed offset to the deformed influence position
                             V3f deformedPosition = deformedInfluencePos + transformedOffset;
                             
                             // Weight by the influence factor
                             newPosition += deformedPosition * weight;
                             totalWeight += weight;
                         }
                         
                         // Apply the weighted position
                         if (totalWeight > 0.0f)
                         {
                             positions[i] = newPosition / totalWeight;
                         }
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