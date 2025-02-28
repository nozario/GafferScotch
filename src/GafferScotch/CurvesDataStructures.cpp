#include "GafferScotch/CurvesDataStructures.h"

using namespace GafferScotch;
using namespace IECore;
using namespace IECoreScene;
using namespace Imath;

namespace GafferScotch
{
    namespace Detail
    {
        // Specialization implementation for V3f - used by both RigidAttachCurves and RigidDeformCurves
        template <>
        V3f primVar<V3f>(const PrimitiveVariable &pv, const float *barycentrics, unsigned int triangleIdx, const V3i &vertexIds)
        {
            typedef TypedData<V3f> DataType;

            if (pv.interpolation == PrimitiveVariable::Constant)
            {
                const DataType *data = runTimeCast<const DataType>(pv.data.get());
                if (data)
                {
                    return data->readable();
                }
            }

            typename PrimitiveVariable::IndexedView<V3f> view(pv);

            switch (pv.interpolation)
            {
            case PrimitiveVariable::Constant:
                assert(view.size() == 1);
                return view[0];

            case PrimitiveVariable::Uniform:
                assert(triangleIdx < view.size());
                return view[triangleIdx];

            case PrimitiveVariable::Vertex:
            case PrimitiveVariable::Varying:
                assert(vertexIds[0] < (int)view.size());
                assert(vertexIds[1] < (int)view.size());
                assert(vertexIds[2] < (int)view.size());
                return static_cast<V3f>(
                    view[vertexIds[0]] * barycentrics[0] +
                    view[vertexIds[1]] * barycentrics[1] +
                    view[vertexIds[2]] * barycentrics[2]);

            case PrimitiveVariable::FaceVarying:
                assert((triangleIdx * 3) + 0 < view.size());
                assert((triangleIdx * 3) + 1 < view.size());
                assert((triangleIdx * 3) + 2 < view.size());
                return static_cast<V3f>(
                    view[(triangleIdx * 3) + 0] * barycentrics[0] +
                    view[(triangleIdx * 3) + 1] * barycentrics[1] +
                    view[(triangleIdx * 3) + 2] * barycentrics[2]);

            default:
                throw InvalidArgumentException("Unsupported primitive variable interpolation");
            }
        }

        CurveData::CurveData() : totalVerts(0)
        {
        }

        void CurveData::initFromCurves(const CurvesPrimitive *curves)
        {
            if (!curves)
                return;

            const std::vector<int> &curveVertCounts = curves->verticesPerCurve()->readable();
            const V3fVectorData *curvePoints = curves->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);

            if (!curvePoints)
                return;

            // Copy vertex counts
            vertsPerCurve = AlignedVector<int>(curveVertCounts.begin(), curveVertCounts.end());

            // Pre-calculate vertex offsets
            vertexOffsets.resize(vertsPerCurve.size());
            size_t offset = 0;
            for (size_t i = 0; i < vertsPerCurve.size(); ++i)
            {
                vertexOffsets[i] = offset;
                offset += vertsPerCurve[i];
            }

            // Copy points with aligned storage
            const std::vector<V3f> &srcPoints = curvePoints->readable();
            points = AlignedVector<V3f>(srcPoints.begin(), srcPoints.end());

            // Calculate total verts
            totalVerts = points.size();

            // Pre-calculate local offsets and falloff values
            localOffsets.resize(totalVerts);
            falloffValues.resize(totalVerts);
            restSpaceOffsets.resize(totalVerts);

            for (size_t i = 0; i < vertsPerCurve.size(); ++i)
            {
                const size_t vertOffset = vertexOffsets[i];
                const int numVerts = vertsPerCurve[i];
                const V3f &rootP = points[vertOffset];

                // Calculate and store offsets and falloff values
                for (int j = 0; j < numVerts; ++j)
                {
                    const size_t idx = vertOffset + j;
                    localOffsets[idx] = points[idx] - rootP;

                    // Pre-calculate falloff
                    float distanceToRoot = localOffsets[idx].length();
                    falloffValues[idx] = std::exp(-distanceToRoot * 0.1f);
                }
            }
        }

        void CurveData::computeRestSpaceOffsets(const AlignedFrame &restFrame, size_t curveIndex) const
        {
            const size_t vertOffset = vertexOffsets[curveIndex];
            const int numVerts = vertsPerCurve[curveIndex];

            for (int i = 0; i < numVerts; ++i)
            {
                const size_t idx = vertOffset + i;
                const V3f &localOffset = localOffsets[idx];

                // Transform offset to rest space
                restSpaceOffsets[idx] = V3f(
                    localOffset.dot(restFrame.tangent),
                    localOffset.dot(restFrame.bitangent),
                    localOffset.dot(restFrame.normal));
            }
        }

        void MeshData::initFromMesh(const MeshPrimitive *mesh, const PrimitiveVariable &normalsVar,
                                    const PrimitiveVariable &tangentsVar)
        {
            if (!mesh)
                return;

            // Get position data
            const V3fVectorData *posData = mesh->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);
            if (!posData)
                return;
            positions = AlignedVector<V3f>(posData->readable().begin(), posData->readable().end());

            // Get normal data
            const V3fVectorData *normalData = runTimeCast<const V3fVectorData>(normalsVar.data.get());
            if (normalData)
            {
                normals = AlignedVector<V3f>(normalData->readable().begin(), normalData->readable().end());
            }

            // Get tangent data
            const V3fVectorData *tangentData = runTimeCast<const V3fVectorData>(tangentsVar.data.get());
            if (tangentData)
            {
                tangents = AlignedVector<V3f>(tangentData->readable().begin(), tangentData->readable().end());

                // Calculate bitangents
                bitangents.resize(tangents.size());
                for (size_t i = 0; i < tangents.size(); ++i)
                {
                    if (i < normals.size())
                    {
                        bitangents[i] = (normals[i] % tangents[i]).normalized();
                    }
                }
            }
        }

        // Basic frame structure used by both curve nodes
        void AlignedFrame::buildFromMeshData(const MeshData &data, size_t index)
        {
            if (index >= data.positions.size())
                return;

            position = data.positions[index];

            if (index < data.normals.size())
            {
                normal = data.normals[index].normalized();
            }

            if (index < data.tangents.size())
            {
                tangent = data.tangents[index].normalized();
            }

            if (index < data.bitangents.size())
            {
                bitangent = data.bitangents[index].normalized();
            }
            else if (normal.length() > 0 && tangent.length() > 0)
            {
                bitangent = (normal % tangent).normalized();
                tangent = (bitangent % normal).normalized();
            }
        }

        ComputationCache::ComputationCache(MeshPrimitiveEvaluator *restEval, MeshPrimitiveEvaluator *animEval)
        {
            if (restEval)
            {
                PrimitiveEvaluator::ResultPtr result = restEval->createResult();
                restResult = static_cast<MeshPrimitiveEvaluator::Result *>(result.get());
            }
            if (animEval)
            {
                PrimitiveEvaluator::ResultPtr result = animEval->createResult();
                animResult = static_cast<MeshPrimitiveEvaluator::Result *>(result.get());
            }
        }

        void CurveBatch::processCurve(size_t curveIndex, ComputationCache &cache) const
        {
            // Use pre-calculated offset
            const size_t vertOffset = curves->vertexOffsets[curveIndex];
            const int numVerts = curves->vertsPerCurve[curveIndex];
            const V3f rootP = curves->points[vertOffset];

            // Find closest point on rest mesh
            if (!restEvaluator->closestPoint(rootP, cache.restResult.get()))
            {
                // If no point found, copy original points unchanged
                for (int i = 0; i < numVerts; ++i)
                {
                    (*outputPoints)[vertOffset + i] = curves->points[vertOffset + i];
                }
                return;
            }

            // Get rest position and frame
            AlignedFrame restFrame;
            const MeshPrimitiveEvaluator::Result *restMeshResult = static_cast<const MeshPrimitiveEvaluator::Result *>(cache.restResult.get());
            const V3f &restBary = restMeshResult->barycentricCoordinates();
            const unsigned int restTriIdx = restMeshResult->triangleIndex();
            const Imath::V3i &restVertIds = restMeshResult->vertexIds();

            // Build rest frame
            restFrame.position = restMeshResult->point();
            restFrame.normal = restMeshResult->normal();
            restFrame.tangent = Detail::primVar<V3f>(*restTangents, &restBary[0], restTriIdx, restVertIds);
            restFrame.orthonormalize();

            // Compute rest space offsets for this curve
            curves->computeRestSpaceOffsets(restFrame, curveIndex);

            // Try multiple methods to find corresponding point on animated mesh
            bool foundAnimPoint = false;
            AlignedFrame animFrame;

            // Method 1: Try UV sampling first
            if (animatedEvaluator->pointAtUV(restMeshResult->uv(), cache.animResult.get()))
            {
                foundAnimPoint = true;
                const MeshPrimitiveEvaluator::Result *animMeshResult = static_cast<const MeshPrimitiveEvaluator::Result *>(cache.animResult.get());
                const V3f &animBary = animMeshResult->barycentricCoordinates();
                const unsigned int animTriIdx = animMeshResult->triangleIndex();
                const Imath::V3i &animVertIds = animMeshResult->vertexIds();

                animFrame.position = animMeshResult->point();
                animFrame.normal = animMeshResult->normal();
                animFrame.tangent = Detail::primVar<V3f>(*animatedTangents, &animBary[0], animTriIdx, animVertIds);
                animFrame.orthonormalize();
            }

            // Method 2: If UV sampling fails, try closest point to expected position
            if (!foundAnimPoint)
            {
                V3f expectedPos = restFrame.position + (cache.animResult->point() - restMeshResult->point());
                if (animatedEvaluator->closestPoint(expectedPos, cache.animResult.get()))
                {
                    foundAnimPoint = true;
                    const MeshPrimitiveEvaluator::Result *animMeshResult = static_cast<const MeshPrimitiveEvaluator::Result *>(cache.animResult.get());
                    const V3f &animBary = animMeshResult->barycentricCoordinates();
                    const unsigned int animTriIdx = animMeshResult->triangleIndex();
                    const Imath::V3i &animVertIds = animMeshResult->vertexIds();

                    animFrame.position = animMeshResult->point();
                    animFrame.normal = animMeshResult->normal();
                    animFrame.tangent = Detail::primVar<V3f>(*animatedTangents, &animBary[0], animTriIdx, animVertIds);
                    animFrame.orthonormalize();
                }
            }

            if (!foundAnimPoint)
            {
                // If all methods fail, just translate the curve
                V3f translation = cache.animResult->point() - restMeshResult->point();
                for (int i = 0; i < numVerts; ++i)
                {
                    (*outputPoints)[vertOffset + i] = curves->points[vertOffset + i] + translation;
                }
                return;
            }

            // Check if we need to flip the frame
            if (animFrame.normal.dot(restFrame.normal) < 0)
            {
                animFrame.normal = -animFrame.normal;
                animFrame.tangent = -animFrame.tangent;
                animFrame.bitangent = -animFrame.bitangent;
            }

            // Ensure tangent alignment between frames
            float tangentDot = animFrame.tangent.dot(restFrame.tangent);
            if (tangentDot < 0.99999f) // Allow for small numerical differences
            {
                // Project animated tangent onto the plane perpendicular to the animated normal
                V3f projectedRestTangent = restFrame.tangent - animFrame.normal * restFrame.tangent.dot(animFrame.normal);
                projectedRestTangent.normalize();

                // Rotate animated frame to align with projected rest tangent
                animFrame.tangent = projectedRestTangent;
                animFrame.bitangent = (animFrame.normal % animFrame.tangent).normalized();
            }

            // Transform each point in the curve
            for (int i = 0; i < numVerts; ++i)
            {
                const size_t pointIndex = vertOffset + i;

                // Use cached rest space offset
                const V3f &restSpaceOffset = curves->restSpaceOffsets[pointIndex];

                // Transform to animated space
                V3f animatedOffset =
                    restSpaceOffset.x * animFrame.tangent +
                    restSpaceOffset.y * animFrame.bitangent +
                    restSpaceOffset.z * animFrame.normal;

                // Use cached falloff value
                float falloff = curves->falloffValues[pointIndex];

                // Simple translation for far points
                V3f translation = animFrame.position - restFrame.position;
                V3f translatedPoint = curves->points[pointIndex] + translation;

                // Blend between deformed and translated positions
                (*outputPoints)[pointIndex] = animFrame.position + animatedOffset * falloff +
                                              (translatedPoint - (animFrame.position + animatedOffset)) * (1.0f - falloff);
            }
        }

    } // namespace Detail
} // namespace GafferScotch
