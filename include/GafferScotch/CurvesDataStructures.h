#ifndef GAFFERSCOTCH_CURVESDATASTRUCTURES_H
#define GAFFERSCOTCH_CURVESDATASTRUCTURES_H

#include "IECore/VectorTypedData.h"
#include "IECore/TypedData.h"
#include "IECore/SimpleTypedData.h"
#include "IECore/GeometricTypedData.h"
#include "IECoreScene/CurvesPrimitive.h"
#include "IECoreScene/MeshPrimitive.h"
#include "IECoreScene/PrimitiveVariable.h"
#include "IECoreScene/MeshPrimitiveEvaluator.h"
#include "Imath/ImathVec.h"
#include "IECore/MurmurHash.h"

#include <tbb/cache_aligned_allocator.h>

namespace GafferScotch
{
    namespace Detail
    {
        using namespace IECore;
        using namespace Imath;

        // Helper function for primitive variable interpolation
        template <typename T>
        T primVar(const IECoreScene::PrimitiveVariable &pv, const float *barycentrics, unsigned int triangleIdx, const V3i &vertexIds);

        // Specialization declaration for V3f
        template <>
        V3f primVar<V3f>(const IECoreScene::PrimitiveVariable &pv, const float *barycentrics, unsigned int triangleIdx, const V3i &vertexIds);

        // Generic implementation for other types
        template <typename T>
        T primVar(const IECoreScene::PrimitiveVariable &pv, const float *barycentrics, unsigned int triangleIdx, const V3i &vertexIds)
        {
            typedef typename GeometricTypedData<T> DataType;

            if (pv.interpolation == IECoreScene::PrimitiveVariable::Constant)
            {
                const DataType *data = runTimeCast<const DataType>(pv.data.get());
                if (data)
                {
                    return data->readable();
                }
            }

            typename IECoreScene::PrimitiveVariable::IndexedView<T> view(pv);

            switch (pv.interpolation)
            {
            case IECoreScene::PrimitiveVariable::Constant:
                assert(view.size() == 1);
                return view[0];

            case IECoreScene::PrimitiveVariable::Uniform:
                assert(triangleIdx < view.size());
                return view[triangleIdx];

            case IECoreScene::PrimitiveVariable::Vertex:
            case IECoreScene::PrimitiveVariable::Varying:
                assert(vertexIds[0] < (int)view.size());
                assert(vertexIds[1] < (int)view.size());
                assert(vertexIds[2] < (int)view.size());
                return static_cast<T>(
                    view[vertexIds[0]] * barycentrics[0] +
                    view[vertexIds[1]] * barycentrics[1] +
                    view[vertexIds[2]] * barycentrics[2]);

            case IECoreScene::PrimitiveVariable::FaceVarying:
                assert((triangleIdx * 3) + 0 < view.size());
                assert((triangleIdx * 3) + 1 < view.size());
                assert((triangleIdx * 3) + 2 < view.size());
                return static_cast<T>(
                    view[(triangleIdx * 3) + 0] * barycentrics[0] +
                    view[(triangleIdx * 3) + 1] * barycentrics[1] +
                    view[(triangleIdx * 3) + 2] * barycentrics[2]);

            default:
                throw IECore::InvalidArgumentException("Unsupported primitive variable interpolation");
            }
        }

        template <typename T>
        using AlignedVector = std::vector<T, tbb::cache_aligned_allocator<T>>;

        // Frame cache for mesh points
        struct AlignedFrame
        {
            Imath::V3f normal;
            Imath::V3f tangent;
            Imath::V3f bitangent;
            Imath::V3f position;

            void buildFromMeshData(const struct MeshData &data, size_t index);

            // Ensure the frame is perfectly orthonormal
            void orthonormalize()
            {
                // Start with normal as primary direction
                normal.normalize();

                // Make tangent perpendicular to normal
                tangent = (tangent - (normal * tangent.dot(normal))).normalized();

                // Compute bitangent from normalized vectors
                bitangent = (normal % tangent).normalized();

                // No need to recompute tangent as it's already perpendicular to normal
            }
        };

        // Optimized mesh data storage
        struct MeshData
        {
            AlignedVector<Imath::V3f> positions;
            AlignedVector<Imath::V3f> normals;
            AlignedVector<Imath::V3f> tangents;
            AlignedVector<Imath::V3f> bitangents;

            void initFromMesh(const IECoreScene::MeshPrimitive *mesh,
                              const IECoreScene::PrimitiveVariable &normalsVar,
                              const IECoreScene::PrimitiveVariable &tangentsVar);
        };

        // Optimized curve data storage
        struct CurveData
        {
            AlignedVector<Imath::V3f> points;
            AlignedVector<int> vertsPerCurve;
            AlignedVector<size_t> vertexOffsets;                // Pre-calculated offsets
            AlignedVector<Imath::V3f> localOffsets;             // Cached local offsets from root points
            AlignedVector<float> falloffValues;                 // Cached falloff values
            mutable AlignedVector<Imath::V3f> restSpaceOffsets; // Cached rest space transformed offsets
            size_t totalVerts;

            CurveData();
            void initFromCurves(const IECoreScene::CurvesPrimitive *curves);
            void computeRestSpaceOffsets(const AlignedFrame &restFrame, size_t curveIndex) const;
        };

        // Cached binding data for a single curve root point
        struct CurveBinding
        {
            unsigned int triangleIndex; // Triangle index in the rest mesh
            Imath::V3f baryCoords;      // Barycentric coordinates
            Imath::V2f uvCoords;        // UV coordinates for faster lookup
            AlignedFrame restFrame;     // Cached rest space frame
            Imath::V3f rootPointOffset; // Offset from mesh surface to root point
            bool valid;                 // Whether this binding is valid

            CurveBinding() : triangleIndex(0), valid(false) {}
        };

        // Collection of cached bindings and acceleration data
        struct BindingCache
        {
            AlignedVector<CurveBinding> bindings; // Per-curve root point bindings
            IECore::MurmurHash restMeshHash;      // Hash for rest mesh validation
            IECore::MurmurHash curvesHash;        // Hash for curves validation
            bool valid;                           // Whether the cache is valid

            BindingCache() : valid(false) {}

            void initializeBindings(size_t numCurves)
            {
                bindings.resize(numCurves);
                valid = false;
            }

            void invalidate()
            {
                valid = false;
            }
        };

        // Thread-local computation cache
        struct ComputationCache
        {
            IECoreScene::MeshPrimitiveEvaluator::ResultPtr restResult;
            IECoreScene::MeshPrimitiveEvaluator::ResultPtr animResult;

            ComputationCache(IECoreScene::MeshPrimitiveEvaluator *restEval,
                             IECoreScene::MeshPrimitiveEvaluator *animEval);
        };

        // Structure to hold curve batch data for parallel processing
        struct CurveBatch
        {
            const CurveData *curves;
            std::vector<Imath::V3f> *outputPoints;
            IECoreScene::MeshPrimitiveEvaluator *restEvaluator;
            IECoreScene::MeshPrimitiveEvaluator *animatedEvaluator;
            const IECoreScene::PrimitiveVariable *restTangents;
            const IECoreScene::PrimitiveVariable *animatedTangents;

            void processCurve(size_t curveIndex, ComputationCache &cache) const;
        };

        struct RestDataCache
        {
            bool valid = false;
            IECore::MurmurHash curvesHash;
            IECore::MurmurHash restMeshHash;
            CurveData curveData;
            MeshData restMeshData;
            IECoreScene::PrimitiveVariable restTangents;
            BindingCache bindingCache;

            RestDataCache() : valid(false) {}

            void invalidate()
            {
                valid = false;
                bindingCache.invalidate();
            }
        };

    } // namespace Detail
} // namespace GafferScotch

#endif // GAFFERSCOTCH_CURVESDATASTRUCTURES_H