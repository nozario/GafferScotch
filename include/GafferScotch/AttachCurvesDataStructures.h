#ifndef GAFFERSCOTCH_ATTACHCURVESDATASTRUCTURES_H
#define GAFFERSCOTCH_ATTACHCURVESDATASTRUCTURES_H

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
#include <shared_mutex>

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
            mutable Imath::V3f normal;
            mutable Imath::V3f tangent;
            mutable Imath::V3f bitangent;
            mutable Imath::V3f position;

            void buildFromMeshData(const struct MeshData &data, size_t index);

            // Ensure the frame is perfectly orthonormal
            void orthonormalize() const
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
            AlignedVector<size_t> vertexOffsets;    // Pre-calculated offsets
            AlignedVector<Imath::V3f> localOffsets; // Cached local offsets from root points
            AlignedVector<float> falloffValues;     // Cached falloff values

            // Thread-safe storage for rest space offsets
            struct ThreadSafeOffsets
            {
                mutable std::shared_mutex mutex;
                AlignedVector<Imath::V3f> offsets;

                void resize(size_t size)
                {
                    std::unique_lock<std::shared_mutex> lock(mutex);
                    offsets.resize(size);
                }

                const Imath::V3f &get(size_t index) const
                {
                    std::shared_lock<std::shared_mutex> lock(mutex);
                    return offsets[index];
                }

                void set(size_t index, const Imath::V3f &value)
                {
                    std::unique_lock<std::shared_mutex> lock(mutex);
                    offsets[index] = value;
                }
            };
            mutable ThreadSafeOffsets restSpaceOffsets;

            size_t totalVerts;

            CurveData();
            void initFromCurves(const IECoreScene::CurvesPrimitive *curves);
            void computeRestSpaceOffsets(const AlignedFrame &restFrame, size_t curveIndex) const;

            // Helper to safely access offsets
            const Imath::V3f &getRestSpaceOffset(size_t index) const
            {
                return restSpaceOffsets.get(index);
            }

            void setRestSpaceOffset(size_t index, const Imath::V3f &value) const
            {
                restSpaceOffsets.set(index, value);
            }
        };

        // Cached binding data for a single curve root point
        struct CurveBinding
        {
            mutable unsigned int triangleIndex; // Triangle index in the rest mesh
            mutable Imath::V3f baryCoords;      // Barycentric coordinates
            mutable Imath::V2f uvCoords;        // UV coordinates for faster lookup
            mutable AlignedFrame restFrame;     // Cached rest space frame
            mutable Imath::V3f rootPointOffset; // Offset from mesh surface to root point
            mutable bool valid;                 // Whether this binding is valid

            CurveBinding() : triangleIndex(0), valid(false) {}
        };

        // Collection of cached bindings and acceleration data
        struct BindingCache
        {
            // Thread-safe storage for bindings
            struct ThreadSafeBindings
            {
                mutable std::shared_mutex mutex;
                AlignedVector<CurveBinding> bindings;

                void resize(size_t size)
                {
                    std::unique_lock<std::shared_mutex> lock(mutex);
                    bindings.resize(size);
                }

                CurveBinding &get(size_t index)
                {
                    std::unique_lock<std::shared_mutex> lock(mutex);
                    return bindings[index];
                }

                const CurveBinding &get(size_t index) const
                {
                    std::shared_lock<std::shared_mutex> lock(mutex);
                    return bindings[index];
                }
            };

            ThreadSafeBindings bindings;
            IECore::MurmurHash restMeshHash; // Hash for rest mesh validation
            IECore::MurmurHash curvesHash;   // Hash for curves validation
            bool valid;                      // Whether the cache is valid

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
            const BindingCache *bindingCache; // Add reference to binding cache

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

#endif // GAFFERSCOTCH_ATTACHCURVESDATASTRUCTURES_H