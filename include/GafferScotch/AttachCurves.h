#ifndef GAFFERSCOTCH_ATTACHCURVES_H
#define GAFFERSCOTCH_ATTACHCURVES_H

#include "GafferScotch/Export.h"
#include "GafferScotch/TypeIds.h"
#include "GafferScotch/AttachCurvesDataStructures.h"

#include "GafferScene/SceneElementProcessor.h"
#include "GafferScene/ScenePlug.h"

#include "Gaffer/StringPlug.h"
#include "Gaffer/TypedPlug.h"

#include "IECoreScene/PrimitiveVariable.h"

namespace GafferScotch
{
    // Forward declarations
    namespace Detail
    {
        template <typename T>
        using AlignedVector = std::vector<T, tbb::cache_aligned_allocator<T>>;

        struct CurveData;
        struct MeshData;
        struct AlignedFrame;
        struct ComputationCache;
        struct CurveBatch;
    }

    class GAFFERSCOTCH_API AttachCurves : public GafferScene::SceneElementProcessor
    {
    public:
        AttachCurves(const std::string &name = defaultName<AttachCurves>());
        ~AttachCurves() override;

        IE_CORE_DECLARERUNTIMETYPEDEXTENSION(GafferScotch::AttachCurves, GafferScotch::TypeId::AttachCurvesTypeId, GafferScene::SceneElementProcessor);

        // Source meshes
        GafferScene::ScenePlug *restMeshPlug();
        const GafferScene::ScenePlug *restMeshPlug() const;

        GafferScene::ScenePlug *animatedMeshPlug();
        const GafferScene::ScenePlug *animatedMeshPlug() const;

        // Root point finding
        Gaffer::StringPlug *rootAttributeNamePlug();
        const Gaffer::StringPlug *rootAttributeNamePlug() const;

        // Binding mode
        Gaffer::BoolPlug *useBindRootAttributePlug();
        const Gaffer::BoolPlug *useBindRootAttributePlug() const;

        Gaffer::StringPlug *rootPathPlug();
        const Gaffer::StringPlug *rootPathPlug() const;

        Gaffer::StringPlug *bindRootAttributePlug();
        const Gaffer::StringPlug *bindRootAttributePlug() const;

        void affects(const Gaffer::Plug *input, AffectedPlugsContainer &outputs) const override;

    protected:
        bool processesObject() const override;
        void hashProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const override;
        IECore::ConstObjectPtr computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::ConstObjectPtr inputObject) const override;

        bool acceptsInput(const Gaffer::Plug *plug, const Gaffer::Plug *inputPlug) const override;

    private:
        // Cache for rest data
        struct RestDataCache
        {
            Detail::MeshData restMeshData;
            Detail::CurveData curveData;
            Detail::BindingCache bindingCache;
            IECoreScene::PrimitiveVariable restTangents;
            IECore::MurmurHash restMeshHash;
            IECore::MurmurHash curvesHash;
            bool valid;

            RestDataCache() : valid(false) {}
        };

        mutable RestDataCache m_restCache;

        // Helper to update rest cache if needed
        void updateRestCache(
            const IECoreScene::MeshPrimitive *restMesh,
            const IECoreScene::CurvesPrimitive *curves,
            const IECore::MurmurHash &restMeshHash,
            const IECore::MurmurHash &curvesHash) const;

        // New helper methods for two-phase computation
        void computeBindings(
            const IECoreScene::MeshPrimitive *restMesh,
            const IECoreScene::CurvesPrimitive *curves) const;

        void applyDeformation(
            const IECoreScene::MeshPrimitive *animatedMesh,
            const IECoreScene::CurvesPrimitive *curves,
            std::vector<Imath::V3f> &outputPoints) const;

        // Helper for spatial acceleration
        void buildSpatialIndex(
            const IECoreScene::MeshPrimitive *mesh,
            IECoreScene::MeshPrimitiveEvaluator *evaluator) const;

        void alignCoordinateSystem(
            const Imath::V3f &refPosition, const Imath::V3f &refNormal,
            const Imath::V3f &targetPosition, const Imath::V3f &targetNormal,
            Imath::M44f &matrix) const;

        static size_t g_firstPlugIndex;
    };

    IE_CORE_DECLAREPTR(AttachCurves)

} // namespace GafferScotch

#endif // GAFFERSCOTCH_ATTACHCURVES_H