#ifndef GAFFERSCOTCH_RIGIDATTACHCURVES_H
#define GAFFERSCOTCH_RIGIDATTACHCURVES_H

#include "GafferScotch/Export.h"
#include "GafferScotch/TypeIds.h"
#include "GafferScotch/AttachCurvesDataStructures.h"

#include "GafferScene/SceneElementProcessor.h"
#include "GafferScene/ScenePlug.h"

#include "Gaffer/StringPlug.h"
#include "Gaffer/TypedPlug.h"

#include "IECoreScene/PrimitiveVariable.h"
#include "IECoreScene/MeshPrimitiveEvaluator.h"
#include "IECoreScene/CurvesPrimitive.h"

namespace GafferScotch
{

    class GAFFERSCOTCH_API RigidAttachCurves : public GafferScene::SceneElementProcessor
    {
    public:
        RigidAttachCurves(const std::string &name = defaultName<RigidAttachCurves>());
        ~RigidAttachCurves() override;

        IE_CORE_DECLARERUNTIMETYPEDEXTENSION(GafferScotch::RigidAttachCurves, GafferScotch::TypeId::RigidAttachCurvesTypeId, GafferScene::SceneElementProcessor);

        // Rest mesh input
        GafferScene::ScenePlug *restMeshPlug();
        const GafferScene::ScenePlug *restMeshPlug() const;

        // Root finding
        Gaffer::StringPlug *rootAttrPlug();
        const Gaffer::StringPlug *rootAttrPlug() const;

        // Binding mode
        Gaffer::BoolPlug *useBindAttrPlug();
        const Gaffer::BoolPlug *useBindAttrPlug() const;

        // Single mesh path
        Gaffer::StringPlug *bindPathPlug();
        const Gaffer::StringPlug *bindPathPlug() const;

        // Per-curve mesh path attribute
        Gaffer::StringPlug *bindAttrPlug();
        const Gaffer::StringPlug *bindAttrPlug() const;

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
            IECore::MurmurHash restMeshHash;
            IECore::MurmurHash curvesHash;
            bool valid;

            // Added members from Detail::RestDataCache
            Detail::CurveData curveData;
            Detail::MeshData restMeshData;
            IECoreScene::PrimitiveVariable restTangents;
            Detail::BindingCache bindingCache;

            RestDataCache() : valid(false) {}

            void invalidate()
            {
                valid = false;
                bindingCache.invalidate();
            }
        };

        mutable RestDataCache m_restCache;

        // Helper to update rest cache if needed
        void updateRestCache(
            const IECoreScene::MeshPrimitive *restMesh,
            const IECoreScene::CurvesPrimitive *curves,
            const IECore::MurmurHash &restMeshHash,
            const IECore::MurmurHash &curvesHash) const;

        // Helper for spatial acceleration
        void buildSpatialIndex(
            const IECoreScene::MeshPrimitive *mesh,
            IECoreScene::MeshPrimitiveEvaluator *evaluator) const;

        // Helper to compute bindings and store as attributes
        void computeBindings(
            const IECoreScene::MeshPrimitive *restMesh,
            const IECoreScene::CurvesPrimitive *curves,
            IECoreScene::CurvesPrimitive *outputCurves) const;

        // Helper to find root point index for a curve
        size_t findRootPointIndex(
            const IECoreScene::CurvesPrimitive *curves,
            const std::vector<size_t> &vertexOffsets,
            size_t curveIndex) const;

        static size_t g_firstPlugIndex;
    };

    IE_CORE_DECLAREPTR(RigidAttachCurves)

} // namespace GafferScotch

#endif // GAFFERSCOTCH_RIGIDATTACHCURVES_H