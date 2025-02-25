#ifndef GAFFERSCOTCH_RIGIDDEFORMCURVES_H
#define GAFFERSCOTCH_RIGIDDEFORMCURVES_H

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
    namespace Detail
    {
        template <typename T>
        T primVar(const IECoreScene::PrimitiveVariable &pv, const float *barycentrics, unsigned int triangleIdx, const Imath::V3i &vertexIds);
    }

    class GAFFERSCOTCH_API RigidDeformCurves : public GafferScene::SceneElementProcessor
    {
    public:
        RigidDeformCurves(const std::string &name = defaultName<RigidDeformCurves>());
        ~RigidDeformCurves() override;

        IE_CORE_DECLARERUNTIMETYPEDEXTENSION(GafferScotch::RigidDeformCurves, GafferScotch::TypeId::RigidDeformCurvesTypeId, GafferScene::SceneElementProcessor);

        // Rest mesh input (for reference)
        GafferScene::ScenePlug *restMeshPlug();
        const GafferScene::ScenePlug *restMeshPlug() const;

        // Animated mesh input
        GafferScene::ScenePlug *animatedMeshPlug();
        const GafferScene::ScenePlug *animatedMeshPlug() const;

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
        // Helper to update rest cache if needed
        void updateRestCache(
            const IECoreScene::CurvesPrimitive *curves,
            const IECore::MurmurHash &curvesHash) const;

        // Helper to deform curves using stored binding data
        void deformCurves(
            const IECoreScene::CurvesPrimitive *curves,
            const IECoreScene::MeshPrimitive *restMesh,
            const IECoreScene::MeshPrimitive *animatedMesh,
            IECoreScene::CurvesPrimitive *outputCurves) const;

        // Helper to compute deformed frame
        void computeDeformedFrame(
            const Imath::V3f &restPosition,
            const Imath::V3f &restNormal,
            const Imath::V3f &restTangent,
            const Imath::V3f &restBitangent,
            const IECoreScene::MeshPrimitive *restMesh,
            const IECoreScene::MeshPrimitive *animatedMesh,
            Imath::V3f &outPosition,
            Imath::V3f &outNormal,
            Imath::V3f &outTangent,
            Imath::V3f &outBitangent) const;

        static size_t g_firstPlugIndex;
        mutable Detail::RestDataCache m_restCache;
    };

    IE_CORE_DECLAREPTR(RigidDeformCurves)

} // namespace GafferScotch

#endif // GAFFERSCOTCH_RIGIDDEFORMCURVES_H