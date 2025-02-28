#ifndef GAFFERSCOTCH_RIGIDDEFORMCURVES_H
#define GAFFERSCOTCH_RIGIDDEFORMCURVES_H

#include "GafferScotch/Export.h"
#include "GafferScotch/TypeIds.h"
#include "GafferScotch/CurvesDataStructures.h"

#include "GafferScene/Deformer.h"
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

    class GAFFERSCOTCH_API RigidDeformCurves : public GafferScene::Deformer
    {
    public:
        RigidDeformCurves(const std::string &name = defaultName<RigidDeformCurves>());
        ~RigidDeformCurves() override;

        IE_CORE_DECLARERUNTIMETYPEDEXTENSION(GafferScotch::RigidDeformCurves, GafferScotch::TypeId::RigidDeformCurvesTypeId, GafferScene::Deformer);

        // Static mesh input (for reference)
        GafferScene::ScenePlug *staticDeformerPlug();
        const GafferScene::ScenePlug *staticDeformerPlug() const;

        // Animated mesh input
        GafferScene::ScenePlug *animatedDeformerPlug();
        const GafferScene::ScenePlug *animatedDeformerPlug() const;

        // Binding mode
        Gaffer::BoolPlug *useBindAttrPlug();
        const Gaffer::BoolPlug *useBindAttrPlug() const;

        // Single mesh path
        Gaffer::StringPlug *deformerPathPlug();
        const Gaffer::StringPlug *deformerPathPlug() const;

        // Per-curve mesh path attribute
        Gaffer::StringPlug *bindAttrPlug();
        const Gaffer::StringPlug *bindAttrPlug() const;

        void affects(const Gaffer::Plug *input, AffectedPlugsContainer &outputs) const override;

    protected:
        // Override ObjectProcessor methods
        bool affectsProcessedObject(const Gaffer::Plug *input) const override;
        void hashProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const override;
        IECore::ConstObjectPtr computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject) const override;

        // Override for bounds computation
        bool affectsProcessedObjectBound(const Gaffer::Plug *input) const override;
        void hashProcessedObjectBound(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const override;
        Imath::Box3f computeProcessedObjectBound(const ScenePath &path, const Gaffer::Context *context) const override;

        bool acceptsInput(const Gaffer::Plug *plug, const Gaffer::Plug *inputPlug) const override;

    private:
        // Helper to deform curves using stored binding data
        void deformCurves(
            const IECoreScene::CurvesPrimitive *curves,
            const IECoreScene::MeshPrimitive *staticMesh,
            const IECoreScene::MeshPrimitive *animatedMesh,
            IECoreScene::CurvesPrimitive *outputCurves) const;

        // Helper to compute deformed frame
        void computeDeformedFrame(
            const Imath::V3f &staticPosition,
            const Imath::V3f &staticNormal,
            const Imath::V3f &staticTangent,
            const Imath::V3f &staticBitangent,
            const IECoreScene::MeshPrimitive *staticMesh,
            const IECoreScene::MeshPrimitive *animatedMesh,
            Imath::V3f &outPosition,
            Imath::V3f &outNormal,
            Imath::V3f &outTangent,
            Imath::V3f &outBitangent) const;

        static size_t g_firstPlugIndex;
    };

    IE_CORE_DECLAREPTR(RigidDeformCurves)

} // namespace GafferScotch

#endif // GAFFERSCOTCH_RIGIDDEFORMCURVES_H