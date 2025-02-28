#ifndef GAFFERSCOTCH_RIGIDATTACHCURVES_H
#define GAFFERSCOTCH_RIGIDATTACHCURVES_H

#include "GafferScotch/Export.h"
#include "GafferScotch/TypeIds.h"
#include "GafferScotch/CurvesDataStructures.h"

#include "GafferScene/ObjectProcessor.h"
#include "GafferScene/ScenePlug.h"

#include "Gaffer/StringPlug.h"
#include "Gaffer/TypedPlug.h"

#include "IECoreScene/PrimitiveVariable.h"
#include "IECoreScene/MeshPrimitiveEvaluator.h"
#include "IECoreScene/CurvesPrimitive.h"

namespace GafferScotch
{

    class GAFFERSCOTCH_API RigidAttachCurves : public GafferScene::ObjectProcessor
    {
    public:
        RigidAttachCurves(const std::string &name = defaultName<RigidAttachCurves>());
        ~RigidAttachCurves() override;

        IE_CORE_DECLARERUNTIMETYPEDEXTENSION(GafferScotch::RigidAttachCurves, GafferScotch::TypeId::RigidAttachCurvesTypeId, GafferScene::ObjectProcessor);

        // Static mesh input
        GafferScene::ScenePlug *staticDeformerPlug();
        const GafferScene::ScenePlug *staticDeformerPlug() const;

        // Root finding
        Gaffer::StringPlug *curveRootAttrPlug();
        const Gaffer::StringPlug *curveRootAttrPlug() const;

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

        bool acceptsInput(const Gaffer::Plug *plug, const Gaffer::Plug *inputPlug) const override;

    private:
        // Cache for static data
        struct RestDataCache
        {
            IECore::MurmurHash staticMeshHash;
            IECore::MurmurHash curvesHash;
            bool valid;

            // Added members from Detail::RestDataCache
            Detail::CurveData curveData;
            Detail::MeshData staticMeshData;
            IECoreScene::PrimitiveVariable staticTangents;
            Detail::BindingCache bindingCache;

            RestDataCache() : valid(false) {}

            void invalidate()
            {
                valid = false;
                bindingCache.invalidate();
            }
        };

        // Helper to compute bindings and store as attributes
        void computeBindings(
            const IECoreScene::MeshPrimitive *staticMesh,
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