#ifndef GAFFERSCOTCH_FEATHERATTACHBARBS_H
#define GAFFERSCOTCH_FEATHERATTACHBARBS_H

#include "GafferScotch/Export.h"
#include "GafferScotch/TypeIds.h"

#include "GafferScene/ObjectProcessor.h"
#include "GafferScene/ScenePlug.h"
#include "Gaffer/StringPlug.h"
#include "IECoreScene/CurvesPrimitive.h"

namespace GafferScotch
{

    class GAFFERSCOTCH_API FeatherAttachBarbs : public GafferScene::ObjectProcessor
    {
    public:
        FeatherAttachBarbs(const std::string &name = staticTypeName());
        ~FeatherAttachBarbs() override;

        IE_CORE_DECLARERUNTIMETYPEDEXTENSION(GafferScotch::FeatherAttachBarbs, GafferScotch::TypeId::FeatherAttachBarbsTypeId, GafferScene::ObjectProcessor);

        GafferScene::ScenePlug *inShaftsPlug();
        const GafferScene::ScenePlug *inShaftsPlug() const;

        Gaffer::StringPlug *hairIdAttrNamePlug();
        const Gaffer::StringPlug *hairIdAttrNamePlug() const;

        Gaffer::StringPlug *curveParamAttrNamePlug();
        const Gaffer::StringPlug *curveParamAttrNamePlug() const;

        Gaffer::StringPlug *shaftPointOrientAttrNamePlug();
        const Gaffer::StringPlug *shaftPointOrientAttrNamePlug() const;

        void affects(const Gaffer::Plug *input, AffectedPlugsContainer &outputs) const override;
        bool acceptsInput(const Gaffer::Plug *plug, const Gaffer::Plug *inputPlug) const override;

    protected:
        bool affectsProcessedObject(const Gaffer::Plug *input) const override;
        void hashProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const override;
        IECore::ConstObjectPtr computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject) const override;

    private:
        void computeBindings(
            const IECoreScene::CurvesPrimitive *shafts,
            const IECoreScene::CurvesPrimitive *barbs,
            IECoreScene::CurvesPrimitive *outputBarbs,
            const IECore::IntVectorData *shaftHairIds,
            const IECore::IntVectorData *barbHairIds,
            const IECore::FloatVectorData *curveParams,
            const IECore::V3fVectorData *shaftPositions,
            const IECore::V3fVectorData *barbPositions,
            const IECore::QuatfVectorData *orientations) const;

        static size_t g_firstPlugIndex;
    };

    IE_CORE_DECLAREPTR(FeatherAttachBarbs);

} // namespace GafferScotch

#endif // GAFFERSCOTCH_FEATHERATTACHBARBS_H