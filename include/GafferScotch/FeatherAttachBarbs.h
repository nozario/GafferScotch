#ifndef GAFFERSCOTCH_FEATHERATTACHBARBS_H
#define GAFFERSCOTCH_FEATHERATTACHBARBS_H

#include "GafferScotch/Export.h"
#include "GafferScotch/TypeIds.h"

#include "GafferScene/ObjectProcessor.h"
#include "GafferScene/ScenePlug.h"
#include "Gaffer/StringPlug.h"

namespace GafferScotch
{

    class FeatherAttachBarbs : public GafferScene::ObjectProcessor
    {
    public:
        FeatherAttachBarbs(const std::string &name = staticTypeName());
        ~FeatherAttachBarbs() override;

        IE_CORE_DECLARERUNTIMETYPEDEXTENSION(GafferScotch::FeatherAttachBarbs, FeatherAttachBarbsTypeId, GafferScene::ObjectProcessor);

        GafferScene::ScenePlug *inShaftsPlug();
        const GafferScene::ScenePlug *inShaftsPlug() const;

        GafferScene::ScenePlug *inBarbsPlug();
        const GafferScene::ScenePlug *inBarbsPlug() const;

        Gaffer::StringPlug *hairIdAttrNamePlug();
        const Gaffer::StringPlug *hairIdAttrNamePlug() const;

        Gaffer::StringPlug *shaftPointIdAttrNamePlug();
        const Gaffer::StringPlug *shaftPointIdAttrNamePlug() const;

        Gaffer::StringPlug *barbParamAttrNamePlug();
        const Gaffer::StringPlug *barbParamAttrNamePlug() const;

        Gaffer::StringPlug *shaftUpVectorPrimVarNamePlug();
        const Gaffer::StringPlug *shaftUpVectorPrimVarNamePlug() const;

        Gaffer::StringPlug *shaftPointOrientAttrNamePlug();
        const Gaffer::StringPlug *shaftPointOrientAttrNamePlug() const;

    protected:
        bool affectsProcessedObject(const Gaffer::Plug *input) const override;
        void hashProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const override;
        IECore::ConstObjectPtr computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject) const override;

    private:
        static size_t g_firstPlugIndex;
    };

    IE_CORE_DECLAREPTR(FeatherAttachBarbs);

} // namespace GafferScotch

#endif // GAFFERSCOTCH_FEATHERATTACHBARBS_H