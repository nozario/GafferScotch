#ifndef GAFFERSCOTCH_FEATHERDEFORMBARBS_H
#define GAFFERSCOTCH_FEATHERDEFORMBARBS_H

#include "GafferScotch/Export.h"
#include "GafferScotch/TypeIds.h"

#include "GafferScene/Deformer.h"
#include "GafferScene/ScenePlug.h"
#include "Gaffer/StringPlug.h"
#include "Gaffer/TypedPlug.h"

namespace IECoreScene
{
    class CurvesPrimitive;
}

namespace GafferScotch
{

    class GAFFERSCOTCH_API FeatherDeformBarbs : public GafferScene::Deformer
    {
    public:
        FeatherDeformBarbs(const std::string &name = staticTypeName());
        ~FeatherDeformBarbs() override;

        IE_CORE_DECLARERUNTIMETYPEDEXTENSION(GafferScotch::FeatherDeformBarbs, GafferScotch::TypeId::FeatherDeformBarbsTypeId, GafferScene::Deformer);

        GafferScene::ScenePlug *animatedShaftsPlug();
        const GafferScene::ScenePlug *animatedShaftsPlug() const;
        
        GafferScene::ScenePlug *restShaftsPlug();
        const GafferScene::ScenePlug *restShaftsPlug() const;

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

        Gaffer::BoolPlug *cleanupBindAttributesPlug();
        const Gaffer::BoolPlug *cleanupBindAttributesPlug() const;

        void affects(const Gaffer::Plug *input, AffectedPlugsContainer &outputs) const override;
        bool acceptsInput(const Gaffer::Plug *plug, const Gaffer::Plug *inputPlug) const override;

    protected:
        // This method is from the Deformer base class, which itself inherits it from ObjectProcessor.
        // It determines if the main geometric properties of the object are affected.
        bool affectsProcessedObject(const Gaffer::Plug *input) const override;
        void hashProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const override;
        IECore::ConstObjectPtr computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject) const override;

        // These methods are specific to the Deformer base class and handle bounding box computations.
        bool affectsProcessedObjectBound(const Gaffer::Plug *input) const override;
        void hashProcessedObjectBound(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const override;
        Imath::Box3f computeProcessedObjectBound(const ScenePath &path, const Gaffer::Context *context) const override;

    private:
        void deformBarbs(
            const IECoreScene::CurvesPrimitive *barbs,
            const IECoreScene::CurvesPrimitive *animatedShafts,
            const IECoreScene::CurvesPrimitive *restShafts,
            IECoreScene::CurvesPrimitive *outputBarbs,
            const std::string &hairIdAttrName,
            const std::string &shaftPointIdAttrName,
            const std::string &barbParamAttrName,
            const std::string &shaftUpVectorName,
            const std::string &orientAttrName) const;

        static size_t g_firstPlugIndex;
    };

    IE_CORE_DECLAREPTR(FeatherDeformBarbs);

}

#endif // GAFFERSCOTCH_FEATHERDEFORMBARBS_H