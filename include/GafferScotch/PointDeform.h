#ifndef GAFFERSCOTCH_POINTDEFORM_H
#define GAFFERSCOTCH_POINTDEFORM_H

#include "GafferScotch/Export.h"
#include "GafferScotch/TypeIds.h"

#include "GafferScene/SceneElementProcessor.h"
#include "GafferScene/ScenePlug.h"

#include "Gaffer/NumericPlug.h"
#include "Gaffer/StringPlug.h"

namespace GafferScotch
{

    class GAFFERSCOTCH_API PointDeform : public GafferScene::SceneElementProcessor
    {
    public:
        PointDeform(const std::string &name = defaultName<PointDeform>());
        ~PointDeform() = default;

        IE_CORE_DECLARERUNTIMETYPEDEXTENSION(GafferScotch::PointDeform, GafferScotch::TypeId::PointDeformTypeId, GafferScene::SceneElementProcessor);

        // Static deformer input (reference)
        GafferScene::ScenePlug *staticDeformerPlug();
        const GafferScene::ScenePlug *staticDeformerPlug() const;

        // Animated deformer input
        GafferScene::ScenePlug *animatedDeformerPlug();
        const GafferScene::ScenePlug *animatedDeformerPlug() const;

        // Deformer path
        Gaffer::StringPlug *deformerPathPlug();
        const Gaffer::StringPlug *deformerPathPlug() const;

        // Whether to cleanup capture attributes after deformation
        Gaffer::BoolPlug *cleanupAttributesPlug();
        const Gaffer::BoolPlug *cleanupAttributesPlug() const;

        void affects(const Gaffer::Plug *input, AffectedPlugsContainer &outputs) const override;

    protected:
        bool processesObject() const override;
        void hashProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const override;
        IECore::ConstObjectPtr computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::ConstObjectPtr inputObject) const override;

    private:
        static size_t g_firstPlugIndex;
    };

    IE_CORE_DECLAREPTR(PointDeform)

} // namespace GafferScotch

#endif // GAFFERSCOTCH_POINTDEFORM_H