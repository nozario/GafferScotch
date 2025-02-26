#ifndef GAFFERSCOTCH_CAPTUREWEIGHT_H
#define GAFFERSCOTCH_CAPTUREWEIGHT_H

#include "GafferScotch/Export.h"
#include "GafferScotch/TypeIds.h"

#include "GafferScene/AttributeProcessor.h"
#include "GafferScene/ScenePlug.h"

#include "Gaffer/NumericPlug.h"
#include "Gaffer/StringPlug.h"

namespace GafferScotch
{

    class GAFFERSCOTCH_API CaptureWeight : public GafferScene::AttributeProcessor
    {
    public:
        CaptureWeight(const std::string &name = defaultName<CaptureWeight>());
        ~CaptureWeight() override = default;

        IE_CORE_DECLARERUNTIMETYPEDEXTENSION(GafferScotch::CaptureWeight, GafferScotch::TypeId::CaptureWeightTypeId, GafferScene::AttributeProcessor);

        // Source points input
        GafferScene::ScenePlug *sourcePlug();
        const GafferScene::ScenePlug *sourcePlug() const;

        // Source points path
        Gaffer::StringPlug *sourcePathPlug();
        const Gaffer::StringPlug *sourcePathPlug() const;

        // Parameters
        Gaffer::FloatPlug *radiusPlug();
        const Gaffer::FloatPlug *radiusPlug() const;

        Gaffer::IntPlug *maxPointsPlug();
        const Gaffer::IntPlug *maxPointsPlug() const;

        Gaffer::IntPlug *minPointsPlug();
        const Gaffer::IntPlug *minPointsPlug() const;

        Gaffer::StringPlug *pieceAttributePlug();
        const Gaffer::StringPlug *pieceAttributePlug() const;

        void affects(const Gaffer::Plug *input, Gaffer::DependencyNode::AffectedPlugsContainer &outputs) const override;

    protected:
        // Override AttributeProcessor methods
        bool affectsProcessedAttributes(const Gaffer::Plug *input) const override;
        void hashProcessedAttributes(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const override;
        IECore::ConstCompoundObjectPtr computeProcessedAttributes(const ScenePath &path, const Gaffer::Context *context, const IECore::CompoundObject *inputAttributes) const override;

        // We also need to process objects to add the capture attributes
        void hashObject(const ScenePath &path, const Gaffer::Context *context, const GafferScene::ScenePlug *parent, IECore::MurmurHash &h) const override;
        IECore::ConstObjectPtr computeObject(const ScenePath &path, const Gaffer::Context *context, const GafferScene::ScenePlug *parent) const override;

    private:
        // Helper method to compute capture weights for a primitive
        IECore::PrimitivePtr computeCaptureWeights(const IECoreScene::Primitive *inputPrimitive, const IECoreScene::Primitive *sourcePrimitive) const;

        static size_t g_firstPlugIndex;
    };

    IE_CORE_DECLAREPTR(CaptureWeight)

} // namespace GafferScotch

#endif // GAFFERSCOTCH_CAPTUREWEIGHT_H