#ifndef GAFFERSCOTCH_CAPTUREWEIGHT_H
#define GAFFERSCOTCH_CAPTUREWEIGHT_H

#include "GafferScotch/Export.h"
#include "GafferScotch/TypeIds.h"

#include "GafferScene/SceneElementProcessor.h"
#include "GafferScene/ScenePlug.h"

#include "Gaffer/NumericPlug.h"
#include "Gaffer/StringPlug.h"

namespace GafferScotch
{

    class GAFFERSCOTCH_API CaptureWeight : public GafferScene::SceneElementProcessor
    {
    public:
        CaptureWeight(const std::string &name = defaultName<CaptureWeight>());
        ~CaptureWeight() = default;

        IE_CORE_DECLARERUNTIMETYPEDEXTENSION(GafferScotch::CaptureWeight, GafferScotch::TypeId::CaptureWeightTypeId, GafferScene::SceneElementProcessor);

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
        // We only process objects (points/vertices)
        bool processesObject() const override;
        void hashProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const override;
        IECore::ConstObjectPtr computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::ConstObjectPtr inputObject) const override;

    private:
        static size_t g_firstPlugIndex;
    };

    IE_CORE_DECLAREPTR(CaptureWeight)

} // namespace GafferScotch

#endif // GAFFERSCOTCH_CAPTUREWEIGHT_H