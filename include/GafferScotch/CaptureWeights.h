#ifndef GAFFERSCOTCH_CAPTUREWEIGHTS_H
#define GAFFERSCOTCH_CAPTUREWEIGHTS_H

#include "GafferScotch/Export.h"
#include "GafferScotch/TypeIds.h"

#include "GafferScene/ObjectProcessor.h"
#include "GafferScene/ScenePlug.h"

#include "Gaffer/NumericPlug.h"
#include "Gaffer/StringPlug.h"

#include "IECoreScene/Primitive.h"
#include "IECore/KDTree.h"

namespace GafferScotch
{

    /// \class CaptureWeights
    /// A node that computes capture weights for deformation.
    ///
    /// This node computes weights for each vertex in the input mesh based on
    /// its proximity to vertices in a static deformer mesh. These weights can
    /// then be used by the PointDeform node to deform the mesh.

    class GAFFERSCOTCH_API CaptureWeights : public GafferScene::ObjectProcessor
    {
    public:
        CaptureWeights(const std::string &name = defaultName<CaptureWeights>());
        ~CaptureWeights() override = default;

        IE_CORE_DECLARERUNTIMETYPEDEXTENSION(GafferScotch::CaptureWeights, GafferScotch::TypeId::CaptureWeightsTypeId, GafferScene::ObjectProcessor);

        // Static deformer input
        GafferScene::ScenePlug *staticDeformerPlug();
        const GafferScene::ScenePlug *staticDeformerPlug() const;

        // Deformer path
        Gaffer::StringPlug *deformerPathPlug();
        const Gaffer::StringPlug *deformerPathPlug() const;

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
        // Override ObjectProcessor methods
        bool affectsProcessedObject(const Gaffer::Plug *input) const override;
        void hashProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const override;
        IECore::ConstObjectPtr computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject) const override;

    private:
        // Helper methods for efficient hashing
        void hashPositions(const IECoreScene::Primitive *primitive, IECore::MurmurHash &h) const;
        void hashPieceAttribute(const IECoreScene::Primitive *primitive, const std::string &attrName, IECore::MurmurHash &h) const;

        static size_t g_firstPlugIndex;
    };

    IE_CORE_DECLAREPTR(CaptureWeights)

} // namespace GafferScotch

#endif // GAFFERSCOTCH_CAPTUREWEIGHTS_H