#ifndef GAFFERSCOTCH_CAPTUREMATRIXWEIGHTS_H
#define GAFFERSCOTCH_CAPTUREMATRIXWEIGHTS_H

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

    /// \class CaptureMatrixWeights
    /// A node that computes matrix-based capture weights for advanced deformation.
    ///
    /// This node extends the functionality of CaptureWeights by also computing
    /// local transformation matrices for each influence point, enabling more
    /// natural deformations when influence points are outside the target mesh.

    class GAFFERSCOTCH_API CaptureMatrixWeights : public GafferScene::ObjectProcessor
    {
    public:
        CaptureMatrixWeights(const std::string &name = defaultName<CaptureMatrixWeights>());
        ~CaptureMatrixWeights() override = default;

        IE_CORE_DECLARERUNTIMETYPEDEXTENSION(GafferScotch::CaptureMatrixWeights, GafferScotch::TypeId::CaptureMatrixWeightsTypeId, GafferScene::ObjectProcessor);

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

        Gaffer::IntPlug *neighborPointsPlug();
        const Gaffer::IntPlug *neighborPointsPlug() const;

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

    IE_CORE_DECLAREPTR(CaptureMatrixWeights)

} // namespace GafferScotch

#endif // GAFFERSCOTCH_CAPTUREMATRIXWEIGHTS_H 