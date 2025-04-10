#ifndef GAFFERSCOTCH_MATRIXDEFORM_H
#define GAFFERSCOTCH_MATRIXDEFORM_H

#include "GafferScotch/Export.h"
#include "GafferScotch/TypeIds.h"

#include "GafferScene/Deformer.h"
#include "GafferScene/ScenePlug.h"

#include "Gaffer/NumericPlug.h"
#include "Gaffer/StringPlug.h"

namespace GafferScotch
{

    class GAFFERSCOTCH_API MatrixDeform : public GafferScene::Deformer
    {
    public:
        MatrixDeform(const std::string &name = defaultName<MatrixDeform>());
        ~MatrixDeform() override = default;

        IE_CORE_DECLARERUNTIMETYPEDEXTENSION(GafferScotch::MatrixDeform, GafferScotch::TypeId::MatrixDeformTypeId, GafferScene::Deformer);

        // Static deformer input (reference)
        GafferScene::ScenePlug *staticDeformerPlug();
        const GafferScene::ScenePlug *staticDeformerPlug() const;

        // Animated deformer input
        GafferScene::ScenePlug *animatedDeformerPlug();
        const GafferScene::ScenePlug *animatedDeformerPlug() const;

        // Deformer path
        Gaffer::StringPlug *deformerPathPlug();
        const Gaffer::StringPlug *deformerPathPlug() const;

        // Whether to apply rigid projection for more stable deformation
        Gaffer::BoolPlug *rigidProjectionPlug();
        const Gaffer::BoolPlug *rigidProjectionPlug() const;

        // Whether to cleanup capture attributes after deformation
        Gaffer::BoolPlug *cleanupAttributesPlug();
        const Gaffer::BoolPlug *cleanupAttributesPlug() const;

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

    private:
        static size_t g_firstPlugIndex;
    };

    IE_CORE_DECLAREPTR(MatrixDeform)

} // namespace GafferScotch

#endif // GAFFERSCOTCH_MATRIXDEFORM_H 