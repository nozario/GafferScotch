#ifndef GAFFERSCOTCH_CURVESTOCURVESDEFORM_H
#define GAFFERSCOTCH_CURVESTOCURVESDEFORM_H

#include "GafferScotch/Export.h"
#include "GafferScotch/TypeIds.h"

#include "GafferScene/Deformer.h"
#include "Gaffer/StringPlug.h"
#include "Gaffer/TypedPlug.h"

#include "IECoreScene/CurvesPrimitive.h"

namespace GafferScotch
{

class CurvesToCurvesDeform : public GafferScene::Deformer
{

public:

    CurvesToCurvesDeform( const std::string &name = staticTypeName() );
    ~CurvesToCurvesDeform() override;

    IE_CORE_DECLARERUNTIMETYPEDEXTENSION( GafferScotch::CurvesToCurvesDeform, GafferScotch::TypeId::CurvesToCurvesDeformTypeId, GafferScene::Deformer );

    GafferScene::ScenePlug *staticParentDeformerPlug();
    const GafferScene::ScenePlug *staticParentDeformerPlug() const;

    GafferScene::ScenePlug *animatedParentDeformerPlug();
    const GafferScene::ScenePlug *animatedParentDeformerPlug() const;

    Gaffer::BoolPlug *useBindAttrPlug();
    const Gaffer::BoolPlug *useBindAttrPlug() const;

    Gaffer::StringPlug *deformerPathPlug();
    const Gaffer::StringPlug *deformerPathPlug() const;

    Gaffer::StringPlug *bindAttrPlug();
    const Gaffer::StringPlug *bindAttrPlug() const;

    Gaffer::V3fPlug *upVectorPlug();
    const Gaffer::V3fPlug *upVectorPlug() const;

    Gaffer::BoolPlug *useUpVectorAttrPlug();
    const Gaffer::BoolPlug *useUpVectorAttrPlug() const;

    Gaffer::StringPlug *upVectorAttrPlug();
    const Gaffer::StringPlug *upVectorAttrPlug() const;
    
    Gaffer::BoolPlug *cleanupBindAttributesPlug();
    const Gaffer::BoolPlug *cleanupBindAttributesPlug() const;

    void affects( const Gaffer::Plug *input, AffectedPlugsContainer &outputs ) const override;
    bool affectsProcessedObject( const Gaffer::Plug *input ) const override;
    void hashProcessedObject( const GafferScene::ScenePlug::ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h ) const override;
    IECore::ConstObjectPtr computeProcessedObject( const GafferScene::ScenePlug::ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject ) const override;

    bool affectsProcessedObjectBound( const Gaffer::Plug *input ) const override;
    void hashProcessedObjectBound( const GafferScene::ScenePlug::ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h ) const override;
    Imath::Box3f computeProcessedObjectBound( const GafferScene::ScenePlug::ScenePath &path, const Gaffer::Context *context ) const override;

private:

    void deformChildCurves(
        const IECoreScene::CurvesPrimitive *inputChildCurves,
        const IECoreScene::CurvesPrimitive *staticParentDeformerCurves, // May be null if rest frame is directly on child
        const IECoreScene::CurvesPrimitive *animatedParentDeformerCurves,
        IECoreScene::CurvesPrimitive *outputChildCurves
    ) const;

    static size_t g_firstPlugIndex;

};

IE_CORE_DECLAREPTR( CurvesToCurvesDeform );

} // namespace GafferScotch

#endif // GAFFERSCOTCH_CURVESTOCURVESDEFORM_H 