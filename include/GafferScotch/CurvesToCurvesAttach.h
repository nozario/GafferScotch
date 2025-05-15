#ifndef GAFFERSCOTCH_CURVESTOCURVESATTACH_H
#define GAFFERSCOTCH_CURVESTOCURVESATTACH_H

#include "GafferScotch/Export.h"
#include "GafferScotch/TypeIds.h"

#include "GafferScene/ObjectProcessor.h"
#include "Gaffer/StringPlug.h"
#include "Gaffer/TypedPlug.h"

#include "IECoreScene/CurvesPrimitive.h"

namespace GafferScotch
{

class GAFFERSCOTCH_API CurvesToCurvesAttach : public GafferScene::ObjectProcessor
{

public:

    CurvesToCurvesAttach( const std::string &name = staticTypeName() );
    ~CurvesToCurvesAttach() override;

    IE_CORE_DECLARERUNTIMETYPEDEXTENSION( GafferScotch::CurvesToCurvesAttach, GafferScotch::TypeId::CurvesToCurvesAttachTypeId, GafferScene::ObjectProcessor );

    GafferScene::ScenePlug *parentDeformerPlug();
    const GafferScene::ScenePlug *parentDeformerPlug() const;

    Gaffer::StringPlug *curveRootAttrPlug();
    const Gaffer::StringPlug *curveRootAttrPlug() const;

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

    bool affectsProcessedObject( const Gaffer::Plug *input ) const override;
    void hashProcessedObject( const GafferScene::ScenePlug::ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h ) const override;
    IECore::ConstObjectPtr computeProcessedObject( const GafferScene::ScenePlug::ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject ) const override;

private:

    size_t findRootPointIndex( const IECoreScene::CurvesPrimitive *curves, const std::vector<size_t> &vertexOffsets, size_t curveIndex ) const;

    // Stores the binding data as primitive variables on the output curves
    void computeAndStoreBindings(
        const IECoreScene::CurvesPrimitive *parentDeformerCurves,
        const IECoreScene::CurvesPrimitive *childCurves,
        IECoreScene::CurvesPrimitive *outputCurves
    ) const;

    static size_t g_firstPlugIndex;

};

IE_CORE_DECLAREPTR( CurvesToCurvesAttach );

} // namespace GafferScotch

#endif // GAFFERSCOTCH_CURVESTOCURVESATTACH_H 