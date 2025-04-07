#ifndef GAFFERSCOTCH_CURVESPOSTPROCESS_H
#define GAFFERSCOTCH_CURVESPOSTPROCESS_H

#include "GafferScotch/Export.h"
#include "GafferScotch/TypeIds.h"

#include "GafferScene/ObjectProcessor.h"

#include "Gaffer/NumericPlug.h"

#include "IECoreScene/CurvesPrimitive.h"

namespace GafferScotch
{

class GAFFERSCOTCH_API CurvesPostProcess : public GafferScene::ObjectProcessor
{
public:
    CurvesPostProcess(const std::string &name = defaultName<CurvesPostProcess>());
    ~CurvesPostProcess() override = default;

    IE_CORE_DECLARERUNTIMETYPEDEXTENSION(GafferScotch::CurvesPostProcess, GafferScotch::TypeId::CurvesPostProcessTypeId, GafferScene::ObjectProcessor);

    // Taubin Smoothing
    Gaffer::BoolPlug *enableTaubinSmoothingPlug();
    const Gaffer::BoolPlug *enableTaubinSmoothingPlug() const;

    // Taubin parameters
    Gaffer::FloatPlug *lambdaPlug();
    const Gaffer::FloatPlug *lambdaPlug() const;

    Gaffer::FloatPlug *muPlug();
    const Gaffer::FloatPlug *muPlug() const;

    Gaffer::IntPlug *iterationsPlug();
    const Gaffer::IntPlug *iterationsPlug() const;

    // End Points Fix
    Gaffer::BoolPlug *enableEndPointsFixPlug();
    const Gaffer::BoolPlug *enableEndPointsFixPlug() const;

    void affects(const Gaffer::Plug *input, AffectedPlugsContainer &outputs) const override;

protected:
    // Override ObjectProcessor methods
    bool affectsProcessedObject(const Gaffer::Plug *input) const override;
    void hashProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const override;
    IECore::ConstObjectPtr computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject) const override;

private:
    // Apply Taubin smoothing to CurvesPrimitive
    void applyTaubinSmoothing(
        IECoreScene::CurvesPrimitivePtr curves,
        float lambda,
        float mu,
        int iterations
    ) const;

    // Apply end points fix to CurvesPrimitive
    void applyEndPointsFix(IECoreScene::CurvesPrimitivePtr curves) const;

    static size_t g_firstPlugIndex;
};

IE_CORE_DECLAREPTR(CurvesPostProcess)

} // namespace GafferScotch

#endif // GAFFERSCOTCH_CURVESPOSTPROCESS_H 