#ifndef GAFFERSCOTCH_TYPEIDS_H
#define GAFFERSCOTCH_TYPEIDS_H

namespace GafferScotch
{

    enum class TypeId
    {
        // Make sure to claim a range with Cortex
        FirstTypeId = 121000,

        CaptureWeightsTypeId = 121001,
        PointDeformTypeId = 121002,
        RigidAttachCurvesTypeId = 110703,
        RigidDeformCurvesTypeId = 110704,
        CurvesPostProcessTypeId = 121005,
        CaptureMatrixWeightsTypeId = 121006,
        MatrixDeformTypeId = 121007,

        FeatherAttachBarbsTypeId = 121008,
        FeatherDeformBarbsTypeId = 121009,
        
        CurvesToCurvesAttachTypeId = 121010,
        CurvesToCurvesDeformTypeId = 121011,

        LastTypeId = 121499,
    };

} // namespace GafferScotch

#endif // GAFFERSCOTCH_TYPEIDS_H