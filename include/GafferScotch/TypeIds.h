#ifndef GAFFERSCOTCH_TYPEIDS_H
#define GAFFERSCOTCH_TYPEIDS_H

namespace GafferScotch
{

    enum class TypeId
    {
        // Make sure to claim a range with Cortex
        FirstTypeId = 121000,

        CaptureWeightTypeId = 121001,
        PointDeformTypeId = 121002,
        AttachCurvesTypeId = 121003,
        RigidAttachCurvesTypeId = 110703,
        RigidDeformCurvesTypeId = 110704,

        LastTypeId = 121499,
    };

} // namespace GafferScotch

#endif // GAFFERSCOTCH_TYPEIDS_H