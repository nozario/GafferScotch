#include <boost/python.hpp>

#include "GafferBindings/DependencyNodeBinding.h"

#include "GafferScotch/CaptureWeight.h"
#include "GafferScotch/PointDeform.h"
#include "GafferScotch/AttachCurves.h"
#include "GafferScotch/RigidAttachCurves.h"
#include "GafferScotch/RigidDeformCurves.h"

using namespace boost::python;
using namespace GafferBindings;
using namespace GafferScotch;

BOOST_PYTHON_MODULE(GafferScotchModule)
{
    GafferBindings::DependencyNodeClass<AttachCurves>();
    GafferBindings::DependencyNodeClass<CaptureWeight>();
    GafferBindings::DependencyNodeClass<PointDeform>();
    GafferBindings::DependencyNodeClass<RigidAttachCurves>();
    GafferBindings::DependencyNodeClass<RigidDeformCurves>();
}