#ifndef GAFFERSCOTCH_ATTACHCURVES_H
#define GAFFERSCOTCH_ATTACHCURVES_H

#include "GafferScotch/Export.h"
#include "GafferScotch/TypeIds.h"

#include "Gaffer/ComputeNode.h"
#include "Gaffer/StringPlug.h"

#include "GafferScene/ScenePlug.h"
#include "GafferScene/Deformer.h"

#include "IECore/MurmurHash.h"
#include "IECore/VectorTypedData.h"
#include "IECore/Object.h"

#include "IECoreScene/CurvesPrimitive.h"
#include "IECoreScene/MeshPrimitive.h"
#include "IECoreScene/PrimitiveVariable.h"

namespace GafferScotch
{

/// The AttachCurves node provides a quick rigid transform binding of curves to a geometry.
/// It takes three inputs: the input curves, a static deformer (for binding), and an animated
/// deformer (for deforming the curves). The node uses the curve's root (first point) as a
/// referential for the transformation, taking into account rotation and translation.
class GAFFERSCOTCH_API AttachCurves : public GafferScene::Deformer
{
	public :

		AttachCurves( const std::string &name=defaultName<AttachCurves>() );
		~AttachCurves() override;

		GAFFER_NODE_DECLARE_TYPE( GafferScotch::AttachCurves, static_cast<int>(TypeId::AttachCurvesTypeId), GafferScene::Deformer );

		/// The static deformer input (for binding the curves)
		GafferScene::ScenePlug *staticDeformerPlug();
		const GafferScene::ScenePlug *staticDeformerPlug() const;

		/// The animated deformer input (to rigidly deform the curves)
		GafferScene::ScenePlug *animatedDeformerPlug();
		const GafferScene::ScenePlug *animatedDeformerPlug() const;

		/// The path to the deformer object within the deformer inputs
		Gaffer::StringPlug *deformerPathPlug();
		const Gaffer::StringPlug *deformerPathPlug() const;

		void affects( const Gaffer::Plug *input, AffectedPlugsContainer &outputs ) const override;

	protected :

		bool affectsProcessedObject( const Gaffer::Plug *input ) const override;
		void hashProcessedObject( const GafferScene::ScenePlug::ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h ) const override;
		IECore::ConstObjectPtr computeProcessedObject( const GafferScene::ScenePlug::ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject ) const override;

	private :

		static size_t g_firstPlugIndex;

};

IE_CORE_DECLAREPTR( AttachCurves )

} // namespace GafferScotch

#endif // GAFFERSCOTCH_ATTACHCURVES_H
