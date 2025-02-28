#include "GafferScotch/AttachCurves.h"
#include "GafferScotch/ScenePathUtil.h"
#include "GafferScotch/AttachCurvesDataStructures.h"

#include "Gaffer/Context.h"
#include "Gaffer/StringPlug.h"

#include "GafferScene/SceneAlgo.h"
#include "GafferScene/ScenePlug.h"

#include "IECore/NullObject.h"
#include "IECore/StringAlgo.h"
#include "IECore/MessageHandler.h"

#include "IECoreScene/CurvesPrimitive.h"
#include "IECoreScene/MeshPrimitive.h"
#include "IECoreScene/MeshAlgo.h"
#include "IECoreScene/MeshPrimitiveEvaluator.h"
#include "IECoreScene/PrimitiveVariable.h"

#include "tbb/parallel_for.h"
#include "tbb/task_arena.h"

using namespace std;
using namespace Imath;
using namespace IECore;
using namespace IECoreScene;
using namespace Gaffer;
using namespace GafferScene;
using namespace GafferScotch;
using namespace GafferScotch::AttachCurvesDataStructures;

//////////////////////////////////////////////////////////////////////////
// AttachCurves implementation
//////////////////////////////////////////////////////////////////////////

IE_CORE_DEFINERUNTIMETYPED( AttachCurves );

size_t AttachCurves::g_firstPlugIndex = 0;

AttachCurves::AttachCurves( const std::string &name )
	:	Deformer( name )
{
	storeIndexOfNextChild( g_firstPlugIndex );
	ScenePlug *staticDeformer = new ScenePlug( "staticDeformer" );
	ScenePlug *animatedDeformer = new ScenePlug( "animatedDeformer" );
	StringPlug *deformerPath = new StringPlug( "deformerPath", Plug::In, "" );
	addChild( staticDeformer );
	addChild( animatedDeformer );
	addChild( deformerPath );
}

AttachCurves::~AttachCurves()
{
}

ScenePlug *AttachCurves::staticDeformerPlug()
{
	return this->template getChild<ScenePlug>( g_firstPlugIndex );
}

const ScenePlug *AttachCurves::staticDeformerPlug() const
{
	return this->template getChild<ScenePlug>( g_firstPlugIndex );
}

ScenePlug *AttachCurves::animatedDeformerPlug()
{
	return this->template getChild<ScenePlug>( g_firstPlugIndex + 1 );
}

const ScenePlug *AttachCurves::animatedDeformerPlug() const
{
	return this->template getChild<ScenePlug>( g_firstPlugIndex + 1 );
}

StringPlug *AttachCurves::deformerPathPlug()
{
	return this->template getChild<StringPlug>( g_firstPlugIndex + 2 );
}

const StringPlug *AttachCurves::deformerPathPlug() const
{
	return this->template getChild<StringPlug>( g_firstPlugIndex + 2 );
}

void AttachCurves::affects( const Plug *input, AffectedPlugsContainer &outputs ) const
{
	Deformer::affects( input, outputs );

	const ScenePlug *staticDeformer = staticDeformerPlug();
	const ScenePlug *animatedDeformer = animatedDeformerPlug();
	const StringPlug *deformerPath = deformerPathPlug();

	if(
		input == staticDeformer->objectPlug() ||
		input == animatedDeformer->objectPlug() ||
		input == deformerPath
	)
	{
		outputs.push_back( outPlug()->objectPlug() );
	}
}

bool AttachCurves::affectsProcessedObject( const Plug *input ) const
{
	return input == staticDeformerPlug()->objectPlug() ||
		   input == animatedDeformerPlug()->objectPlug() ||
		   input == deformerPathPlug();
}

void AttachCurves::hashProcessedObject( const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	Deformer::hashProcessedObject( path, context, h );
	
	// Hash the deformer path
	deformerPathPlug()->hash( h );
	
	// Get the deformer path
	std::string deformerPathStr = deformerPathPlug()->getValue();
	if( deformerPathStr.empty() )
	{
		return;
	}
	
	ScenePlug::ScenePath deformerPath = makeScenePath( deformerPathStr );
	
	// Hash the static deformer object
	{
		ScenePlug::PathScope pathScope( context, deformerPath );
		staticDeformerPlug()->objectPlug()->hash( h );
	}
	
	// Hash the animated deformer object
	{
		ScenePlug::PathScope pathScope( context, deformerPath );
		animatedDeformerPlug()->objectPlug()->hash( h );
	}
}

IECoreScene::ConstObjectPtr AttachCurves::computeProcessedObject( const ScenePath &path, const Gaffer::Context *context, const IECoreScene::Object *inputObject ) const
{
	// Check if the input is a curves primitive
	const CurvesPrimitive *curves = runTimeCast<const CurvesPrimitive>( inputObject );
	if( !curves )
	{
		return inputObject->copy();
	}
	
	// Get the deformer path
	std::string deformerPathStr = deformerPathPlug()->getValue();
	if( deformerPathStr.empty() )
	{
		return inputObject->copy();
	}
	
	ScenePlug::ScenePath deformerPath = makeScenePath( deformerPathStr );
	
	// Get the static deformer object
	ConstObjectPtr staticDeformerObject;
	{
		ScenePlug::PathScope pathScope( context, deformerPath );
		staticDeformerObject = staticDeformerPlug()->objectPlug()->getValue();
	}
	
	// Get the animated deformer object
	ConstObjectPtr animatedDeformerObject;
	{
		ScenePlug::PathScope pathScope( context, deformerPath );
		animatedDeformerObject = animatedDeformerPlug()->objectPlug()->getValue();
	}
	
	// Check if the deformer objects are valid meshes
	const MeshPrimitive *staticMesh = runTimeCast<const MeshPrimitive>( staticDeformerObject.get() );
	const MeshPrimitive *animatedMesh = runTimeCast<const MeshPrimitive>( animatedDeformerObject.get() );
	
	if( !staticMesh || !animatedMesh )
	{
		IECore::msg( IECore::MessageHandler::Warning, "AttachCurves", "Deformer objects are not valid meshes." );
		return inputObject->copy();
	}
	
	// Check if the meshes have the necessary UV coordinates
	if( !staticMesh->variables.count( "uv" ) )
	{
		IECore::msg( IECore::MessageHandler::Warning, "AttachCurves", "Static mesh does not have UV coordinates." );
		return inputObject->copy();
	}
	
	if( !animatedMesh->variables.count( "uv" ) )
	{
		IECore::msg( IECore::MessageHandler::Warning, "AttachCurves", "Animated mesh does not have UV coordinates." );
		return inputObject->copy();
	}
	
	// Check if the meshes have the necessary position data
	if( !staticMesh->variables.count( "P" ) || !animatedMesh->variables.count( "P" ) )
	{
		IECore::msg( IECore::MessageHandler::Warning, "AttachCurves", "Meshes do not have position data." );
		return inputObject->copy();
	}
	
	// Get the position data from the curves
	const V3fVectorData *curvePositions = curves->variableData<V3fVectorData>( "P" );
	if( !curvePositions )
	{
		IECore::msg( IECore::MessageHandler::Warning, "AttachCurves", "Curves do not have position data." );
		return inputObject->copy();
	}
	
	// Get the vertices per curve
	const IntVectorData *vertsPerCurve = curves->verticesPerCurve();
	const std::vector<int> &vertsPerCurveData = vertsPerCurve->readable();
	
	// Create a copy of the curves to modify
	CurvesPrimitivePtr result = curves->copy();
	V3fVectorDataPtr newPositions = result->variableData<V3fVectorData>( "P", PrimitiveVariable::Vertex )->copy();
	std::vector<V3f> &newPositionsData = newPositions->writable();
	
	// Compute vertex offsets for each curve
	IntVectorDataPtr vertexOffsetsData = computeVertexOffsets( curves );
	const std::vector<int> &vertexOffsets = vertexOffsetsData->readable();
	
	// Process curves in parallel
	tbb::task_arena arena( tbb::task_arena::automatic );
	arena.execute( [&]() {
		tbb::parallel_for( 
			tbb::blocked_range<size_t>( 0, vertsPerCurveData.size() ),
			[&]( const tbb::blocked_range<size_t> &r ) {
				// Create thread-local evaluator results
				MeshPrimitiveEvaluatorPtr staticEvaluator = new MeshPrimitiveEvaluator( staticMesh );
				MeshPrimitiveEvaluatorPtr animatedEvaluator = new MeshPrimitiveEvaluator( animatedMesh );
				PrimitiveEvaluator::ResultPtr staticResult = staticEvaluator->createResult();
				PrimitiveEvaluator::ResultPtr animatedResult = animatedEvaluator->createResult();
				
				for( size_t curveIndex = r.begin(); curveIndex < r.end(); ++curveIndex )
				{
					int numVerts = vertsPerCurveData[curveIndex];
					if( numVerts <= 0 )
					{
						continue;
					}
					
					int vertexOffset = vertexOffsets[curveIndex];
					
					// Get the root point of the curve
					const V3f &rootPoint = curvePositions->readable()[vertexOffset];
					
					// Find the closest point on the static mesh to the root point
					V2f uv;
					if( !findClosestPointUV( staticMesh, rootPoint, uv ) )
					{
						continue;
					}
					
					// Find the corresponding points on the static and animated meshes
					V3f staticPosition, staticNormal, staticTangentU, staticTangentV;
					V3f animatedPosition, animatedNormal, animatedTangentU, animatedTangentV;
					
					if( !findPointAtUV( staticMesh, uv, staticPosition, staticNormal, staticTangentU, staticTangentV ) )
					{
						continue;
					}
					
					if( !findPointAtUV( animatedMesh, uv, animatedPosition, animatedNormal, animatedTangentU, animatedTangentV ) )
					{
						continue;
					}
					
					// Create transformation matrices for the static and animated points
					M44f staticMatrix = createTransformMatrix( staticPosition, staticNormal, staticTangentU, staticTangentV );
					M44f animatedMatrix = createTransformMatrix( animatedPosition, animatedNormal, animatedTangentU, animatedTangentV );
					
					// Calculate the transformation from static to animated space
					M44f staticMatrixInverse;
					staticMatrix.inverse( staticMatrixInverse );
					M44f transformation = animatedMatrix * staticMatrixInverse;
					
					// Apply the transformation to all points of the curve
					transformCurve( transformation, curvePositions->readable(), newPositionsData, vertexOffset, numVerts );
				}
			}
		);
	});
	
	// Update the position primitive variable
	result->variables["P"] = PrimitiveVariable( PrimitiveVariable::Vertex, newPositions );
	
	return result;
}
