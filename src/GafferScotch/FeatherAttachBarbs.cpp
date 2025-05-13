#include "GafferScotch/FeatherAttachBarbs.h"

#include "Gaffer/StringPlug.h"
#include "GafferScene/ScenePlug.h"
#include "GafferScene/ObjectProcessor.h"

#include "IECoreScene/CurvesPrimitive.h"
#include "IECore/TypedData.h"
#include "IECore/MessageHandler.h"
#include "IECore/DataAlgo.h"

#include "Imath/ImathMatrix.h"
#include "Imath/ImathVecAlgo.h"
#include "Imath/ImathQuat.h"

#include <map>

using namespace Gaffer;
using namespace GafferScene;
using namespace GafferScotch;
using namespace IECore;
using namespace IECoreScene;
using namespace Imath;

IE_CORE_DEFINERUNTIMETYPED( FeatherAttachBarbs );

size_t FeatherAttachBarbs::g_firstPlugIndex = 0;

FeatherAttachBarbs::FeatherAttachBarbs( const std::string &name )
    : ObjectProcessor( name )
{
    storeIndexOfNextChild( g_firstPlugIndex );

    addChild( new ScenePlug( "inShafts", Plug::In ) );
    // inBarbs is inPlug() from ObjectProcessor

    addChild( new StringPlug( "hairIdAttrName", Plug::In, "hair_id" ) );
    addChild( new StringPlug( "shaftPointIdAttrName", Plug::In, "shaft_pt" ) );
    addChild( new StringPlug( "barbParamAttrName", Plug::In, "curveu" ) );
    addChild( new StringPlug( "shaftUpVectorPrimVarName", Plug::In, "" ) );
    addChild( new StringPlug( "shaftPointOrientAttrName", Plug::In, "" ) );

    // Pass through attributes, transform, and bound by default
    outPlug()->attributesPlug()->setInput( inPlug()->attributesPlug() );
    outPlug()->transformPlug()->setInput( inPlug()->transformPlug() );
    outPlug()->boundPlug()->setInput( inPlug()->boundPlug() );
}

FeatherAttachBarbs::~FeatherAttachBarbs()
{
}

ScenePlug *FeatherAttachBarbs::inShaftsPlug()
{
    return getChild<ScenePlug>( g_firstPlugIndex );
}

const ScenePlug *FeatherAttachBarbs::inShaftsPlug() const
{
    return getChild<ScenePlug>( g_firstPlugIndex );
}

StringPlug *FeatherAttachBarbs::hairIdAttrNamePlug()
{
    return getChild<StringPlug>( g_firstPlugIndex + 1 );
}

const StringPlug *FeatherAttachBarbs::hairIdAttrNamePlug() const
{
    return getChild<StringPlug>( g_firstPlugIndex + 1 );
}

StringPlug *FeatherAttachBarbs::shaftPointIdAttrNamePlug()
{
    return getChild<StringPlug>( g_firstPlugIndex + 2 );
}

const StringPlug *FeatherAttachBarbs::shaftPointIdAttrNamePlug() const
{
    return getChild<StringPlug>( g_firstPlugIndex + 2 );
}

StringPlug *FeatherAttachBarbs::barbParamAttrNamePlug()
{
    return getChild<StringPlug>( g_firstPlugIndex + 3 );
}

const StringPlug *FeatherAttachBarbs::barbParamAttrNamePlug() const
{
    return getChild<StringPlug>( g_firstPlugIndex + 3 );
}

StringPlug *FeatherAttachBarbs::shaftUpVectorPrimVarNamePlug()
{
    return getChild<StringPlug>( g_firstPlugIndex + 4 );
}

const StringPlug *FeatherAttachBarbs::shaftUpVectorPrimVarNamePlug() const
{
    return getChild<StringPlug>( g_firstPlugIndex + 4 );
}

StringPlug *FeatherAttachBarbs::shaftPointOrientAttrNamePlug()
{
    return getChild<StringPlug>( g_firstPlugIndex + 5 );
}

const StringPlug *FeatherAttachBarbs::shaftPointOrientAttrNamePlug() const
{
    return getChild<StringPlug>( g_firstPlugIndex + 5 );
}


void FeatherAttachBarbs::affects( const Gaffer::Plug *input, AffectedPlugsContainer &outputs ) const
{
    ObjectProcessor::affects( input, outputs );

    if(
        input == inShaftsPlug()->objectPlug() ||
        input == hairIdAttrNamePlug() ||
        input == shaftPointIdAttrNamePlug() ||
        input == barbParamAttrNamePlug() ||
        input == shaftUpVectorPrimVarNamePlug() ||
        input == shaftPointOrientAttrNamePlug()
    )
    {
        outputs.push_back( outPlug()->objectPlug() );
    }
}

bool FeatherAttachBarbs::affectsProcessedObject( const Gaffer::Plug *input ) const
{
    return
        input == inShaftsPlug()->objectPlug() ||
        input == hairIdAttrNamePlug() ||
        input == shaftPointIdAttrNamePlug() ||
        input == barbParamAttrNamePlug() ||
        input == shaftUpVectorPrimVarNamePlug() ||
        input == shaftPointOrientAttrNamePlug();
}

void FeatherAttachBarbs::hashProcessedObject( const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
    ObjectProcessor::hashProcessedObject( path, context, h );
    inShaftsPlug()->objectPlug()->hash( h );
    // Input barbs are hashed by ObjectProcessor via inPlug()->objectPlug()->hash(h)
    hairIdAttrNamePlug()->hash( h );
    shaftPointIdAttrNamePlug()->hash( h );
    barbParamAttrNamePlug()->hash( h );
    shaftUpVectorPrimVarNamePlug()->hash( h );
    shaftPointOrientAttrNamePlug()->hash( h );
}

namespace
{

// Helper to get a V3f from a PrimitiveVariable, checking for Uniform and Vertex interpolation
// Returns false if not found or wrong type/interpolation for vertex data.
// For Uniform, assumes single value for the primitive.
bool getV3fPrimVarValue( const IECoreScene::PrimitiveVariable &pv, size_t primIndex, size_t vertIndex, V3f &outValue )
{
    if( pv.interpolation == PrimitiveVariable::Uniform )
    {
        if( const V3fData *data = runTimeCast<const V3fData>( pv.data.get() ) )
        {
            outValue = data->readable();
            return true;
        }
        else if( const V3fVectorData *data = runTimeCast<const V3fVectorData>( pv.data.get() ) )
        {
            if( !data->readable().empty() )
            {
                outValue = data->readable()[0]; // Assuming one value for Uniform, could also use primIndex if it makes sense for the data
                return true;
            }
        }
    }
    else if( pv.interpolation == PrimitiveVariable::Vertex || pv.interpolation == PrimitiveVariable::Varying )
    {
        if( const V3fVectorData *data = runTimeCast<const V3fVectorData>( pv.data.get() ) )
        {
            if( vertIndex < data->readable().size() )
            {
                outValue = data->readable()[vertIndex];
                return true;
            }
        }
    }
    return false;
}

// Helper to get a Quatf from a PrimitiveVariable (assumed V4f or V4fVectorData Uniform)
bool getQuatfPrimVarValue( const IECoreScene::PrimitiveVariable &pv, size_t primIndex, Quatf &outValue )
{
    if( pv.interpolation == PrimitiveVariable::Uniform )
    {
        if( const V4fData *data = runTimeCast<const V4fData>( pv.data.get() ) )
        {
            const V4f& q = data->readable();
            outValue = Quatf(q[3], q[0], q[1], q[2]); // Assuming xyzw stored, Quatf constructor is (r, i, j, k)
            return true;
        }
        else if( const V4fVectorData *data = runTimeCast<const V4fVectorData>( pv.data.get() ) )
        {
            if( !data->readable().empty() )
            {
                // For Uniform, take the value for the specific primitive if available, otherwise first.
                size_t indexToUse = (primIndex < data->readable().size()) ? primIndex : 0;
                const V4f& q = data->readable()[indexToUse];
                outValue = Quatf(q[3], q[0], q[1], q[2]);
                return true;
            }
        }
    }
    return false;
}


// Calculate Tangent, Bitangent, Normal for a point on a curve
void calculateCurveFrame( const IECore::V3fVectorData *pointsData, int pointIdx, int numShaftPoints,
                          const V3f *shaftUpVector, const Quatf *shaftOrientation,
                          V3f &T, V3f &B, V3f &N )
{
    const std::vector<V3f> &P_shaft = pointsData->readable();

    if (numShaftPoints == 1)
    {
        T = V3f(1, 0, 0);
    }
    else if (pointIdx < numShaftPoints - 1)
    {
        T = P_shaft[pointIdx + 1] - P_shaft[pointIdx];
    }
    else // pointIdx == numShaftPoints - 1
    {
        T = P_shaft[pointIdx] - P_shaft[pointIdx - 1];
    }
    T.normalize();

    if(shaftOrientation)
    {
        M44f orientMatrix = shaftOrientation->toMatrix44();
        T = V3f(1,0,0) * orientMatrix;
        B = V3f(0,1,0) * orientMatrix;
        N = V3f(0,0,1) * orientMatrix;
        T.normalize();
        B.normalize();
        N.normalize();
        return; // Orientation overrides other methods
    }

    if (shaftUpVector)
    {
        B = T.cross(*shaftUpVector);
        float blen = B.length();
        if (blen < 1e-6f)
        { // Tangent and shaftUpVector are parallel, fallback
            V3f ref_vec = V3f(0, 1, 0);
            if (std::abs(T.dot(ref_vec)) > 0.99f) ref_vec = V3f(1, 0, 0);
            B = T.cross(ref_vec);
        }
        B.normalize();
        N = B.cross(T);
        N.normalize();
    }
    else
    {
        V3f ref_vec = V3f(0, 1, 0);
        if (std::abs(T.dot(ref_vec)) > 0.99f) ref_vec = V3f(1, 0, 0);
        B = T.cross(ref_vec);
        B.normalize();
        N = B.cross(T);
        N.normalize();
    }
}

}

IECore::ConstObjectPtr FeatherAttachBarbs::computeProcessedObject( const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject )
{
    const CurvesPrimitive *inBarbsCurves = runTimeCast<const CurvesPrimitive>( inputObject );
    if( !inBarbsCurves )
    {
        return inputObject;
    }

    ConstObjectPtr inShaftsObject = inShaftsPlug()->objectPlug()->getValue();
    const CurvesPrimitive *inShaftsCurves = runTimeCast<const CurvesPrimitive>( inShaftsObject.get() );
    if( !inShaftsCurves )
    {
        msg( Msg::Warning, "FeatherAttachBarbs", "Input shafts are not CurvesPrimitive." );
        return inputObject;
    }

    // Get attribute names from plugs
    const std::string hairIdAttrName = hairIdAttrNamePlug()->getValue();
    const std::string shaftPointIdAttrName = shaftPointIdAttrNamePlug()->getValue();
    const std::string barbParamAttrName = barbParamAttrNamePlug()->getValue();
    const std::string shaftUpVectorPrimVarName = shaftUpVectorPrimVarNamePlug()->getValue();
    const std::string shaftPointOrientAttrName = shaftPointOrientAttrNamePlug()->getValue();

    // Validate presence of P on barbs and shafts
    auto barbPIt = inBarbsCurves->variables.find( "P" );
    if( barbPIt == inBarbsCurves->variables.end() || !runTimeCast<V3fVectorData>(barbPIt->second.data.get()) )
    {
        msg( Msg::Warning, "FeatherAttachBarbs", "Barbs missing 'P' (Position) Vertex attribute." );
        return inputObject;
    }
    const V3fVectorData *barbsP = runTimeCast<const V3fVectorData>(barbPIt->second.data.get());

    auto shaftPIt = inShaftsCurves->variables.find( "P" );
    if( shaftPIt == inShaftsCurves->variables.end() || !runTimeCast<V3fVectorData>(shaftPIt->second.data.get()) )
    {
        msg( Msg::Warning, "FeatherAttachBarbs", "Shafts missing 'P' (Position) Vertex attribute." );
        return inputObject;
    }
    const V3fVectorData *shaftsP = runTimeCast<const V3fVectorData>(shaftPIt->second.data.get());

    // --- Prepare shaft data map ---
    // Map: hair_id (int or string) -> { shaft_primitive_index, shaft_P_data, shaft_up_vector_data (optional), shaft_orient_data (optional) }
    // Using variant for hair_id type
    using HairIdVariant = std::variant<int, std::string>;
    struct ShaftInfo {
        int primitiveIndex;
        const V3fVectorData* pData;
        const V3fVectorData* upVectorData; // Could be per-primitive (Uniform) or per-vertex
        const V4fVectorData* orientationData; // Could be per-primitive (Uniform)
        const PrimitiveVariable* upVectorPrimVar;
        const PrimitiveVariable* orientationPrimVar;
    };
    std::map<HairIdVariant, ShaftInfo> shaftDataMap;

    const PrimitiveVariable* shaftsHairIdPV = findPrimitiveVariable( inShaftsCurves->variables, hairIdAttrName );
    if( !shaftsHairIdPV )
    {
        msg( Msg::Warning, "FeatherAttachBarbs", boost::format("Shafts missing '%1%' attribute.") % hairIdAttrName );
        return inputObject;
    }

    const IntVectorData* shaftsHairIdInt = runTimeCast<const IntVectorData>( shaftsHairIdPV->data.get() );
    const StringVectorData* shaftsHairIdString = runTimeCast<const StringVectorData>( shaftsHairIdPV->data.get() );

    if( !shaftsHairIdInt && !shaftsHairIdString )
    {
        msg( Msg::Warning, "FeatherAttachBarbs", boost::format("'%1%' attribute on shafts must be IntVectorData or StringVectorData.") % hairIdAttrName );
        return inputObject;
    }
    if( shaftsHairIdPV->interpolation != PrimitiveVariable::Uniform && shaftsHairIdPV->interpolation != PrimitiveVariable::Primitive )
    {
        msg( Msg::Warning, "FeatherAttachBarbs", boost::format("'%1%' attribute on shafts must have Uniform or Primitive interpolation.") % hairIdAttrName );
        return inputObject;
    }

    const PrimitiveVariable* shaftsUpVecPV = nullptr;
    if(!shaftUpVectorPrimVarName.empty())
        shaftsUpVecPV = findPrimitiveVariable(inShaftsCurves->variables, shaftUpVectorPrimVarName);
    
    const PrimitiveVariable* shaftsOrientPV = nullptr;
    if(!shaftPointOrientAttrName.empty())
        shaftsOrientPV = findPrimitiveVariable(inShaftsCurves->variables, shaftPointOrientAttrName);

    for( size_t i = 0; i < inShaftsCurves->numCurves(); ++i )
    {
        HairIdVariant id;
        if( shaftsHairIdInt )
        {
            id = shaftsHairIdInt->readable()[ (shaftsHairIdPV->interpolation == PrimitiveVariable::Uniform && !shaftsHairIdInt->readable().empty()) ? 0 : i ];
        }
        else // shaftsHairIdString must be valid
        {
            id = shaftsHairIdString->readable()[ (shaftsHairIdPV->interpolation == PrimitiveVariable::Uniform && !shaftsHairIdString->readable().empty()) ? 0 : i ];
        }
        shaftDataMap[id] = { (int)i, shaftsP, 
                             runTimeCast<const V3fVectorData>(shaftsUpVecPV ? shaftsUpVecPV->data.get() : nullptr),
                             runTimeCast<const V4fVectorData>(shaftsOrientPV ? shaftsOrientPV->data.get() : nullptr),
                             shaftsUpVecPV,
                             shaftsOrientPV
                           };
    }

    // --- Prepare barb attributes ---
    const PrimitiveVariable* barbsHairIdPV = findPrimitiveVariable( inBarbsCurves->variables, hairIdAttrName );
    const PrimitiveVariable* barbsShaftPointIdPV = findPrimitiveVariable( inBarbsCurves->variables, shaftPointIdAttrName );
    const PrimitiveVariable* barbsParamPV = findPrimitiveVariable( inBarbsCurves->variables, barbParamAttrName );

    if( !barbsHairIdPV || !barbsShaftPointIdPV || !barbsParamPV )
    {
        msg( Msg::Warning, "FeatherAttachBarbs", boost::format("Barbs missing one or more required attributes: '%1%', '%2%', '%3%'.") % hairIdAttrName % shaftPointIdAttrName % barbParamAttrName );
        return inputObject;
    }

    const IntVectorData* barbsHairIdInt = runTimeCast<const IntVectorData>( barbsHairIdPV->data.get() );
    const StringVectorData* barbsHairIdString = runTimeCast<const StringVectorData>( barbsHairIdPV->data.get() );
    const IntVectorData* barbsShaftPointId = runTimeCast<const IntVectorData>( barbsShaftPointIdPV->data.get() );
    const FloatVectorData* barbsParam = runTimeCast<const FloatVectorData>( barbsParamPV->data.get() );

    if( (!barbsHairIdInt && !barbsHairIdString) || !barbsShaftPointId || !barbsParam )
    {
        msg( Msg::Warning, "FeatherAttachBarbs", "Barb attributes have incorrect data types." );
        return inputObject;
    }
    if( (barbsHairIdPV->interpolation != PrimitiveVariable::Uniform && barbsHairIdPV->interpolation != PrimitiveVariable::Primitive) ||
        (barbsShaftPointIdPV->interpolation != PrimitiveVariable::Uniform && barbsShaftPointIdPV->interpolation != PrimitiveVariable::Primitive) ||
        barbsParamPV->interpolation != PrimitiveVariable::Vertex )
    {
        msg( Msg::Warning, "FeatherAttachBarbs", "Barb attributes have incorrect interpolation types." );
        return inputObject;
    }

    // --- Output attribute data ---
    // For bind_shaftHairId, we need to match the type of the input hairIdAttrName
    DataPtr bindShaftHairIdData;
    if (shaftsHairIdInt) bindShaftHairIdData = new IntVectorData;
    else bindShaftHairIdData = new StringVectorData;
    
    IntVectorDataPtr bindShaftPointIdData = new IntVectorData;
    V3fVectorDataPtr bindShaftRestPData = new V3fVectorData;
    V3fVectorDataPtr bindShaftRestTData = new V3fVectorData;
    V3fVectorDataPtr bindShaftRestBData = new V3fVectorData;
    V3fVectorDataPtr bindShaftRestNData = new V3fVectorData;
    V3fVectorDataPtr bindBarbRootOffsetLocalData = new V3fVectorData;

    std::vector<int>& bindShaftPointIdVec = bindShaftPointIdData->writable();
    std::vector<V3f>& bindShaftRestPVec = bindShaftRestPData->writable();
    std::vector<V3f>& bindShaftRestTVec = bindShaftRestTData->writable();
    std::vector<V3f>& bindShaftRestBVec = bindShaftRestBData->writable();
    std::vector<V3f>& bindShaftRestNVec = bindShaftRestNData->writable();
    std::vector<V3f>& bindBarbRootOffsetLocalVec = bindBarbRootOffsetLocalData->writable();

    const std::vector<int>& vertsPerBarb = inBarbsCurves->verticesPerCurve()->readable();
    size_t barbVertexOffset = 0;

    // --- Iterate through barb primitives ---
    for( size_t i = 0; i < inBarbsCurves->numCurves(); ++i )
    {
        HairIdVariant currentBarbHairId;
        size_t barbHairIdIndex = (barbsHairIdPV->interpolation == PrimitiveVariable::Uniform && ((barbsHairIdInt && !barbsHairIdInt->readable().empty()) || (barbsHairIdString && !barbsHairIdString->readable().empty()))) ? 0 : i;
        if( barbsHairIdInt )
        {
            if (barbHairIdIndex >= barbsHairIdInt->readable().size()) { 
                msg( Msg::Warning, "FeatherAttachBarbs", boost::format("Barb %1% hairId index out of bounds.") % i ); 
                barbVertexOffset += vertsPerBarb[i]; continue; 
            }
            currentBarbHairId = barbsHairIdInt->readable()[barbHairIdIndex];
        }
        else
        {
             if (barbHairIdIndex >= barbsHairIdString->readable().size()) { 
                msg( Msg::Warning, "FeatherAttachBarbs", boost::format("Barb %1% hairId index out of bounds.") % i ); 
                barbVertexOffset += vertsPerBarb[i]; continue; 
            }
            currentBarbHairId = barbsHairIdString->readable()[barbHairIdIndex];
        }

        size_t barbShaftPointIdIndex = (barbsShaftPointIdPV->interpolation == PrimitiveVariable::Uniform && !barbsShaftPointId->readable().empty()) ? 0 : i;
        if (barbShaftPointIdIndex >= barbsShaftPointId->readable().size()) { 
            msg( Msg::Warning, "FeatherAttachBarbs", boost::format("Barb %1% shaftPointId index out of bounds.") % i ); 
            barbVertexOffset += vertsPerBarb[i];
            continue; 
        }
        int targetShaftPtIdx = barbsShaftPointId->readable()[barbShaftPointIdIndex];

        auto shaftIt = shaftDataMap.find( currentBarbHairId );
        if( shaftIt == shaftDataMap.end() )
        {
            msg( Msg::Warning, "FeatherAttachBarbs", boost::format("Barb %1% with hairId cannot find matching shaft.") % i );
            barbVertexOffset += vertsPerBarb[i];
            continue;
        }
        const ShaftInfo &shaftInfo = shaftIt->second;
        const V3fVectorData* currentShaftPData = shaftInfo.pData;

        // Get shaft points for this specific shaft curve
        const std::vector<int>& vertsPerShaft = inShaftsCurves->verticesPerCurve()->readable();
        size_t shaftVertexStart = 0;
        for(int sIdx = 0; sIdx < shaftInfo.primitiveIndex; ++sIdx) shaftVertexStart += vertsPerShaft[sIdx];
        int numShaftPoints = vertsPerShaft[shaftInfo.primitiveIndex];

        if( targetShaftPtIdx < 0 || targetShaftPtIdx >= numShaftPoints )
        {
            msg( Msg::Warning, "FeatherAttachBarbs", boost::format("Barb %1% target_shaft_pt_idx %2% out of range for shaft with %3% points.") % i % targetShaftPtIdx % numShaftPoints );
            barbVertexOffset += vertsPerBarb[i];
            continue;
        }
        size_t actualShaftPointIndexInP = shaftVertexStart + targetShaftPtIdx;

        // Calculate Shaft Attachment Frame (Rest)
        V3f P_attach_shaft_rest = currentShaftPData->readable()[actualShaftPointIndexInP];
        V3f T_attach_shaft_rest, B_attach_shaft_rest, N_attach_shaft_rest;
        
        V3f shaftUpVecVal;
        const V3f* shaftUpVecPtr = nullptr;
        Quatf shaftOrientVal;
        const Quatf* shaftOrientPtr = nullptr;

        if(shaftInfo.upVectorPrimVar)
        {
            if(getV3fPrimVarValue(*shaftInfo.upVectorPrimVar, shaftInfo.primitiveIndex, actualShaftPointIndexInP, shaftUpVecVal))
            {
                 shaftUpVecPtr = &shaftUpVecVal;
            }
        }
        if(shaftInfo.orientationPrimVar)
        {
            if(getQuatfPrimVarValue(*shaftInfo.orientationPrimVar, shaftInfo.primitiveIndex, shaftOrientVal))
            {
                shaftOrientPtr = &shaftOrientVal;
            }
        }

        calculateCurveFrame( currentShaftPData, actualShaftPointIndexInP, 
                               currentShaftPData->readable().size(), // Pass total points, calculateCurveFrame will use actualShaftPointIndexInP to access points
                               shaftUpVecPtr, shaftOrientPtr, 
                               T_attach_shaft_rest, B_attach_shaft_rest, N_attach_shaft_rest );

        // Find barb root point
        size_t barbRootVertexIndex = barbVertexOffset;
        float minParamU = std::numeric_limits<float>::max();
        for( int v = 0; v < vertsPerBarb[i]; ++v )
        {
            size_t currentVertexGlobalIndex = barbVertexOffset + v;
            if (currentVertexGlobalIndex >= barbsParam->readable().size()) {
                 msg( Msg::Warning, "FeatherAttachBarbs", boost::format("Barb %1% vertex param index out of bounds.") % i ); 
                 // This barb is malformed, we should skip it, but need to advance offset
                 goto next_barb; 
            }
            float currentParamU = barbsParam->readable()[currentVertexGlobalIndex];
            if( currentParamU < minParamU )
            {
                minParamU = currentParamU;
                barbRootVertexIndex = currentVertexGlobalIndex;
            }
        }
        if (barbRootVertexIndex >= barbsP->readable().size()) {
            msg( Msg::Warning, "FeatherAttachBarbs", boost::format("Barb %1% root vertex index out of P bounds.") % i ); 
            goto next_barb; 
        }

        V3f P_barb_root_rest = barbsP->readable()[barbRootVertexIndex];

        // Construct shaft rest frame matrix
        M44f M_shaft_rest;
        M_shaft_rest.setValue( T_attach_shaft_rest.x, T_attach_shaft_rest.y, T_attach_shaft_rest.z, 0.0f,
                               B_attach_shaft_rest.x, B_attach_shaft_rest.y, B_attach_shaft_rest.z, 0.0f,
                               N_attach_shaft_rest.x, N_attach_shaft_rest.y, N_attach_shaft_rest.z, 0.0f,
                               P_attach_shaft_rest.x, P_attach_shaft_rest.y, P_attach_shaft_rest.z, 1.0f );

        V3f world_offset_vector = P_barb_root_rest - P_attach_shaft_rest;
        V3f bind_barbRootOffsetLocal = world_offset_vector * M_shaft_rest.inverse(); // Transform world-space offset vector into shaft's local frame.
                                                                                    // For direction vector, only rotational part of inverse needed.
                                                                                    // P_local = (P_world - Origin_world) * Rotation_world_to_local
        bind_barbRootOffsetLocal = world_offset_vector;
        M_shaft_rest.inverse().multDirMatrix(world_offset_vector, bind_barbRootOffsetLocal);

        // Store bind attributes
        if (shaftsHairIdInt)
        {
            std::get<std::vector<int>>(bindShaftHairIdData->writable()).push_back(std::get<int>(currentBarbHairId));
        }
        else
        {
            std::get<std::vector<std::string>>(bindShaftHairIdData->writable()).push_back(std::get<std::string>(currentBarbHairId));
        }
        bindShaftPointIdVec.push_back( targetShaftPtIdx );
        bindShaftRestPVec.push_back( P_attach_shaft_rest );
        bindShaftRestTVec.push_back( T_attach_shaft_rest );
        bindShaftRestBVec.push_back( B_attach_shaft_rest );
        bindShaftRestNVec.push_back( N_attach_shaft_rest );
        bindBarbRootOffsetLocalVec.push_back( bind_barbRootOffsetLocal );

    next_barb:
        barbVertexOffset += vertsPerBarb[i];
    }

    // Create output CurvesPrimitive
    CurvesPrimitivePtr outCurves = new CurvesPrimitive();
    outCurves->setTopology( inBarbsCurves->verticesPerCurve(), inBarbsCurves->basis(), inBarbsCurves->periodic() );

    // Copy existing primvars from inBarbs
    for( const auto &pvPair : inBarbsCurves->variables )
    {
        outCurves->variables[pvPair.first] = pvPair.second;
    }

    // Add new bind attributes
    outCurves->variables["bind_shaftHairId"] = PrimitiveVariable( PrimitiveVariable::Uniform, bindShaftHairIdData );
    outCurves->variables["bind_shaftPointId"] = PrimitiveVariable( PrimitiveVariable::Uniform, bindShaftPointIdData );
    outCurves->variables["bind_shaftRest_P"] = PrimitiveVariable( PrimitiveVariable::Uniform, bindShaftRestPData );
    outCurves->variables["bind_shaftRest_T"] = PrimitiveVariable( PrimitiveVariable::Uniform, bindShaftRestTData );
    outCurves->variables["bind_shaftRest_B"] = PrimitiveVariable( PrimitiveVariable::Uniform, bindShaftRestBData );
    outCurves->variables["bind_shaftRest_N"] = PrimitiveVariable( PrimitiveVariable::Uniform, bindShaftRestNData );
    outCurves->variables["bind_barbRootOffsetLocal"] = PrimitiveVariable( PrimitiveVariable::Uniform, bindBarbRootOffsetLocalData );

    return outCurves;
}

