#include "GafferScotch/FeatherDeformBarbs.h"

#include "Gaffer/StringPlug.h"
#include "Gaffer/TypedPlug.h"
#include "GafferScene/ScenePlug.h"

#include "IECoreScene/CurvesPrimitive.h"
#include "IECore/TypedData.h"
#include "IECore/MessageHandler.h"
#include "IECore/DataAlgo.h"

#include "Imath/ImathMatrix.h"
#include "Imath/ImathVecAlgo.h"
#include "Imath/ImathQuat.h"

#include <map>
#include <variant>

using namespace Gaffer;
using namespace GafferScene;
using namespace GafferScotch;
using namespace IECore;
using namespace IECoreScene;
using namespace Imath;

IE_CORE_DEFINERUNTIMETYPED( FeatherDeformBarbs );

size_t FeatherDeformBarbs::g_firstPlugIndex = 0;

FeatherDeformBarbs::FeatherDeformBarbs( const std::string &name )
    : Deformer( name )
{
    storeIndexOfNextChild( g_firstPlugIndex );

    addChild( new ScenePlug( "animatedShafts", Plug::In ) );
    // inBarbs is inPlug() from Deformer

    addChild( new StringPlug( "hairIdAttrName", Plug::In, "hair_id" ) );
    addChild( new StringPlug( "shaftUpVectorPrimVarName", Plug::In, "" ) );
    addChild( new StringPlug( "shaftPointOrientAttrName", Plug::In, "" ) );
    addChild( new BoolPlug( "cleanupBindAttributes", Plug::In, true ) );

    // Pass through attributes, transform, and bound by default
    // Deformer base class handles this if we don't override affectsProcessedObject()
    // but it's good to be explicit if we want to ensure these passthroughs.
    outPlug()->attributesPlug()->setInput(inPlug()->attributesPlug());
    outPlug()->transformPlug()->setInput(inPlug()->transformPlug());
    outPlug()->boundPlug()->setInput(inPlug()->boundPlug());
}

FeatherDeformBarbs::~FeatherDeformBarbs()
{
}

ScenePlug *FeatherDeformBarbs::animatedShaftsPlug()
{
    return getChild<ScenePlug>( g_firstPlugIndex );
}

const ScenePlug *FeatherDeformBarbs::animatedShaftsPlug() const
{
    return getChild<ScenePlug>( g_firstPlugIndex );
}

StringPlug *FeatherDeformBarbs::hairIdAttrNamePlug()
{
    return getChild<StringPlug>( g_firstPlugIndex + 1 );
}

const StringPlug *FeatherDeformBarbs::hairIdAttrNamePlug() const
{
    return getChild<StringPlug>( g_firstPlugIndex + 1 );
}

StringPlug *FeatherDeformBarbs::shaftUpVectorPrimVarNamePlug()
{
    return getChild<StringPlug>( g_firstPlugIndex + 2 );
}

const StringPlug *FeatherDeformBarbs::shaftUpVectorPrimVarNamePlug() const
{
    return getChild<StringPlug>( g_firstPlugIndex + 2 );
}

StringPlug *FeatherDeformBarbs::shaftPointOrientAttrNamePlug()
{
    return getChild<StringPlug>( g_firstPlugIndex + 3 );
}

const StringPlug *FeatherDeformBarbs::shaftPointOrientAttrNamePlug() const
{
    return getChild<StringPlug>( g_firstPlugIndex + 3 );
}

BoolPlug *FeatherDeformBarbs::cleanupBindAttributesPlug()
{
    return getChild<BoolPlug>( g_firstPlugIndex + 4 );
}

const BoolPlug *FeatherDeformBarbs::cleanupBindAttributesPlug() const
{
    return getChild<BoolPlug>( g_firstPlugIndex + 4 );
}

void FeatherDeformBarbs::affects( const Gaffer::Plug *input, AffectedPlugsContainer &outputs ) const
{
    Deformer::affects( input, outputs );

    if(
        input == animatedShaftsPlug()->objectPlug() ||
        input == hairIdAttrNamePlug() ||
        input == shaftUpVectorPrimVarNamePlug() ||
        input == shaftPointOrientAttrNamePlug() ||
        input == cleanupBindAttributesPlug()
      )
    {
        outputs.push_back( outPlug()->objectPlug() );
    }
}

bool FeatherDeformBarbs::affectsProcessedObject( const Gaffer::Plug *input ) const
{
    // Note: Deformer::affectsProcessedObject checks inPlug connection.
    // We only need to add our specific plugs.
    return Deformer::affectsProcessedObject(input) ||
           input == animatedShaftsPlug()->objectPlug() ||
           input == hairIdAttrNamePlug() ||
           input == shaftUpVectorPrimVarNamePlug() ||
           input == shaftPointOrientAttrNamePlug() ||
           input == cleanupBindAttributesPlug();
}

void FeatherDeformBarbs::hashProcessedObject( const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
    Deformer::hashProcessedObject( path, context, h );
    animatedShaftsPlug()->objectPlug()->hash( h );
    // Input barbs (inPlug) are hashed by Deformer base class
    hairIdAttrNamePlug()->hash( h );
    shaftUpVectorPrimVarNamePlug()->hash( h );
    shaftPointOrientAttrNamePlug()->hash( h );
    cleanupBindAttributesPlug()->hash( h );

    // Hash relevant bind attributes on the input object if they exist
    // This ensures that if the bind data changes, we recompute.
    ConstObjectPtr inputObject = inPlug()->objectPlug()->getValue();
    const CurvesPrimitive *curves = runTimeCast<const CurvesPrimitive>( inputObject.get() );
    if( curves )
    {
        for( const auto &attrName : { "bind_shaftHairId", "bind_shaftPointId", "bind_shaftRest_P", 
                                       "bind_shaftRest_T", "bind_shaftRest_B", "bind_shaftRest_N", 
                                       "bind_barbRootOffsetLocal" } )
        {
            auto it = curves->variables.find( attrName );
            if( it != curves->variables.end() && it->second.data )
            {
                it->second.data->hash( h );
            }
        }
    }
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
                // For Uniform, take the value for the specific primitive if available, otherwise first.
                size_t indexToUse = (primIndex < data->readable().size()) ? primIndex : 0;
                outValue = data->readable()[indexToUse];
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

void calculateCurveFrame( const IECore::V3fVectorData *pointsData, int pointIdxOnCurve, int numPointsOnThisCurve, size_t globalPointStartIndex,
                          const V3f *shaftUpVector, const Quatf* shaftOrientation, 
                          V3f &T, V3f &B, V3f &N )
{
    const std::vector<V3f> &P_shaft_all = pointsData->readable();

    if (numPointsOnThisCurve == 1)
    {
        T = V3f(1, 0, 0);
    }
    else if (pointIdxOnCurve < numPointsOnThisCurve - 1)
    {
        T = P_shaft_all[globalPointStartIndex + pointIdxOnCurve + 1] - P_shaft_all[globalPointStartIndex + pointIdxOnCurve];
    }
    else // pointIdxOnCurve == numPointsOnThisCurve - 1
    {
        T = P_shaft_all[globalPointStartIndex + pointIdxOnCurve] - P_shaft_all[globalPointStartIndex + pointIdxOnCurve - 1];
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

// Templated getter for bind attributes to handle different HairId types
// Since hairId is now always int, this template might be overkill or simplified.
// For now, we'll keep the structure but expect TIdType to always be int when used.
template<typename TIdType> // Effectively, TIdType will be int
struct BindAttributeReaders
{
    const TypedData<std::vector<TIdType>> *bindShaftHairIdReader = nullptr;
    const IntVectorData *bindShaftPointIdReader = nullptr;
    const V3fVectorData *bindShaftRestPReader = nullptr;
    const V3fVectorData *bindShaftRestTReader = nullptr;
    const V3fVectorData *bindShaftRestBReader = nullptr;
    const V3fVectorData *bindShaftRestNReader = nullptr;
    const V3fVectorData *bindBarbRootOffsetLocalReader = nullptr;
    const FloatVectorData *barbParamReader = nullptr; // Added for root finding

    BindAttributeReaders(const CurvesPrimitive *curves, const std::string& barbParamAttrName)
    {
        auto findAndCast = [&](const std::string &name) -> auto {
            auto it = curves->variables.find(name);
            if (it == curves->variables.end() || !it->second.data) return static_cast<decltype(runTimeCast<TypedData<std::vector<TIdType>>>(it->second.data.get()))>(nullptr);
            return runTimeCast<TypedData<std::vector<TIdType>>>(it->second.data.get());
        };
        auto findAndCastInt = [&](const std::string &name) -> const IntVectorData* {
            auto it = curves->variables.find(name);
            if (it == curves->variables.end() || !it->second.data) return nullptr;
            return runTimeCast<const IntVectorData>(it->second.data.get());
        };
        auto findAndCastV3f = [&](const std::string &name) -> const V3fVectorData* {
            auto it = curves->variables.find(name);
            if (it == curves->variables.end() || !it->second.data) return nullptr;
            return runTimeCast<const V3fVectorData>(it->second.data.get());
        };
         auto findAndCastFloat = [&](const std::string &name) -> const FloatVectorData* {
            auto it = curves->variables.find(name);
            if (it == curves->variables.end() || !it->second.data) return nullptr;
            return runTimeCast<const FloatVectorData>(it->second.data.get());
        };

        bindShaftHairIdReader = findAndCast("bind_shaftHairId");
        bindShaftPointIdReader = findAndCastInt("bind_shaftPointId");
        bindShaftRestPReader = findAndCastV3f("bind_shaftRest_P");
        bindShaftRestTReader = findAndCastV3f("bind_shaftRest_T");
        bindShaftRestBReader = findAndCastV3f("bind_shaftRest_B");
        bindShaftRestNReader = findAndCastV3f("bind_shaftRest_N");
        bindBarbRootOffsetLocalReader = findAndCastV3f("bind_barbRootOffsetLocal");
        
        auto barbParamIt = curves->variables.find(barbParamAttrName);
        if(barbParamIt != curves->variables.end() && barbParamIt->second.interpolation == PrimitiveVariable::Vertex)
        {
            barbParamReader = runTimeCast<const FloatVectorData>(barbParamIt->second.data.get());
        }
    }

    bool isValid() const
    {
        return bindShaftHairIdReader && bindShaftPointIdReader && bindShaftRestPReader && 
               bindShaftRestTReader && bindShaftRestBReader && bindShaftRestNReader && 
               bindBarbRootOffsetLocalReader && barbParamReader;
    }
};

}

IECore::ConstObjectPtr FeatherDeformBarbs::computeProcessedObject( const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject )
{
    const CurvesPrimitive *inBarbsCurves = runTimeCast<const CurvesPrimitive>( inputObject );
    if( !inBarbsCurves )
    {
        return inputObject;
    }

    ConstObjectPtr animatedShaftsObject = animatedShaftsPlug()->objectPlug()->getValue();
    const CurvesPrimitive *animatedShaftsCurves = runTimeCast<const CurvesPrimitive>( animatedShaftsObject.get() );
    if( !animatedShaftsCurves )
    {
        msg( Msg::Warning, "FeatherDeformBarbs", "Animated shafts are not CurvesPrimitive." );
        return inputObject;
    }

    const std::string hairIdAttrName = hairIdAttrNamePlug()->getValue();
    const std::string shaftUpVectorPrimVarName = shaftUpVectorPrimVarNamePlug()->getValue();
    const std::string shaftPointOrientAttrName = shaftPointOrientAttrNamePlug()->getValue();
    const std::string barbParamAttrNameFromAttach = "curveu"; // Default used in Attach, should ideally come from there or a shared plug.
                                                             // For now, assume it was "curveu" during attach.

    // --- Validate P on input barbs and animated shafts ---
    auto barbPIt = inBarbsCurves->variables.find( "P" );
    if( barbPIt == inBarbsCurves->variables.end() || !runTimeCast<V3fVectorData>(barbPIt->second.data.get()) )
    {
        msg( Msg::Warning, "FeatherDeformBarbs", "Input barbs missing 'P' (Position) Vertex attribute." );
        return inputObject;
    }
    const V3fVectorData *inBarbsP = runTimeCast<const V3fVectorData>(barbPIt->second.data.get());

    auto animShaftPIt = animatedShaftsCurves->variables.find( "P" );
    if( animShaftPIt == animatedShaftsCurves->variables.end() || !runTimeCast<V3fVectorData>(animShaftPIt->second.data.get()) )
    {
        msg( Msg::Warning, "FeatherDeformBarbs", "Animated shafts missing 'P' (Position) Vertex attribute." );
        return inputObject;
    }
    const V3fVectorData *animShaftsP = runTimeCast<const V3fVectorData>(animShaftPIt->second.data.get());

    // --- Read bind attributes from inBarbs --- 
    // Determine HairId type from the bind attribute itself
    auto bindShaftHairIdIt = inBarbsCurves->variables.find("bind_shaftHairId");
    if(bindShaftHairIdIt == inBarbsCurves->variables.end() || !bindShaftHairIdIt->second.data)
    {
        msg( Msg::Warning, "FeatherDeformBarbs", "Input barbs missing 'bind_shaftHairId' attribute (expected IntVectorData)." );
        return inputObject;
    }

    // bool idsAreInt = runTimeCast<IntVectorData>(bindShaftHairIdIt->second.data.get()) != nullptr; // REMOVE THIS, should always be true
    // The following smart pointer and void* for readers might be simplified if we only have one type of reader.
    // For now, let's assume the structure to minimize large scale changes, but it will always take the "idsAreInt" path.
    std::unique_ptr<BindAttributeReaders<int>> readers(new BindAttributeReaders<int>(inBarbsCurves, barbParamAttrNameFromAttach));
    if(!readers->isValid()) { 
        msg( Msg::Warning, "FeatherDeformBarbs", "Input barbs missing one or more required bind_* attributes (expecting Int ID type)."); 
        return inputObject;
    }
    // The old else branch for BindAttributeReaders<std::string> is removed.

    // --- Prepare animated shaft data map ---
    // using HairIdVariant = std::variant<int, std::string>; // REMOVE
    struct AnimShaftInfo {
        int primitiveIndex;
        const V3fVectorData* pData;
        const PrimitiveVariable* upVectorPrimVar;
        const PrimitiveVariable* orientationPrimVar;
    };
    // std::map<HairIdVariant, AnimShaftInfo> animShaftDataMap; // REPLACE
    std::map<int, AnimShaftInfo> animShaftDataMap; // WITH THIS

    const PrimitiveVariable* animShaftsHairIdPV = nullptr;
    auto animShaftsHairIdIt = animatedShaftsCurves->variables.find( hairIdAttrName );
    if( animShaftsHairIdIt != animatedShaftsCurves->variables.end() )
    {
        animShaftsHairIdPV = &(animShaftsHairIdIt->second);
    }

    if( !animShaftsHairIdPV )
    {
        msg( Msg::Warning, "FeatherDeformBarbs", boost::format("Animated shafts missing '%1%' attribute.") % hairIdAttrName );
        return inputObject;
    }
    const IntVectorData* animShaftsHairIdInt = runTimeCast<const IntVectorData>( animShaftsHairIdPV->data.get() );
    // const StringVectorData* animShaftsHairIdString = runTimeCast<const StringVectorData>( animShaftsHairIdPV->data.get() ); // REMOVE
    // if( (!animShaftsHairIdInt && !animShaftsHairIdString) ||  // REPLACE
    if( !animShaftsHairIdInt || 
        (animShaftsHairIdPV->interpolation != PrimitiveVariable::Uniform) )
    {
        // msg( Msg::Warning, "FeatherDeformBarbs", boost::format("'%1%' attribute on animated shafts has incorrect type or interpolation.") % hairIdAttrName ); // REPLACE
        msg( Msg::Warning, "FeatherDeformBarbs", boost::format("'%1%' attribute on animated shafts must be IntVectorData with Uniform interpolation.") % hairIdAttrName ); // WITH THIS
        return inputObject;
    }

    const PrimitiveVariable* animShaftsUpVecPV = nullptr;
    if(!shaftUpVectorPrimVarName.empty())
    {
        auto animShaftsUpVecIt = animatedShaftsCurves->variables.find( shaftUpVectorPrimVarName );
        if( animShaftsUpVecIt != animatedShaftsCurves->variables.end() )
        {
            animShaftsUpVecPV = &(animShaftsUpVecIt->second);
        }
    }
    const PrimitiveVariable* animShaftsOrientPV = nullptr;
    if(!shaftPointOrientAttrName.empty())
    {
        auto animShaftsOrientIt = animatedShaftsCurves->variables.find( shaftPointOrientAttrName );
        if( animShaftsOrientIt != animatedShaftsCurves->variables.end() )
        {
            animShaftsOrientPV = &(animShaftsOrientIt->second);
        }
    }

    for( size_t i = 0; i < animatedShaftsCurves->numCurves(); ++i )
    {
        // HairIdVariant id; // REPLACE
        int id; // WITH THIS
        // if( animShaftsHairIdInt ) id = animShaftsHairIdInt->readable()[(animShaftsHairIdPV->interpolation == PrimitiveVariable::Uniform && !animShaftsHairIdInt->readable().empty()) ? 0 : i]; // SIMPLIFY
        // else id = animShaftsHairIdString->readable()[(animShaftsHairIdPV->interpolation == PrimitiveVariable::Uniform && !animShaftsHairIdString->readable().empty()) ? 0 : i]; // REMOVE ELSE
        id = animShaftsHairIdInt->readable()[(animShaftsHairIdPV->interpolation == PrimitiveVariable::Uniform && !animShaftsHairIdInt->readable().empty()) ? 0 : i];
        animShaftDataMap[id] = { (int)i, animShaftsP, animShaftsUpVecPV, animShaftsOrientPV };
    }

    // --- Prepare output points ---
    V3fVectorDataPtr outBarbsPData = new V3fVectorData();
    outBarbsPData->writable().resize( inBarbsP->readable().size() );
    std::vector<V3f> &outBarbsPVec = outBarbsPData->writable();

    const std::vector<int>& vertsPerBarb = inBarbsCurves->verticesPerCurve()->readable();
    size_t barbVertexOffset = 0;

    // --- Iterate through input barb primitives ---
    for( size_t i = 0; i < inBarbsCurves->numCurves(); ++i )
    {
        // Read bind attributes for this barb
        // HairIdVariant bind_shaftHairId_val; // REPLACE
        int bind_shaftHairId_val; // WITH THIS
        int bind_shaftPointId_val;
        V3f bind_shaftRest_P_val, bind_shaftRest_T_val, bind_shaftRest_B_val, bind_shaftRest_N_val, bind_barbRootOffsetLocal_val;

        // if(idsAreInt) // REMOVE THIS CONDITION, it's the only path
        // { // REMOVE
        // auto readers_ptr = static_cast<BindAttributeReaders<int>*>(rawReadersPtr); // Use the unique_ptr directly
        if (i >= readers->bindShaftHairIdReader->readable().size() || i >= readers->bindShaftPointIdReader->readable().size() ||
            i >= readers->bindShaftRestPReader->readable().size() || i >= readers->bindShaftRestTReader->readable().size() ||
            i >= readers->bindShaftRestBReader->readable().size() || i >= readers->bindShaftRestNReader->readable().size() ||
            i >= readers->bindBarbRootOffsetLocalReader->readable().size())
        {
            msg(Msg::Warning, "FeatherDeformBarbs", boost::format("Barb %1% bind attribute index out of bounds (Int ID).") % i);
            for(int v=0; v < vertsPerBarb[i]; ++v) outBarbsPVec[barbVertexOffset + v] = inBarbsP->readable()[barbVertexOffset+v];
            barbVertexOffset += vertsPerBarb[i]; continue;
        }
        bind_shaftHairId_val = readers->bindShaftHairIdReader->readable()[i];
        bind_shaftPointId_val = readers->bindShaftPointIdReader->readable()[i];
        bind_shaftRest_P_val = readers->bindShaftRestPReader->readable()[i];
        bind_shaftRest_T_val = readers->bindShaftRestTReader->readable()[i];
        bind_shaftRest_B_val = readers->bindShaftRestBReader->readable()[i];
        bind_shaftRest_N_val = readers->bindShaftRestNReader->readable()[i];
        bind_barbRootOffsetLocal_val = readers->bindBarbRootOffsetLocalReader->readable()[i];
        // } // REMOVE
        // else // REMOVE THIS ELSE BRANCH ENTIRELY
        // { ... } // REMOVE

        auto animShaftIt = animShaftDataMap.find( bind_shaftHairId_val );
        if( animShaftIt == animShaftDataMap.end() )
        {
            msg( Msg::Warning, "FeatherDeformBarbs", boost::format("Barb %1% (HairId) cannot find matching animated shaft. Copying points.") % i );
            for(int v=0; v < vertsPerBarb[i]; ++v) outBarbsPVec[barbVertexOffset + v] = inBarbsP->readable()[barbVertexOffset+v];
            barbVertexOffset += vertsPerBarb[i];
            continue;
        }
        const AnimShaftInfo &animShaftInfo = animShaftIt->second;

        const std::vector<int>& vertsPerAnimShaft = animatedShaftsCurves->verticesPerCurve()->readable();
        size_t animShaftVertexStart = 0;
        for(int sIdx = 0; sIdx < animShaftInfo.primitiveIndex; ++sIdx) animShaftVertexStart += vertsPerAnimShaft[sIdx];
        int numPointsOnAnimShaft = vertsPerAnimShaft[animShaftInfo.primitiveIndex];

        if( bind_shaftPointId_val < 0 || bind_shaftPointId_val >= numPointsOnAnimShaft )
        {
            msg( Msg::Warning, "FeatherDeformBarbs", boost::format("Barb %1% bind_shaftPointId %2% out of range for animated shaft with %3% points. Copying points.") % i % bind_shaftPointId_val % numPointsOnAnimShaft );
            for(int v=0; v < vertsPerBarb[i]; ++v) outBarbsPVec[barbVertexOffset + v] = inBarbsP->readable()[barbVertexOffset+v];
            barbVertexOffset += vertsPerBarb[i];
            continue;
        }
        size_t actualAnimShaftPtIdxInP = animShaftVertexStart + bind_shaftPointId_val;

        // Calculate Animated Shaft Attachment Frame
        V3f P_attach_shaft_anim = animShaftInfo.pData->readable()[actualAnimShaftPtIdxInP];
        V3f T_attach_shaft_anim, B_attach_shaft_anim, N_attach_shaft_anim;
        
        V3f animShaftUpVecVal;
        const V3f* animShaftUpVecPtr = nullptr;
        Quatf animShaftOrientVal;
        const Quatf* animShaftOrientPtr = nullptr;

        if(animShaftInfo.upVectorPrimVar)
        {
            if(getV3fPrimVarValue(*animShaftInfo.upVectorPrimVar, animShaftInfo.primitiveIndex, actualAnimShaftPtIdxInP, animShaftUpVecVal))
            {
                 animShaftUpVecPtr = &animShaftUpVecVal;
            }
        }
        if(animShaftInfo.orientationPrimVar)
        {
            if(getQuatfPrimVarValue(*animShaftInfo.orientationPrimVar, animShaftInfo.primitiveIndex, animShaftOrientVal))
            {
                animShaftOrientPtr = &animShaftOrientVal;
            }
        }

        calculateCurveFrame( animShaftInfo.pData, bind_shaftPointId_val, numPointsOnAnimShaft, animShaftVertexStart, 
                               animShaftUpVecPtr, animShaftOrientPtr, 
                               T_attach_shaft_anim, B_attach_shaft_anim, N_attach_shaft_anim );

        M44f M_shaft_anim;
        M_shaft_anim.setValue( T_attach_shaft_anim.x, T_attach_shaft_anim.y, T_attach_shaft_anim.z, 0.0f,
                               B_attach_shaft_anim.x, B_attach_shaft_anim.y, B_attach_shaft_anim.z, 0.0f,
                               N_attach_shaft_anim.x, N_attach_shaft_anim.y, N_attach_shaft_anim.z, 0.0f,
                               P_attach_shaft_anim.x, P_attach_shaft_anim.y, P_attach_shaft_anim.z, 1.0f );

        // Calculate New Barb Root Position
        V3f P_barb_root_anim_offset_world;
        M_shaft_anim.multDirMatrix(bind_barbRootOffsetLocal_val, P_barb_root_anim_offset_world);
        V3f P_barb_root_anim = P_attach_shaft_anim + P_barb_root_anim_offset_world;

        // Transform Barb Points
        M44f M_shaft_rest_inverse;
        M_shaft_rest_inverse.setValue( bind_shaftRest_T_val.x, bind_shaftRest_B_val.x, bind_shaftRest_N_val.x, 0.0f,
                                       bind_shaftRest_T_val.y, bind_shaftRest_B_val.y, bind_shaftRest_N_val.y, 0.0f,
                                       bind_shaftRest_T_val.z, bind_shaftRest_B_val.z, bind_shaftRest_N_val.z, 0.0f,
                                       0.0f,                   0.0f,                   0.0f,                   1.0f );
        V3f translation_inv( -bind_shaftRest_P_val.dot(bind_shaftRest_T_val), 
                               -bind_shaftRest_P_val.dot(bind_shaftRest_B_val), 
                               -bind_shaftRest_P_val.dot(bind_shaftRest_N_val) );
        M_shaft_rest_inverse[3][0] = translation_inv.x;
        M_shaft_rest_inverse[3][1] = translation_inv.y;
        M_shaft_rest_inverse[3][2] = translation_inv.z;

        M44f M_transform_points = M_shaft_rest_inverse * M_shaft_anim;

        size_t barbRootVertexIndex = barbVertexOffset;
        float minParamU = std::numeric_limits<float>::max();
        const FloatVectorData* currentBarbParamReader = nullptr;
        // if(idsAreInt) currentBarbParamReader = static_cast<BindAttributeReaders<int>*>(rawReadersPtr)->barbParamReader; // SIMPLIFY
        // else currentBarbParamReader = static_cast<BindAttributeReaders<std::string>*>(rawReadersPtr)->barbParamReader; // REMOVE
        currentBarbParamReader = readers->barbParamReader; // Use the unique_ptr directly

        for( int v = 0; v < vertsPerBarb[i]; ++v )
        {
            size_t currentVertexGlobalIndex = barbVertexOffset + v;
            if (currentVertexGlobalIndex >= currentBarbParamReader->readable().size()) {
                 msg( Msg::Warning, "FeatherDeformBarbs", boost::format("Barb %1% vertex param index out of bounds for root finding.") % i ); 
                 goto next_barb_deform; 
            }
            float currentParamU = currentBarbParamReader->readable()[currentVertexGlobalIndex];
            if( currentParamU < minParamU )
            {
                minParamU = currentParamU;
                barbRootVertexIndex = currentVertexGlobalIndex;
            }
        }
        if (barbRootVertexIndex >= inBarbsP->readable().size()) {
            msg( Msg::Warning, "FeatherDeformBarbs", boost::format("Barb %1% root vertex index out of P bounds for root finding.") % i ); 
            goto next_barb_deform; 
        }
        V3f P_barb_root_rest = inBarbsP->readable()[barbRootVertexIndex];

        for( int v = 0; v < vertsPerBarb[i]; ++v )
        {
            size_t p_idx = barbVertexOffset + v;
            const V3f &p_k_orig = inBarbsP->readable()[p_idx];
            V3f vector_from_rest_barb_root = p_k_orig - P_barb_root_rest;
            V3f vector_transformed;
            M_transform_points.multDirMatrix(vector_from_rest_barb_root, vector_transformed);
            outBarbsPVec[p_idx] = P_barb_root_anim + vector_transformed;
        }
    next_barb_deform:
        barbVertexOffset += vertsPerBarb[i];
    }

    // Create output CurvesPrimitive
    CurvesPrimitivePtr outCurves = new CurvesPrimitive();
    outCurves->setTopology( inBarbsCurves->verticesPerCurve(), inBarbsCurves->basis(), inBarbsCurves->periodic() );

    // Copy existing primvars from inBarbs, except P
    for( const auto &pvPair : inBarbsCurves->variables )
    {
        if( pvPair.first != "P" )
        {
            outCurves->variables[pvPair.first] = pvPair.second;
        }
    }
    outCurves->variables["P"] = PrimitiveVariable( PrimitiveVariable::Vertex, outBarbsPData );

    if( cleanupBindAttributesPlug()->getValue() )
    {
        for( const auto &attrName : { "bind_shaftHairId", "bind_shaftPointId", "bind_shaftRest_P", 
                                       "bind_shaftRest_T", "bind_shaftRest_B", "bind_shaftRest_N", 
                                       "bind_barbRootOffsetLocal" } )
        {
            outCurves->variables.erase( attrName );
        }
    }

    return outCurves;
}
