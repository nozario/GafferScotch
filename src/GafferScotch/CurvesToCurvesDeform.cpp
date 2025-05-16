#include "GafferScotch/CurvesToCurvesDeform.h"
#include "GafferScotch/ScenePathUtil.h"
#include "GafferScotch/TypeIds.h"

#include "IECoreScene/CurvesPrimitive.h"
#include "IECoreScene/CurvesPrimitiveEvaluator.h"
#include "IECoreScene/PrimitiveVariable.h"
#include "IECore/DataAlgo.h"
#include "IECore/StringAlgo.h"
#include "IECore/VectorTypedData.h"
#include "IECore/NullObject.h"
#include "IECore/MessageHandler.h"

#include "Gaffer/Context.h"
#include "GafferScene/ScenePlug.h"

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

using namespace Gaffer;
using namespace GafferScene;
using namespace GafferScotch;
using namespace IECore;
using namespace IECoreScene;
using namespace Imath;

namespace
{
    // Helper function for safe vector normalization (copied from RigidDeformCurves.cpp)
    inline V3f safeNormalize(const V3f &v)
    {
        float length = v.length();
        if (length > 0.0f)
        {
            return v / length;
        }
        return v;
    }

    // Helper to determine optimal batch size (copied from RigidDeformCurves.cpp)
    inline size_t calculateBatchSize(size_t numItems, size_t numThreads)
    {
        const size_t targetBatches = numThreads * 4;
        return std::max(size_t(1), numItems / targetBatches);
    }

    struct DeformFrame // Similar to AttachFrame in CurvesToCurvesAttach & RestFrame in RigidDeformCurves
    {
        V3f position;
        V3f tangent;
        V3f normal;
        V3f bitangent;

        void orthonormalize(const V3f &initialUpVectorHint)
        {
            IECore::msg( IECore::Msg::Debug, "CurvesToCurvesDeform::DeformFrame::orthonormalize", "Input tangent: " + std::to_string(tangent.x) + ", " + std::to_string(tangent.y) + ", " + std::to_string(tangent.z) );
            IECore::msg( IECore::Msg::Debug, "CurvesToCurvesDeform::DeformFrame::orthonormalize", "Input initialUpVectorHint: " + std::to_string(initialUpVectorHint.x) + ", " + std::to_string(initialUpVectorHint.y) + ", " + std::to_string(initialUpVectorHint.z) );

            tangent = safeNormalize(tangent);
            IECore::msg( IECore::Msg::Debug, "CurvesToCurvesDeform::DeformFrame::orthonormalize", "Normalized tangent: " + std::to_string(tangent.x) + ", " + std::to_string(tangent.y) + ", " + std::to_string(tangent.z) );

            if (tangent.length2() < 1e-12f)
            {
                IECore::msg( IECore::Msg::Warning, "CurvesToCurvesDeform::DeformFrame::orthonormalize", "Degenerate tangent detected. Setting default frame." );
                this->tangent   = V3f(1.0f, 0.0f, 0.0f);
                this->bitangent = V3f(0.0f, 1.0f, 0.0f);
                this->normal    = V3f(0.0f, 0.0f, 1.0f);
                return;
            }

            V3f up = safeNormalize(initialUpVectorHint);
            if (up.length2() < 1e-12f)
            {
                IECore::msg( IECore::Msg::Debug, "CurvesToCurvesDeform::DeformFrame::orthonormalize", "Initial up vector hint was zero or tiny. Defaulting to Y-up." );
                up = V3f(0.0f, 1.0f, 0.0f);
            }
            IECore::msg( IECore::Msg::Debug, "CurvesToCurvesDeform::DeformFrame::orthonormalize", "Initial 'up' vector: " + std::to_string(up.x) + ", " + std::to_string(up.y) + ", " + std::to_string(up.z) );

            if (std::abs(tangent.dot(up)) > 0.9999f)
            {
                IECore::msg( IECore::Msg::Warning, "CurvesToCurvesDeform::DeformFrame::orthonormalize", "Tangent and 'up' are collinear. Attempting fallback 'up'." );
                if (std::abs(tangent.x) < 0.9f)
                {
                    up = V3f(1.0f, 0.0f, 0.0f);
                    IECore::msg( IECore::Msg::Debug, "CurvesToCurvesDeform::DeformFrame::orthonormalize", "Using X-axis as fallback 'up'." );
                }
                else
                {
                    up = V3f(0.0f, 1.0f, 0.0f);
                    IECore::msg( IECore::Msg::Debug, "CurvesToCurvesDeform::DeformFrame::orthonormalize", "Using Y-axis as fallback 'up'." );
                }

                if (std::abs(tangent.dot(up)) > 0.9999f) {
                    IECore::msg( IECore::Msg::Warning, "CurvesToCurvesDeform::DeformFrame::orthonormalize", "Fallback 'up' still collinear. Second fallback attempt." );
                    if (std::abs(tangent.y) > 0.9f) {
                        up = V3f(1.0f, 0.0f, 0.0f);
                        IECore::msg( IECore::Msg::Debug, "CurvesToCurvesDeform::DeformFrame::orthonormalize", "Second fallback: Using X-axis as 'up'." );
                    } else {
                         IECore::msg( IECore::Msg::Debug, "CurvesToCurvesDeform::DeformFrame::orthonormalize", "Second fallback: 'up' remains: " + std::to_string(up.x) + ", " + std::to_string(up.y) + ", " + std::to_string(up.z) );
                    }
                }
            }

            V3f calculatedNormal = tangent.cross(up);
            IECore::msg( IECore::Msg::Debug, "CurvesToCurvesDeform::DeformFrame::orthonormalize", "Calculated normal (before normalization): " + std::to_string(calculatedNormal.x) + ", " + std::to_string(calculatedNormal.y) + ", " + std::to_string(calculatedNormal.z) + " Length^2: " + std::to_string(calculatedNormal.length2()) );

            if (calculatedNormal.length2() < 1e-12f)
            {
                IECore::msg( IECore::Msg::Warning, "CurvesToCurvesDeform::DeformFrame::orthonormalize", "Normal from tangent.cross(up) is zero or tiny. Using fallback normal generation." );
                if (std::abs(tangent.x) > std::abs(tangent.z)) {
                    calculatedNormal = V3f(-tangent.y, tangent.x, 0.0f);
                } else {
                    calculatedNormal = V3f(0.0f, -tangent.z, tangent.y);
                }
                IECore::msg( IECore::Msg::Debug, "CurvesToCurvesDeform::DeformFrame::orthonormalize", "Fallback normal (before normalization): " + std::to_string(calculatedNormal.x) + ", " + std::to_string(calculatedNormal.y) + ", " + std::to_string(calculatedNormal.z) );
            }
            this->normal = safeNormalize(calculatedNormal);

            this->bitangent = safeNormalize(this->normal.cross(this->tangent)); 
            this->normal = safeNormalize(this->tangent.cross(this->bitangent)); // Ensure strict orthogonality

            IECore::msg( IECore::Msg::Debug, "CurvesToCurvesDeform::DeformFrame::orthonormalize", "Final tangent: " + std::to_string(this->tangent.x) + ", " + std::to_string(this->tangent.y) + ", " + std::to_string(this->tangent.z) );
            IECore::msg( IECore::Msg::Debug, "CurvesToCurvesDeform::DeformFrame::orthonormalize", "Final normal: " + std::to_string(this->normal.x) + ", " + std::to_string(this->normal.y) + ", " + std::to_string(this->normal.z) );
            IECore::msg( IECore::Msg::Debug, "CurvesToCurvesDeform::DeformFrame::orthonormalize", "Final bitangent: " + std::to_string(this->bitangent.x) + ", " + std::to_string(this->bitangent.y) + ", " + std::to_string(this->bitangent.z) );
        }

        M44f toMatrix() const
        {
            M44f matrix;
            matrix[0][0] = tangent.x;   matrix[0][1] = bitangent.x; matrix[0][2] = normal.x;   matrix[0][3] = position.x;
            matrix[1][0] = tangent.y;   matrix[1][1] = bitangent.y; matrix[1][2] = normal.y;   matrix[1][3] = position.y;
            matrix[2][0] = tangent.z;   matrix[2][1] = bitangent.z; matrix[2][2] = normal.z;   matrix[2][3] = position.z;
            matrix[3][0] = 0.0f;        matrix[3][1] = 0.0f;        matrix[3][2] = 0.0f;       matrix[3][3] = 1.0f;
            return matrix.transposed(); // Gaffer/Cortex typically use row-major for matrix construction from components
                                        // but M44f expects column vectors for its set() or constructor from 16 floats.
                                        // Or, build by setting columns directly to avoid confusion.
        }
        
        static M44f buildMatrix(const V3f& t, const V3f& b, const V3f& n, const V3f& p)
        {
            return M44f(
                t.x, t.y, t.z, 0.0f,
                b.x, b.y, b.z, 0.0f,
                n.x, n.y, n.z, 0.0f,
                p.x, p.y, p.z, 1.0f
            );
        }
    };

    const std::vector<std::string> g_bindingAttributes = {
        "cfx:restPosition", "cfx:restTangent", "cfx:restNormal", "cfx:restBitangent",
        "cfx:rootPointOffset", "cfx:parentCurveIndex", "cfx:parentCurveU"
    };

    // Helper to hash binding data from input curves
    void hashBindingData(const IECoreScene::CurvesPrimitive *curves, IECore::MurmurHash &h)
    {
        if (!curves)
            return;
        for (const auto &attrName : g_bindingAttributes)
        {
            auto it = curves->variables.find(attrName);
            if (it != curves->variables.end() && it->second.data)
            {
                it->second.data->hash(h);
            }
            else
            {
                h.append(attrName); // Hash name if data is missing to differentiate
            }
        }
    }
}

IE_CORE_DEFINERUNTIMETYPED( CurvesToCurvesDeform );

size_t CurvesToCurvesDeform::g_firstPlugIndex = 0;

CurvesToCurvesDeform::CurvesToCurvesDeform( const std::string &name )
    : Deformer( name )
{
    storeIndexOfNextChild( g_firstPlugIndex );

    addChild( new ScenePlug( "staticParentDeformer", Plug::In ) );
    addChild( new ScenePlug( "animatedParentDeformer", Plug::In ) );
    addChild( new BoolPlug( "useBindAttr", Plug::In, false ) );
    addChild( new StringPlug( "deformerPath", Plug::In, "" ) ); // Path to both static and animated deformers
    addChild( new StringPlug( "bindAttr", Plug::In, "" ) );
    addChild( new V3fPlug( "upVector", Plug::In, V3f( 0.0f, 1.0f, 0.0f ) ) );
    addChild( new BoolPlug( "useUpVectorAttr", Plug::In, false ) );
    addChild( new StringPlug( "upVectorAttr", Plug::In, "" ) );
    addChild( new BoolPlug( "translationOnly", Plug::In, false ) );
    addChild( new BoolPlug( "cleanupBindAttributes", Plug::In, true ) );

    // Unlike ObjectProcessor, Deformer doesn't automatically pass through attributes.
    // We will handle attribute passthrough in computeProcessedObject if needed, or users can use AttributeCopy.
    // Pass through transform and bound by default, though bound will be recomputed.
    outPlug()->transformPlug()->setInput(inPlug()->transformPlug());
    outPlug()->boundPlug()->setInput(inPlug()->boundPlug()); 
}

CurvesToCurvesDeform::~CurvesToCurvesDeform()
{
}

// Plug accessors
ScenePlug *CurvesToCurvesDeform::staticParentDeformerPlug()
{
    return getChild<ScenePlug>( g_firstPlugIndex );
}
const ScenePlug *CurvesToCurvesDeform::staticParentDeformerPlug() const
{
    return getChild<ScenePlug>( g_firstPlugIndex );
}

ScenePlug *CurvesToCurvesDeform::animatedParentDeformerPlug()
{
    return getChild<ScenePlug>( g_firstPlugIndex + 1 );
}
const ScenePlug *CurvesToCurvesDeform::animatedParentDeformerPlug() const
{
    return getChild<ScenePlug>( g_firstPlugIndex + 1 );
}

BoolPlug *CurvesToCurvesDeform::useBindAttrPlug()
{
    return getChild<BoolPlug>( g_firstPlugIndex + 2 );
}
const BoolPlug *CurvesToCurvesDeform::useBindAttrPlug() const
{
    return getChild<BoolPlug>( g_firstPlugIndex + 2 );
}

StringPlug *CurvesToCurvesDeform::deformerPathPlug()
{
    return getChild<StringPlug>( g_firstPlugIndex + 3 );
}
const StringPlug *CurvesToCurvesDeform::deformerPathPlug() const
{
    return getChild<StringPlug>( g_firstPlugIndex + 3 );
}

StringPlug *CurvesToCurvesDeform::bindAttrPlug()
{
    return getChild<StringPlug>( g_firstPlugIndex + 4 );
}
const StringPlug *CurvesToCurvesDeform::bindAttrPlug() const
{
    return getChild<StringPlug>( g_firstPlugIndex + 4 );
}

V3fPlug *CurvesToCurvesDeform::upVectorPlug()
{
    return getChild<V3fPlug>( g_firstPlugIndex + 5 );
}
const V3fPlug *CurvesToCurvesDeform::upVectorPlug() const
{
    return getChild<V3fPlug>( g_firstPlugIndex + 5 );
}

BoolPlug *CurvesToCurvesDeform::useUpVectorAttrPlug()
{
    return getChild<BoolPlug>( g_firstPlugIndex + 6 );
}
const BoolPlug *CurvesToCurvesDeform::useUpVectorAttrPlug() const
{
    return getChild<BoolPlug>( g_firstPlugIndex + 6 );
}

StringPlug *CurvesToCurvesDeform::upVectorAttrPlug()
{
    return getChild<StringPlug>( g_firstPlugIndex + 7 );
}
const StringPlug *CurvesToCurvesDeform::upVectorAttrPlug() const
{
    return getChild<StringPlug>( g_firstPlugIndex + 7 );
}

BoolPlug *CurvesToCurvesDeform::translationOnlyPlug()
{
    return getChild<BoolPlug>( g_firstPlugIndex + 8 );
}
const BoolPlug *CurvesToCurvesDeform::translationOnlyPlug() const
{
    return getChild<BoolPlug>( g_firstPlugIndex + 8 );
}

BoolPlug *CurvesToCurvesDeform::cleanupBindAttributesPlug()
{
    return getChild<BoolPlug>( g_firstPlugIndex + 9 );
}
const BoolPlug *CurvesToCurvesDeform::cleanupBindAttributesPlug() const
{
    return getChild<BoolPlug>( g_firstPlugIndex + 9 );
}

void CurvesToCurvesDeform::affects( const Gaffer::Plug *input, AffectedPlugsContainer &outputs ) const
{
    Deformer::affects( input, outputs );

    if(
        input == staticParentDeformerPlug()->objectPlug() ||
        input == animatedParentDeformerPlug()->objectPlug() ||
        input == useBindAttrPlug() ||
        input == deformerPathPlug() ||
        input == bindAttrPlug() ||
        input == upVectorPlug() ||
        input == useUpVectorAttrPlug() ||
        input == upVectorAttrPlug() ||
        input == translationOnlyPlug() ||
        input == cleanupBindAttributesPlug()
    )
    {
        outputs.push_back( outPlug()->objectPlug() );
    }
    
    // If the animated deformer changes, the bound of the output will also change.
    if (input == animatedParentDeformerPlug()->objectPlug() || input == animatedParentDeformerPlug()->boundPlug())
    {
        outputs.push_back( outPlug()->boundPlug() );
    }
}

bool CurvesToCurvesDeform::affectsProcessedObject( const Gaffer::Plug *input ) const
{
    return
        input == staticParentDeformerPlug()->objectPlug() ||
        input == animatedParentDeformerPlug()->objectPlug() ||
        input == useBindAttrPlug() ||
        input == deformerPathPlug() ||
        input == bindAttrPlug() ||
        input == upVectorPlug() ||
        input == useUpVectorAttrPlug() ||
        input == upVectorAttrPlug() ||
        input == translationOnlyPlug() ||
        input == cleanupBindAttributesPlug();
}

void CurvesToCurvesDeform::hashProcessedObject( const GafferScene::ScenePlug::ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
    Deformer::hashProcessedObject( path, context, h ); // Hashes inPlug()->objectPlug() among other things.

    const CurvesPrimitive *childCurves = runTimeCast<const CurvesPrimitive>( inPlug()->object( path ).get() );
    if( !childCurves )
    {
        return; // Input object hash is enough
    }

    ScenePath parentDeformerScenePath;
    const bool useBind = useBindAttrPlug()->getValue();
    if( useBind )
    {
        const std::string bindAttrName = bindAttrPlug()->getValue();
        if( !bindAttrName.empty() )
        {
            auto it = childCurves->variables.find( bindAttrName );
            if( it != childCurves->variables.end() )
            {
                if( const StringData *pathData = runTimeCast<const StringData>( it->second.data.get() ) )
                {
                    parentDeformerScenePath = GafferScotch::makeScenePath( pathData->readable() );
                }
                else if( const StringVectorData *pathVectorData = runTimeCast<const StringVectorData>( it->second.data.get() ) )
                {
                    const std::vector<std::string> &paths = pathVectorData->readable();
                    if( !paths.empty() ) parentDeformerScenePath = GafferScotch::makeScenePath( paths[0] );
                }
            }
        }
    }
    else
    {
        parentDeformerScenePath = GafferScotch::makeScenePath( deformerPathPlug()->getValue() );
    }

    if( !parentDeformerScenePath.empty() )
    {
        // Static parent deformer is less critical if cfx:rest* attributes are present and trusted,
        // but hash it for completeness or if logic evolves to use it.
        h.append( staticParentDeformerPlug()->objectHash( parentDeformerScenePath ) );
        h.append( animatedParentDeformerPlug()->objectHash( parentDeformerScenePath ) );
    }
    else
    {
        IECore::MurmurHash emptyPathHash("emptyParentDeformerPath");
        h.append(emptyPathHash);
    }

    // Hash binding data stored on the input child curves
    hashBindingData( childCurves, h );

    // Hash own plugs
    // useBindAttr, deformerPath, bindAttr already used for path resolution, but hash them directly too for clarity.
    useBindAttrPlug()->hash( h );
    deformerPathPlug()->hash( h );
    bindAttrPlug()->hash( h );
    upVectorPlug()->hash( h );
    useUpVectorAttrPlug()->hash( h );
    upVectorAttrPlug()->hash( h );
    translationOnlyPlug()->hash( h );
    cleanupBindAttributesPlug()->hash( h );

    // If up-vector is from an attribute on the child, hash that attribute.
    // This should ideally be part of hashBindingData if "upVectorSourceAttr" was a binding attr.
    // For now, handle explicitly based on plug.
    if (useUpVectorAttrPlug()->getValue()) 
    {
        const std::string upVecAttrName = upVectorAttrPlug()->getValue();
        if (!upVecAttrName.empty()) 
        {
            auto it = childCurves->variables.find(upVecAttrName);
            if (it != childCurves->variables.end() && it->second.data) 
            {
                it->second.data->hash(h);
            }
        }
    }
}

IECore::ConstObjectPtr CurvesToCurvesDeform::computeProcessedObject( const GafferScene::ScenePlug::ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject ) const
{
    const CurvesPrimitive *inputChildCurves = runTimeCast<const CurvesPrimitive>( inputObject );
    if( !inputChildCurves || inputChildCurves->verticesPerCurve()->readable().empty() )
    {
        return inputObject; // Not curves or no curves to process
    }

    // Check for essential binding attributes before proceeding
    if( inputChildCurves->variables.find("cfx:restPosition") == inputChildCurves->variables.end() ||
        inputChildCurves->variables.find("cfx:parentCurveIndex") == inputChildCurves->variables.end() )
    {
        // Missing core binding data. Issue warning?
        return inputObject;
    }

    ScenePath parentDeformerScenePath;
    const bool useBind = useBindAttrPlug()->getValue();
    if( useBind )
    {
        const std::string bindAttrName = bindAttrPlug()->getValue();
        if( !bindAttrName.empty() )
        {
            auto it = inputChildCurves->variables.find( bindAttrName );
            if( it != inputChildCurves->variables.end() )
            {
                if( const StringData *pathData = runTimeCast<const StringData>( it->second.data.get() ) )
                {
                    parentDeformerScenePath = GafferScotch::makeScenePath( pathData->readable() );
                }
                else if( const StringVectorData *pathVectorData = runTimeCast<const StringVectorData>( it->second.data.get() ) )
                {
                    const std::vector<std::string> &paths = pathVectorData->readable();
                    if( !paths.empty() ) parentDeformerScenePath = GafferScotch::makeScenePath( paths[0] );
                }
            }
        }
    }
    else
    {
        parentDeformerScenePath = GafferScotch::makeScenePath( deformerPathPlug()->getValue() );
    }

    if( parentDeformerScenePath.empty() )
    {
        return inputObject; // Path to parent deformer not specified
    }

    // Static parent deformer might not be strictly needed if all rest info is from cfx: attributes,
    // but retrieve it for completeness or future logic.
    ConstObjectPtr staticParentObj = staticParentDeformerPlug()->object( parentDeformerScenePath );
    const CurvesPrimitive *staticParentDeformerCurves = runTimeCast<const CurvesPrimitive>( staticParentObj.get() );

    ConstObjectPtr animatedParentObj = animatedParentDeformerPlug()->object( parentDeformerScenePath );
    const CurvesPrimitive *animatedParentDeformerCurves = runTimeCast<const CurvesPrimitive>( animatedParentObj.get() );

    if( !animatedParentDeformerCurves || animatedParentDeformerCurves->verticesPerCurve()->readable().empty() )
    {
        return inputObject; // Animated parent deformer is invalid or empty
    }

    CurvesPrimitivePtr outputChildCurves = inputChildCurves->copy();

    deformChildCurves( inputChildCurves, staticParentDeformerCurves, animatedParentDeformerCurves, outputChildCurves.get() );

    if( cleanupBindAttributesPlug()->getValue() )
    {
        for( const auto &attrName : g_bindingAttributes )
        {
            auto it = outputChildCurves->variables.find( attrName );
            if( it != outputChildCurves->variables.end() )
            {
                outputChildCurves->variables.erase( it );
            }
        }
        // Also remove the bind path attribute if it exists and we used it
        if (useBind) 
        {
            const std::string bindAttrName = bindAttrPlug()->getValue();
            if (!bindAttrName.empty()) 
            {
                 auto it = outputChildCurves->variables.find(bindAttrName);
                 if (it != outputChildCurves->variables.end()) outputChildCurves->variables.erase(it);
            }
        }
    }
    
    return outputChildCurves;
}

void CurvesToCurvesDeform::deformChildCurves(
    const IECoreScene::CurvesPrimitive *inputChildCurves,
    const IECoreScene::CurvesPrimitive * /* staticParentDeformerCurves */, // Currently unused, rest frame from attributes
    const IECoreScene::CurvesPrimitive *animatedParentDeformerCurves,
    IECoreScene::CurvesPrimitive *outputChildCurves) const
{
    // Retrieve binding data - assuming Uniform interpolation for all cfx attributes
    const V3fVectorData *restPData = inputChildCurves->variableData<V3fVectorData>("cfx:restPosition", PrimitiveVariable::Uniform);
    const V3fVectorData *restTData = inputChildCurves->variableData<V3fVectorData>("cfx:restTangent", PrimitiveVariable::Uniform);
    const V3fVectorData *restNData = inputChildCurves->variableData<V3fVectorData>("cfx:restNormal", PrimitiveVariable::Uniform);
    const V3fVectorData *restBData = inputChildCurves->variableData<V3fVectorData>("cfx:restBitangent", PrimitiveVariable::Uniform);
    const V3fVectorData *rootOffsetData = inputChildCurves->variableData<V3fVectorData>("cfx:rootPointOffset", PrimitiveVariable::Uniform);
    const IntVectorData *parentIndexData = inputChildCurves->variableData<IntVectorData>("cfx:parentCurveIndex", PrimitiveVariable::Uniform);
    const FloatVectorData *parentUData = inputChildCurves->variableData<FloatVectorData>("cfx:parentCurveU", PrimitiveVariable::Uniform);

    if (!restPData || !restTData || !restNData || !restBData || !rootOffsetData || !parentIndexData || !parentUData)
    {
        // Missing one or more essential binding attributes.
        IECore::msg( IECore::Msg::Warning, "CurvesToCurvesDeform", "Missing one or more cfx:* binding attributes on input curves." );
        return;
    }

    const std::vector<V3f> &restPositions = restPData->readable();
    const std::vector<V3f> &restTangents = restTData->readable();
    const std::vector<V3f> &restNormals = restNData->readable();
    const std::vector<V3f> &restBitangents = restBData->readable();
    const std::vector<V3f> &rootPointOffsets = rootOffsetData->readable();
    const std::vector<int> &parentCurveIndices = parentIndexData->readable();
    const std::vector<float> &parentCurveUs = parentUData->readable();

    const V3fVectorData *inputPData = inputChildCurves->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);
    if (!inputPData) return; // No points to deform
    
    V3fVectorDataPtr outputPData = inputPData->copy();
    std::vector<V3f> &outputPoints = outputPData->writable();

    const std::vector<int> &childVertsPerCurve = inputChildCurves->verticesPerCurve()->readable();
    const size_t numChildCurves = childVertsPerCurve.size();

    if (numChildCurves == 0 || restPositions.size() != numChildCurves) {
         IECore::msg( IECore::Msg::Warning, "CurvesToCurvesDeform", "Mismatch between number of child curves and binding data size." );
        return; // Mismatch or no curves
    }

    std::vector<size_t> childVertexOffsets;
    childVertexOffsets.reserve(numChildCurves);
    size_t currentOffset = 0;
    for (int count : childVertsPerCurve)
    {
        childVertexOffsets.push_back(currentOffset);
        currentOffset += count;
    }

    CurvesPrimitiveEvaluatorPtr animatedParentEvaluator = new CurvesPrimitiveEvaluator(ConstCurvesPrimitivePtr(animatedParentDeformerCurves));
    
    const bool useUpVecAttr = useUpVectorAttrPlug()->getValue();
    const std::string upVecAttrName = upVectorAttrPlug()->getValue();
    const V3f defaultUpVectorFromPlug = upVectorPlug()->getValue();
    const bool translationOnly = translationOnlyPlug()->getValue();

    IECore::msg( IECore::Msg::Debug, "CurvesToCurvesDeform::upVectorSetup", std::string("Use Attribute for UpVector: ") + (useUpVecAttr ? "True" : "False") + ". Attr name: '" + upVecAttrName + "'. Plug default: (" + std::to_string(defaultUpVectorFromPlug.x) + ", " + std::to_string(defaultUpVectorFromPlug.y) + ", " + std::to_string(defaultUpVectorFromPlug.z) + ")" );
    if (translationOnly) {
        IECore::msg( IECore::Msg::Debug, "CurvesToCurvesDeform::mode", "Translation Only Mode: True");
    }

    // TBB Parallelization
    const size_t numThreads = std::thread::hardware_concurrency();
    const size_t batchSize = calculateBatchSize(numChildCurves, numThreads);

    tbb::parallel_for(tbb::blocked_range<size_t>(0, numChildCurves, batchSize),
        [&](const tbb::blocked_range<size_t> &range) {
            PrimitiveEvaluator::ResultPtr localAnimEvalResult = animatedParentEvaluator->createResult(); // Thread-local
            
            for (size_t i = range.begin(); i != range.end(); ++i)
            {
                if (parentCurveIndices[i] < 0) continue; // Invalid binding for this curve

                DeformFrame restFrameOnParent;
                restFrameOnParent.position = restPositions[i];
                restFrameOnParent.tangent = restTangents[i];
                restFrameOnParent.normal = restNormals[i];
                restFrameOnParent.bitangent = restBitangents[i];

                const V3f &rootOffset = rootPointOffsets[i];
                V3f childRestRootPos = restFrameOnParent.position + rootOffset;

                if (!animatedParentEvaluator->pointAtV(parentCurveIndices[i], parentCurveUs[i], localAnimEvalResult.get()))
                {
                    // Failed to evaluate animated parent curve, skip this child curve or copy points?
                    // For now, points for this curve remain unchanged (as they were copied)
                    continue;
                }
                const CurvesPrimitiveEvaluator::Result *animEvalRes = static_cast<const CurvesPrimitiveEvaluator::Result *>(localAnimEvalResult.get());

                DeformFrame deformedFrameOnParent;
                deformedFrameOnParent.position = animEvalRes->point();
                deformedFrameOnParent.tangent = animEvalRes->vTangent();
                
                V3f finalUpVectorForCurve = defaultUpVectorFromPlug;
                std::string upVectorSource = "plug"; // Default to plug

                if (useUpVecAttr && !upVecAttrName.empty()) {
                    auto it = inputChildCurves->variables.find(upVecAttrName);
                    if (it != inputChildCurves->variables.end() && it->second.data) {
                        bool attributeSuccessfullyUsed = false;
                        if (it->second.interpolation == PrimitiveVariable::Constant) {
                            if (const V3fData *upData = runTimeCast<const V3fData>(it->second.data.get())) { 
                                finalUpVectorForCurve = upData->readable();
                                attributeSuccessfullyUsed = true;
                            }
                        }
                        else if (it->second.interpolation == PrimitiveVariable::Uniform) {
                            if (const V3fVectorData *upVecData = runTimeCast<const V3fVectorData>(it->second.data.get())) { 
                                if (i < upVecData->readable().size())
                                {
                                    finalUpVectorForCurve = upVecData->readable()[i];
                                    attributeSuccessfullyUsed = true;
                                }
                            }
                        } else if (it->second.interpolation == PrimitiveVariable::Vertex || it->second.interpolation == PrimitiveVariable::Varying) {
                            if (const V3fVectorData *upVecData = runTimeCast<const V3fVectorData>(it->second.data.get())) {
                                if (!upVecData->readable().empty()) { 
                                    finalUpVectorForCurve = upVecData->readable()[childVertexOffsets[i]]; 
                                    attributeSuccessfullyUsed = true;
                                     // Simplified: No detailed log about using first point here, covered by general fallback message if it fails.
                                }
                            }
                        }
                        
                        if (attributeSuccessfullyUsed) {
                            upVectorSource = "attribute ('" + upVecAttrName + "')";
                        } else {
                            IECore::msg( IECore::Msg::Warning, "CurvesToCurvesDeform::upVector", "Curve " + std::to_string(i) + ": Failed to use attribute '" + upVecAttrName + "' (type/size/interp mismatch or data issue). Falling back to plug." );
                            // finalUpVectorForCurve remains defaultUpVectorFromPlug
                        }
                    }
                    else
                    {
                         IECore::msg( IECore::Msg::Debug, "CurvesToCurvesDeform::upVector", "Curve " + std::to_string(i) + ": Attribute '" + upVecAttrName + "' not found. Using plug." );
                        // finalUpVectorForCurve remains defaultUpVectorFromPlug
                    }
                }
                // No specific message if useUpVecAttr is true but attr name is empty, or if useUpVecAttr is false; it just uses the plug.

                IECore::msg( IECore::Msg::Debug, "CurvesToCurvesDeform::upVectorDetail", "Curve " + std::to_string(i) + ": Final up-vector source: " + upVectorSource + ". Value: (" + std::to_string(finalUpVectorForCurve.x) + ", " + std::to_string(finalUpVectorForCurve.y) + ", " + std::to_string(finalUpVectorForCurve.z) + ")" );
                deformedFrameOnParent.orthonormalize(finalUpVectorForCurve);

                M44f restMatrix = DeformFrame::buildMatrix(restFrameOnParent.tangent, restFrameOnParent.bitangent, restFrameOnParent.normal, restFrameOnParent.position);
                M44f transformMatrix;

                if (translationOnly)
                {
                    // Use rest orientation with the new animated position
                    M44f effectiveDeformedMatrix = DeformFrame::buildMatrix(
                        restFrameOnParent.tangent,    // Rest T
                        restFrameOnParent.bitangent,  // Rest B
                        restFrameOnParent.normal,     // Rest N
                        deformedFrameOnParent.position  // Animated P
                    );
                    transformMatrix = effectiveDeformedMatrix * restMatrix.inverse();
                }
                else
                {
                    M44f deformedMatrix = DeformFrame::buildMatrix(
                        deformedFrameOnParent.tangent,
                        deformedFrameOnParent.bitangent,
                        deformedFrameOnParent.normal,
                        deformedFrameOnParent.position
                    );
                    transformMatrix = deformedMatrix * restMatrix.inverse();
                }
                
                const size_t childStartVtx = childVertexOffsets[i];
                const size_t numVertsOnChild = childVertsPerCurve[i];

                for (size_t j = 0; j < numVertsOnChild; ++j)
                {
                    const size_t vtxIdx = childStartVtx + j;
                    const V3f &origP = inputPData->readable()[vtxIdx]; 
                    
                    V3f pRelativeToChildRestRoot = origP - childRestRootPos;
                    V3f deformedPRelative;
                    transformMatrix.multDirMatrix(pRelativeToChildRestRoot, deformedPRelative); // Apply 3x3 part
                    
                    V3f transformedRootOffset;
                    transformMatrix.multDirMatrix(rootOffset, transformedRootOffset);
                    V3f deformedChildRootPos = transformedRootOffset + deformedFrameOnParent.position;

                    outputPoints[vtxIdx] = deformedChildRootPos + deformedPRelative;
                }
            }
        }
    );

    outputChildCurves->variables["P"] = PrimitiveVariable(PrimitiveVariable::Vertex, outputPData);
}

// Bound computations - similar to RigidDeformCurves

bool CurvesToCurvesDeform::affectsProcessedObjectBound( const Gaffer::Plug *input ) const
{
    return Deformer::affectsProcessedObjectBound(input) ||
           input == animatedParentDeformerPlug()->objectPlug() ||
           input == animatedParentDeformerPlug()->boundPlug() ||
           input == useBindAttrPlug() ||
           input == deformerPathPlug() ||
           input == bindAttrPlug();
}

void CurvesToCurvesDeform::hashProcessedObjectBound( const GafferScene::ScenePlug::ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
    Deformer::hashProcessedObjectBound( path, context, h );

    const CurvesPrimitive *childCurves = runTimeCast<const CurvesPrimitive>( inPlug()->object( path ).get() );
    // No need to hash childCurves specific data for bound beyond what Deformer::hashProcessedObjectBound does (which hashes inPlug()->boundHash())

    ScenePath parentDeformerScenePath;
    if( childCurves ) // Only resolve path if we have child curves to potentially read bindAttr from
    {
        const bool useBind = useBindAttrPlug()->getValue();
        if( useBind )
        {
            const std::string bindAttrName = bindAttrPlug()->getValue();
            if( !bindAttrName.empty() )
            {
                auto it = childCurves->variables.find( bindAttrName );
                if( it != childCurves->variables.end() )
                {
                    if( const StringData *pathData = runTimeCast<const StringData>( it->second.data.get() ) )
                    {
                        parentDeformerScenePath = GafferScotch::makeScenePath( pathData->readable() );
                    }
                    else if( const StringVectorData *pathVectorData = runTimeCast<const StringVectorData>( it->second.data.get() ) )
                    {
                        const std::vector<std::string> &paths = pathVectorData->readable();
                        if( !paths.empty() ) parentDeformerScenePath = GafferScotch::makeScenePath( paths[0] );
                    }
                }
            }
        }
        else
        {
            parentDeformerScenePath = GafferScotch::makeScenePath( deformerPathPlug()->getValue() );
        }
    }
    else
    {
         parentDeformerScenePath = GafferScotch::makeScenePath( deformerPathPlug()->getValue() );
    }
    
    if (!parentDeformerScenePath.empty())
    {
        h.append(animatedParentDeformerPlug()->boundHash(parentDeformerScenePath));
        // We don't strictly need objectHash for bounds, boundHash is usually enough.
    }
    else
    {
        IECore::MurmurHash emptyPathHash("emptyParentDeformerPathForBound");
        h.append(emptyPathHash);
    }

    useBindAttrPlug()->hash(h);
    deformerPathPlug()->hash(h);
    bindAttrPlug()->hash(h);
    // Other plugs don't directly influence the bound beyond the animatedParentDeformer's bound itself.
}

Imath::Box3f CurvesToCurvesDeform::computeProcessedObjectBound( const GafferScene::ScenePlug::ScenePath &path, const Gaffer::Context *context ) const
{
    ConstObjectPtr inputObject = inPlug()->object(path);
    const CurvesPrimitive *childCurves = runTimeCast<const CurvesPrimitive>(inputObject.get());

    ScenePath parentDeformerScenePath;
    if (childCurves) 
    {
        const bool useBind = useBindAttrPlug()->getValue();
        if (useBind) 
        {
            const std::string bindAttrName = bindAttrPlug()->getValue();
            if (!bindAttrName.empty()) 
            {
                auto it = childCurves->variables.find(bindAttrName);
                if (it != childCurves->variables.end()) 
                {
                    if (const StringData *pathData = runTimeCast<const StringData>(it->second.data.get())) 
                    {
                        parentDeformerScenePath = GafferScotch::makeScenePath(pathData->readable());
                    }
                     else if (const StringVectorData *pathVectorData = runTimeCast<const StringVectorData>(it->second.data.get())) 
                    {
                        const std::vector<std::string> &paths = pathVectorData->readable();
                        if (!paths.empty()) parentDeformerScenePath = GafferScotch::makeScenePath(paths[0]);
                    }
                }
            }
        }
         else 
        {
            parentDeformerScenePath = GafferScotch::makeScenePath(deformerPathPlug()->getValue());
        }
    }
    else
    {
        parentDeformerScenePath = GafferScotch::makeScenePath(deformerPathPlug()->getValue());
    }

    if (!parentDeformerScenePath.empty())
    {
        ConstObjectPtr animatedDeformerObj = animatedParentDeformerPlug()->object(parentDeformerScenePath);
        const CurvesPrimitive *animatedDeformer = runTimeCast<const CurvesPrimitive>(animatedDeformerObj.get());
        if (animatedDeformer)
        {
            // For simplicity, start by using the animated deformer's bound.
            // A more accurate bound would involve the extent of the child curves relative to it.
            return animatedParentDeformerPlug()->bound(parentDeformerScenePath);
        }
    }
    
    // Fallback to input bound if no valid animated deformer path or object
    return Deformer::computeProcessedObjectBound( path, context );
} 
} 
