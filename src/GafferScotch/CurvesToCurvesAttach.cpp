#include "GafferScotch/CurvesToCurvesAttach.h"
#include "GafferScotch/ScenePathUtil.h"
#include "GafferScotch/TypeIds.h" // Included for clarity, though CurvesToCurvesAttach.h includes it

#include "IECoreScene/CurvesPrimitive.h"
#include "IECoreScene/CurvesPrimitiveEvaluator.h"
#include "IECoreScene/PrimitiveVariable.h"
#include "IECore/DataAlgo.h"
#include "IECore/StringAlgo.h"
#include "IECore/VectorTypedData.h"
#include "IECore/NullObject.h"
#include "IECore/MessageHandler.h" // For IECore::msg

#include "Gaffer/Context.h"
#include "GafferScene/ScenePlug.h"

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#include <algorithm> // For std::max_element if needed for findRootPointIndex

using namespace Gaffer;
using namespace GafferScene;
using namespace GafferScotch;
using namespace IECore;
using namespace IECoreScene;
using namespace Imath;

// YOUHOU

namespace
{
    // Helper function for safe vector normalization (copied from RigidAttachCurves.cpp)
    inline V3f safeNormalize(const V3f &v)
    {
        float length = v.length();
        if (length > 0.0f)
        {
            return v / length;
        }
        return v;
    }

    // Helper to determine optimal batch size (copied from RigidAttachCurves.cpp)
    inline size_t calculateBatchSize(size_t numItems, size_t numThreads)
    {
        const size_t targetBatches = numThreads * 4;
        return std::max(size_t(1), numItems / targetBatches);
    }

    struct AttachFrame // Similar to RestFrame in RigidAttachCurves
    {
        V3f position;  // Point on parent curve
        V3f tangent;   // Tangent of parent curve
        V3f normal;    // Normal (derived)
        V3f bitangent; // Bitangent (derived)

        void orthonormalize(const V3f &initialUpVectorHint)
        {
            IECore::msg( IECore::Msg::Debug, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Input tangent: " + std::to_string(tangent.x) + ", " + std::to_string(tangent.y) + ", " + std::to_string(tangent.z) );
            IECore::msg( IECore::Msg::Debug, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Input initialUpVectorHint: " + std::to_string(initialUpVectorHint.x) + ", " + std::to_string(initialUpVectorHint.y) + ", " + std::to_string(initialUpVectorHint.z) );

            tangent = safeNormalize(tangent);
            IECore::msg( IECore::Msg::Debug, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Normalized tangent: " + std::to_string(tangent.x) + ", " + std::to_string(tangent.y) + ", " + std::to_string(tangent.z) );

            if (tangent.length2() < 1e-12f)
            {
                IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Degenerate tangent detected. Setting default frame." );
                this->tangent   = V3f(1.0f, 0.0f, 0.0f);
                this->bitangent = V3f(0.0f, 1.0f, 0.0f);
                this->normal    = V3f(0.0f, 0.0f, 1.0f);
                return;
            }

            V3f up = safeNormalize(initialUpVectorHint);
            if (up.length2() < 1e-12f)
            {
                IECore::msg( IECore::Msg::Debug, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Initial up vector hint was zero or tiny. Defaulting to Y-up." );
                up = V3f(0.0f, 1.0f, 0.0f);
            }
            IECore::msg( IECore::Msg::Debug, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Initial 'up' vector: " + std::to_string(up.x) + ", " + std::to_string(up.y) + ", " + std::to_string(up.z) );

            if (std::abs(tangent.dot(up)) > 0.9999f)
            {
                IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Tangent and 'up' are collinear. Attempting fallback 'up'." );
                if (std::abs(tangent.x) < 0.9f)
                {
                    up = V3f(1.0f, 0.0f, 0.0f);
                    IECore::msg( IECore::Msg::Debug, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Using X-axis as fallback 'up'." );
                }
                else
                {
                    up = V3f(0.0f, 1.0f, 0.0f);
                    IECore::msg( IECore::Msg::Debug, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Using Y-axis as fallback 'up'." );
                }

                if (std::abs(tangent.dot(up)) > 0.9999f) {
                    IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Fallback 'up' still collinear. Second fallback attempt." );
                    if (std::abs(tangent.y) > 0.9f) {
                        up = V3f(1.0f, 0.0f, 0.0f);
                        IECore::msg( IECore::Msg::Debug, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Second fallback: Using X-axis as 'up'." );
                    } else {
                        // No change to 'up', or could try Z-axis if needed, though current logic covers X/Y alignment
                         IECore::msg( IECore::Msg::Debug, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Second fallback: 'up' remains: " + std::to_string(up.x) + ", " + std::to_string(up.y) + ", " + std::to_string(up.z) );
                    }
                }
            }

            V3f calculatedNormal = tangent.cross(up);
            IECore::msg( IECore::Msg::Debug, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Calculated normal (before normalization): " + std::to_string(calculatedNormal.x) + ", " + std::to_string(calculatedNormal.y) + ", " + std::to_string(calculatedNormal.z) + " Length^2: " + std::to_string(calculatedNormal.length2()) );

            if (calculatedNormal.length2() < 1e-12f)
            {
                IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Normal from tangent.cross(up) is zero or tiny. Using fallback normal generation." );
                if (std::abs(tangent.x) > std::abs(tangent.z)) {
                    calculatedNormal = V3f(-tangent.y, tangent.x, 0.0f);
                } else {
                    calculatedNormal = V3f(0.0f, -tangent.z, tangent.y);
                }
                IECore::msg( IECore::Msg::Debug, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Fallback normal (before normalization): " + std::to_string(calculatedNormal.x) + ", " + std::to_string(calculatedNormal.y) + ", " + std::to_string(calculatedNormal.z) );
            }
            this->normal = safeNormalize(calculatedNormal);

            // Recompute bitangent to be orthogonal to both
            this->bitangent = safeNormalize(this->normal.cross(this->tangent));
            // Recompute normal to ensure orthogonality with new bitangent and original tangent
            this->normal = safeNormalize(this->tangent.cross(this->bitangent));

            IECore::msg( IECore::Msg::Debug, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Final tangent: " + std::to_string(this->tangent.x) + ", " + std::to_string(this->tangent.y) + ", " + std::to_string(this->tangent.z) );
            IECore::msg( IECore::Msg::Debug, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Final normal: " + std::to_string(this->normal.x) + ", " + std::to_string(this->normal.y) + ", " + std::to_string(this->normal.z) );
            IECore::msg( IECore::Msg::Debug, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Final bitangent: " + std::to_string(this->bitangent.x) + ", " + std::to_string(this->bitangent.y) + ", " + std::to_string(this->bitangent.z) );
        }
    };

    struct CurveBindingData
    {
        AttachFrame restFrame;        // Frame on the parent deformer curve
        V3f rootPointOffset;          // Offset from restFrame.position to child's root
        int parentDeformerCurveIndex; // Index of the curve in the parent CurvesPrimitive
        float parentDeformerCurveU;   // Parametric U along the parent curve
        bool valid;

        CurveBindingData() : parentDeformerCurveIndex(-1), parentDeformerCurveU(0.0f), valid(false) {}
    };
}

IE_CORE_DEFINERUNTIMETYPED(CurvesToCurvesAttach);

size_t CurvesToCurvesAttach::g_firstPlugIndex = 0;

CurvesToCurvesAttach::CurvesToCurvesAttach(const std::string &name)
    : ObjectProcessor(name)
{
    storeIndexOfNextChild(g_firstPlugIndex);

    addChild(new ScenePlug("parentDeformer", Plug::In));
    addChild(new StringPlug("curveRootAttr", Plug::In, ""));
    addChild(new BoolPlug("useBindAttr", Plug::In, false));
    addChild(new StringPlug("deformerPath", Plug::In, ""));
    addChild(new StringPlug("bindAttr", Plug::In, ""));
    addChild(new V3fPlug("upVector", Plug::In, V3f(0.0f, 1.0f, 0.0f)));
    addChild(new BoolPlug("useUpVectorAttr", Plug::In, false));
    addChild(new StringPlug("upVectorAttr", Plug::In, ""));

    // Fast pass-throughs for things we don't modify by default
    outPlug()->attributesPlug()->setInput(inPlug()->attributesPlug());
    outPlug()->transformPlug()->setInput(inPlug()->transformPlug());
    outPlug()->boundPlug()->setInput(inPlug()->boundPlug());
}

CurvesToCurvesAttach::~CurvesToCurvesAttach()
{
}

ScenePlug *CurvesToCurvesAttach::parentDeformerPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

const ScenePlug *CurvesToCurvesAttach::parentDeformerPlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

StringPlug *CurvesToCurvesAttach::curveRootAttrPlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 1);
}

const StringPlug *CurvesToCurvesAttach::curveRootAttrPlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 1);
}

BoolPlug *CurvesToCurvesAttach::useBindAttrPlug()
{
    return getChild<BoolPlug>(g_firstPlugIndex + 2);
}

const BoolPlug *CurvesToCurvesAttach::useBindAttrPlug() const
{
    return getChild<BoolPlug>(g_firstPlugIndex + 2);
}

StringPlug *CurvesToCurvesAttach::deformerPathPlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 3);
}

const StringPlug *CurvesToCurvesAttach::deformerPathPlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 3);
}

StringPlug *CurvesToCurvesAttach::bindAttrPlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 4);
}

const StringPlug *CurvesToCurvesAttach::bindAttrPlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 4);
}

V3fPlug *CurvesToCurvesAttach::upVectorPlug()
{
    return getChild<V3fPlug>(g_firstPlugIndex + 5);
}

const V3fPlug *CurvesToCurvesAttach::upVectorPlug() const
{
    return getChild<V3fPlug>(g_firstPlugIndex + 5);
}

BoolPlug *CurvesToCurvesAttach::useUpVectorAttrPlug()
{
    return getChild<BoolPlug>(g_firstPlugIndex + 6);
}

const BoolPlug *CurvesToCurvesAttach::useUpVectorAttrPlug() const
{
    return getChild<BoolPlug>(g_firstPlugIndex + 6);
}

StringPlug *CurvesToCurvesAttach::upVectorAttrPlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 7);
}

const StringPlug *CurvesToCurvesAttach::upVectorAttrPlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 7);
}

bool CurvesToCurvesAttach::affectsProcessedObject(const Gaffer::Plug *input) const
{
    return input == parentDeformerPlug()->objectPlug() ||
           input == curveRootAttrPlug() ||
           input == useBindAttrPlug() ||
           input == deformerPathPlug() ||
           input == bindAttrPlug() ||
           input == upVectorPlug() ||
           input == useUpVectorAttrPlug() ||
           input == upVectorAttrPlug();
}

void CurvesToCurvesAttach::hashProcessedObject(const GafferScene::ScenePlug::ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    ObjectProcessor::hashProcessedObject(path, context, h); // Hash input object first

    ConstObjectPtr childInputObject = inPlug()->object(path);
    const CurvesPrimitive *childCurves = runTimeCast<const CurvesPrimitive>(childInputObject.get());

    if (!childCurves)
    {
        // If it's not curves, or null, the hash of inputObject is enough
        return;
    }

    ScenePath parentDeformerScenePath;
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
                    if (!paths.empty())
                    {
                        parentDeformerScenePath = GafferScotch::makeScenePath(paths[0]); // Use first path
                    }
                }
            }
        }
    }
    else
    {
        parentDeformerScenePath = GafferScotch::makeScenePath(deformerPathPlug()->getValue());
    }

    if (!parentDeformerScenePath.empty())
    {
        h.append(parentDeformerPlug()->objectHash(parentDeformerScenePath));
    }
    else
    {
        // If path is empty, hash something to signify no parent deformer
        IECore::MurmurHash emptyPathHash;
        emptyPathHash.append("emptyParentDeformerPath");
        h.append(emptyPathHash);
    }

    // Hash own plugs
    curveRootAttrPlug()->hash(h);
    useBindAttrPlug()->hash(h);
    deformerPathPlug()->hash(h);
    bindAttrPlug()->hash(h);
    upVectorPlug()->hash(h);
    useUpVectorAttrPlug()->hash(h);
    upVectorAttrPlug()->hash(h);

    // If up-vector is from an attribute, hash that attribute on the child curves
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

size_t CurvesToCurvesAttach::findRootPointIndex(
    const IECoreScene::CurvesPrimitive *curves,
    const std::vector<size_t> &vertexOffsets,
    size_t curveIndex) const
{
    const std::string rootAttrName = curveRootAttrPlug()->getValue();
    if (rootAttrName.empty() || curveIndex >= vertexOffsets.size())
    {
        return curveIndex < vertexOffsets.size() ? vertexOffsets[curveIndex] : 0; // Default to first point
    }

    auto it = curves->variables.find(rootAttrName);
    if (it == curves->variables.end() || !it->second.data)
    {
        return vertexOffsets[curveIndex];
    }

    const size_t startIdx = vertexOffsets[curveIndex];
    const size_t numVertsOnCurve = (curveIndex + 1 < vertexOffsets.size()) ? (vertexOffsets[curveIndex + 1] - startIdx) : (curves->variableSize(PrimitiveVariable::Vertex) - startIdx);

    size_t rootIdx = startIdx;
    float maxVal = -std::numeric_limits<float>::max(); // Initialize with a very small number

    // Check for FloatVectorData
    if (const FloatVectorData *rootData = runTimeCast<const FloatVectorData>(it->second.data.get()))
    {
        const std::vector<float> &values = rootData->readable();
        if (it->second.interpolation == PrimitiveVariable::Vertex || it->second.interpolation == PrimitiveVariable::Varying)
        {
            for (size_t i = 0; i < numVertsOnCurve; ++i)
            {
                const size_t currentVertexIdx = startIdx + i;
                if (currentVertexIdx < values.size() && values[currentVertexIdx] > maxVal)
                {
                    maxVal = values[currentVertexIdx];
                    rootIdx = currentVertexIdx;
                }
            }
        }
        else if (it->second.interpolation == PrimitiveVariable::Uniform && curveIndex < values.size())
        {
            // If uniform, the value applies to the whole curve, so first point is effectively the root by attribute.
            // This case might not make sense for root finding but handle defensively.
            if (values[curveIndex] > 0.0f)
                return startIdx; // Or some other logic for Uniform root
        }
    }
    // Check for IntVectorData
    else if (const IntVectorData *rootData = runTimeCast<const IntVectorData>(it->second.data.get()))
    {
        const std::vector<int> &values = rootData->readable();
        if (it->second.interpolation == PrimitiveVariable::Vertex || it->second.interpolation == PrimitiveVariable::Varying)
        {
            for (size_t i = 0; i < numVertsOnCurve; ++i)
            {
                const size_t currentVertexIdx = startIdx + i;
                if (currentVertexIdx < values.size() && static_cast<float>(values[currentVertexIdx]) > maxVal)
                {
                    maxVal = static_cast<float>(values[currentVertexIdx]);
                    rootIdx = currentVertexIdx;
                }
            }
        }
        else if (it->second.interpolation == PrimitiveVariable::Uniform && curveIndex < values.size())
        {
            if (values[curveIndex] > 0)
                return startIdx;
        }
    }

    // If maxVal is still very small, it means no point had a positive root attribute value or attr was not found/valid type.
    // In such cases, or if rootAttrName was empty, we default to the first point of the curve.
    if (maxVal <= 0.0f && !rootAttrName.empty()) // Check maxVal only if an attribute was specified
    {
        // If an attribute was specified but no vertex had a value > 0, warn or fallback gracefully.
        // For now, fallback to first point if no positive value found.
        return vertexOffsets[curveIndex];
    }
    else if (rootAttrName.empty())
    {
        return vertexOffsets[curveIndex];
    }

    return rootIdx;
}

void CurvesToCurvesAttach::computeAndStoreBindings(
    const IECoreScene::CurvesPrimitive *parentDeformerCurves,
    const IECoreScene::CurvesPrimitive *childCurves,
    IECoreScene::CurvesPrimitive *outputCurves) const
{
    if (!parentDeformerCurves || !childCurves || !outputCurves)
    {
        return;
    }

    const V3fVectorData *childPData = childCurves->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);
    if (!childPData)
    {
        // No points on child curves to attach.
        return;
    }
    const std::vector<V3f> &childPoints = childPData->readable();
    const std::vector<int> &childVertsPerCurve = childCurves->verticesPerCurve()->readable();

    if (childVertsPerCurve.empty())
    {
        return; // No child curves to process
    }

    // Precompute vertex offsets for child curves
    std::vector<size_t> childVertexOffsets;
    childVertexOffsets.reserve(childVertsPerCurve.size());
    size_t currentOffset = 0;
    for (int count : childVertsPerCurve)
    {
        childVertexOffsets.push_back(currentOffset);
        currentOffset += count;
    }

    CurvesPrimitiveEvaluatorPtr parentEvaluator = new CurvesPrimitiveEvaluator(ConstCurvesPrimitivePtr(parentDeformerCurves));
    PrimitiveEvaluator::ResultPtr parentEvalResult = parentEvaluator->createResult();

    const size_t numChildCurves = childVertsPerCurve.size();
    std::vector<CurveBindingData> bindings(numChildCurves);

    const bool useUpVecAttr = useUpVectorAttrPlug()->getValue();
    const std::string upVecAttrName = upVectorAttrPlug()->getValue();
    const V3f defaultUpVectorFromPlug = upVectorPlug()->getValue();

    IECore::msg( IECore::Msg::Debug, "CurvesToCurvesAttach::upVectorSetup", std::string("Use Attribute for UpVector: ") + (useUpVecAttr ? "True" : "False") + ". Attr name: '" + upVecAttrName + "'. Plug default: (" + std::to_string(defaultUpVectorFromPlug.x) + ", " + std::to_string(defaultUpVectorFromPlug.y) + ", " + std::to_string(defaultUpVectorFromPlug.z) + ")" );

    // Optional: Parallelize this loop if performance becomes an issue
    // const size_t numThreads = std::thread::hardware_concurrency();
    // const size_t batchSize = calculateBatchSize(numChildCurves, numThreads);
    // tbb::parallel_for( tbb::blocked_range<size_t>( 0, numChildCurves, batchSize ),
    //    [&]( const tbb::blocked_range<size_t> &range ) {
    //        PrimitiveEvaluator::ResultPtr localParentEvalResult = parentEvaluator->createResult(); // Thread-local result
    //        for( size_t i = range.begin(); i != range.end(); ++i )
    //        {
    //           ... body of loop using localParentEvalResult ...
    //        }
    //    }
    // );

    for (size_t i = 0; i < numChildCurves; ++i)
    {
        CurveBindingData &binding = bindings[i];
        const size_t rootPointVtxIdx = findRootPointIndex(childCurves, childVertexOffsets, i);

        if (rootPointVtxIdx >= childPoints.size())
        {
            binding.valid = false;
            continue;
        }
        const V3f &childRootP = childPoints[rootPointVtxIdx];

        if (!parentEvaluator->closestPoint(childRootP, parentEvalResult.get()))
        {
            binding.valid = false;
            continue;
        }

        const CurvesPrimitiveEvaluator::Result *evalRes = static_cast<const CurvesPrimitiveEvaluator::Result *>(parentEvalResult.get());

        binding.restFrame.position = evalRes->point();
        binding.restFrame.tangent = evalRes->vTangent();
        binding.parentDeformerCurveIndex = evalRes->curveIndex();
        binding.parentDeformerCurveU = evalRes->uv()[1];

        V3f finalUpVectorForCurve = defaultUpVectorFromPlug;
        std::string upVectorSource = "plug"; // Default to plug

        if (useUpVecAttr && !upVecAttrName.empty())
        {
            auto it = childCurves->variables.find(upVecAttrName);
            if (it != childCurves->variables.end() && it->second.data)
            {
                bool attributeSuccessfullyUsed = false;
                if (it->second.interpolation == PrimitiveVariable::Constant)
                {
                    if (const V3fData *upData = runTimeCast<const V3fData>(it->second.data.get()))
                    {
                        finalUpVectorForCurve = upData->readable();
                        attributeSuccessfullyUsed = true;
                    }
                }
                else if (it->second.interpolation == PrimitiveVariable::Uniform)
                {
                    if (const V3fVectorData *upVecListData = runTimeCast<const V3fVectorData>(it->second.data.get()))
                    {
                        if (i < upVecListData->readable().size())
                        {
                            finalUpVectorForCurve = upVecListData->readable()[i];
                            attributeSuccessfullyUsed = true;
                        }
                    }
                }
                else if (it->second.interpolation == PrimitiveVariable::Vertex || it->second.interpolation == PrimitiveVariable::Varying)
                {
                    if (const V3fVectorData *upVecData = runTimeCast<const V3fVectorData>(it->second.data.get()))
                    {
                        if (rootPointVtxIdx < upVecData->readable().size())
                        {
                            finalUpVectorForCurve = upVecData->readable()[rootPointVtxIdx];
                            attributeSuccessfullyUsed = true;
                        }
                    }
                }

                if (attributeSuccessfullyUsed) {
                    upVectorSource = "attribute ('" + upVecAttrName + "')";
                } else {
                    IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::upVector", "Curve " + std::to_string(i) + ": Failed to use attribute '" + upVecAttrName + "' (type/size/interp mismatch or data issue). Falling back to plug." );
                    // finalUpVectorForCurve remains defaultUpVectorFromPlug
                }
            }
            else
            {
                 IECore::msg( IECore::Msg::Debug, "CurvesToCurvesAttach::upVector", "Curve " + std::to_string(i) + ": Attribute '" + upVecAttrName + "' not found. Using plug." );
                // finalUpVectorForCurve remains defaultUpVectorFromPlug
            }
        }
        // No specific message if useUpVecAttr is true but attr name is empty, or if useUpVecAttr is false; it just uses the plug.

        IECore::msg( IECore::Msg::Debug, "CurvesToCurvesAttach::upVectorDetail", "Curve " + std::to_string(i) + ": Final up-vector source: " + upVectorSource + ". Value: (" + std::to_string(finalUpVectorForCurve.x) + ", " + std::to_string(finalUpVectorForCurve.y) + ", " + std::to_string(finalUpVectorForCurve.z) + ")" );
        binding.restFrame.orthonormalize(finalUpVectorForCurve);
        binding.rootPointOffset = childRootP - binding.restFrame.position;
        binding.valid = true;
    }

    // Store binding data as primvars
    V3fVectorDataPtr restPositionsData = new V3fVectorData;
    V3fVectorDataPtr restTangentsData = new V3fVectorData;
    V3fVectorDataPtr restNormalsData = new V3fVectorData;
    V3fVectorDataPtr restBitangentsData = new V3fVectorData;
    V3fVectorDataPtr rootPointOffsetsData = new V3fVectorData;
    IntVectorDataPtr parentCurveIndicesData = new IntVectorData;
    FloatVectorDataPtr parentCurveUsData = new FloatVectorData;

    std::vector<V3f> &restPositions = restPositionsData->writable();
    std::vector<V3f> &restTangents = restTangentsData->writable();
    std::vector<V3f> &restNormals = restNormalsData->writable();
    std::vector<V3f> &restBitangents = restBitangentsData->writable();
    std::vector<V3f> &rootPointOffsets = rootPointOffsetsData->writable();
    std::vector<int> &parentCurveIndices = parentCurveIndicesData->writable();
    std::vector<float> &parentCurveUs = parentCurveUsData->writable();

    restPositions.reserve(numChildCurves);
    restTangents.reserve(numChildCurves);
    restNormals.reserve(numChildCurves);
    restBitangents.reserve(numChildCurves);
    rootPointOffsets.reserve(numChildCurves);
    parentCurveIndices.reserve(numChildCurves);
    parentCurveUs.reserve(numChildCurves);

    for (size_t i = 0; i < numChildCurves; ++i)
    {
        const CurveBindingData &binding = bindings[i];
        if (binding.valid)
        {
            restPositions.push_back(binding.restFrame.position);
            restTangents.push_back(binding.restFrame.tangent);
            restNormals.push_back(binding.restFrame.normal);
            restBitangents.push_back(binding.restFrame.bitangent);
            rootPointOffsets.push_back(binding.rootPointOffset);
            parentCurveIndices.push_back(binding.parentDeformerCurveIndex);
            parentCurveUs.push_back(binding.parentDeformerCurveU);
        }
        else // Push default/invalid values to maintain parallel arrays
        {
            restPositions.push_back(V3f(0.0f));
            restTangents.push_back(V3f(1.0f, 0.0f, 0.0f));   // Default tangent
            restNormals.push_back(V3f(0.0f, 1.0f, 0.0f));    // Default normal
            restBitangents.push_back(V3f(0.0f, 0.0f, 1.0f)); // Default bitangent
            rootPointOffsets.push_back(V3f(0.0f));
            parentCurveIndices.push_back(-1);
            parentCurveUs.push_back(0.0f);
        }
    }

    outputCurves->variables["cfx:restPosition"] = PrimitiveVariable(PrimitiveVariable::Uniform, restPositionsData);
    outputCurves->variables["cfx:restTangent"] = PrimitiveVariable(PrimitiveVariable::Uniform, restTangentsData);
    outputCurves->variables["cfx:restNormal"] = PrimitiveVariable(PrimitiveVariable::Uniform, restNormalsData);
    outputCurves->variables["cfx:restBitangent"] = PrimitiveVariable(PrimitiveVariable::Uniform, restBitangentsData);
    outputCurves->variables["cfx:rootPointOffset"] = PrimitiveVariable(PrimitiveVariable::Uniform, rootPointOffsetsData);
    outputCurves->variables["cfx:parentCurveIndex"] = PrimitiveVariable(PrimitiveVariable::Uniform, parentCurveIndicesData);
    outputCurves->variables["cfx:parentCurveU"] = PrimitiveVariable(PrimitiveVariable::Uniform, parentCurveUsData);
}

IECore::ConstObjectPtr CurvesToCurvesAttach::computeProcessedObject(const GafferScene::ScenePlug::ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject) const
{
    const CurvesPrimitive *childCurves = runTimeCast<const CurvesPrimitive>(inputObject);
    if (!childCurves || childCurves->verticesPerCurve()->readable().empty())
    {
        return inputObject; // Not curves or no curves to process
    }

    ScenePath parentDeformerScenePath;
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
                    if (!paths.empty())
                    {
                        parentDeformerScenePath = GafferScotch::makeScenePath(paths[0]); // Use first path
                    }
                }
            }
        }
    }
    else
    {
        parentDeformerScenePath = GafferScotch::makeScenePath(deformerPathPlug()->getValue());
    }

    if (parentDeformerScenePath.empty())
    {
        // Path to parent deformer is not specified or found.
        // It might be desirable to issue a warning here.
        return inputObject;
    }

    ConstObjectPtr parentDeformerObject = parentDeformerPlug()->object(parentDeformerScenePath);
    const CurvesPrimitive *parentDeformerCurves = runTimeCast<const CurvesPrimitive>(parentDeformerObject.get());

    if (!parentDeformerCurves || parentDeformerCurves->verticesPerCurve()->readable().empty())
    {
        // Parent deformer is not valid curves or has no curves.
        // It might be desirable to issue a warning here.
        return inputObject;
    }

    // Create output curves, starting as a copy of the input child curves
    CurvesPrimitivePtr outputCurves = childCurves->copy();

    // Compute and store the binding data
    computeAndStoreBindings(parentDeformerCurves, childCurves, outputCurves.get());

    return outputCurves;
}
}