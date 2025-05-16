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
            IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Input tangent: " + std::to_string(tangent.x) + ", " + std::to_string(tangent.y) + ", " + std::to_string(tangent.z) );
            IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Input initialUpVectorHint: " + std::to_string(initialUpVectorHint.x) + ", " + std::to_string(initialUpVectorHint.y) + ", " + std::to_string(initialUpVectorHint.z) );

            tangent = safeNormalize(tangent);
            IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Normalized tangent: " + std::to_string(tangent.x) + ", " + std::to_string(tangent.y) + ", " + std::to_string(tangent.z) );

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
                IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Initial up vector hint was zero or tiny. Defaulting to Y-up." );
                up = V3f(0.0f, 1.0f, 0.0f);
            }
            IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Initial 'up' vector: " + std::to_string(up.x) + ", " + std::to_string(up.y) + ", " + std::to_string(up.z) );

            if (std::abs(tangent.dot(up)) > 0.9999f) // If tangent and current 'up' are collinear
            {
                IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Tangent and initial 'up' are collinear. Selecting a new 'up'." );
                // Select a new 'up' vector that is least aligned with the tangent.
                // Prefer Z-axis, then X-axis, then Y-axis to break collinearity.
                if (std::abs(tangent.z) < 0.9f) // If tangent is not strongly Z-aligned
                {
                    up = V3f(0.0f, 0.0f, 1.0f); // Try Z-axis
                }
                else if (std::abs(tangent.x) < 0.9f) // Else if tangent is not strongly X-aligned
                {
                    up = V3f(1.0f, 0.0f, 0.0f); // Try X-axis
                }
                else // Tangent must be strongly Y-aligned (or Z and X aligned, implies Y is best remaining option)
                {
                    up = V3f(0.0f, 1.0f, 0.0f); // Try Y-axis
                }
                IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "New fallback 'up': " + std::to_string(up.x) + ", " + std::to_string(up.y) + ", " + std::to_string(up.z) );

                // Safety check: if somehow still collinear
                if (std::abs(tangent.dot(up)) > 0.9999f) {
                    IECore::msg( IECore::Msg::Error, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Critical: Fallback 'up' is still collinear. Attempting direct perpendicular construction." );
                    // As a last resort, force a frame if tangent is valid.
                    if( tangent.length2() > 1e-12f ) { // Check if tangent is not zero
                        if( std::abs(tangent.x) > std::abs(tangent.z) ) {
                            up = V3f(-tangent.y, tangent.x, 0.0f); // Perpendicular in XY plane relative to tangent
                        } else {
                            up = V3f(0.0f, -tangent.z, tangent.y); // Perpendicular in YZ plane relative to tangent
                        }
                        up = safeNormalize(up);
                        // If the new 'up' is still bad (e.g. tangent was (0,0,1), first fallback was (0,0,1), direct perp up=(0,-1,0) )
                        // or if tangent was (1,0,0) first fallback (1,0,0), direct perp up=(-0,1,0)=(0,1,0)
                        // This 'up' should be good unless it became zero vector itself (e.g. tangent was purely X, Y, or Z)
                        if (up.length2() < 1e-12f || std::abs(tangent.dot(up)) > 0.9999f ) {
                             IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Directly computed 'up' failed or still collinear. Using arbitrary axis." );
                            // Final arbitrary fallback if direct computation failed
                            if(std::abs(tangent.x) < 0.9f) up = V3f(1.0f,0.0f,0.0f); // Prefer X
                            else if(std::abs(tangent.y) < 0.9f) up = V3f(0.0f,1.0f,0.0f); // Then Y
                            else up = V3f(0.0f,0.0f,1.0f); // Then Z
                        }
                    } else { // tangent itself is degenerate (should have been caught earlier)
                         this->tangent   = V3f(1.0f, 0.0f, 0.0f); // Default tangent
                         up = V3f(0.0f, 1.0f, 0.0f); // Default up
                         IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Tangent degenerate in critical fallback. Reset to default frame." );
                    }
                    IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Final resort 'up': " + std::to_string(up.x) + ", " + std::to_string(up.y) + ", " + std::to_string(up.z) );
                }
            }

            V3f calculatedNormal = tangent.cross(up);
            IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Calculated normal (before normalization): " + std::to_string(calculatedNormal.x) + ", " + std::to_string(calculatedNormal.y) + ", " + std::to_string(calculatedNormal.z) + " Length^2: " + std::to_string(calculatedNormal.length2()) );

            if (calculatedNormal.length2() < 1e-12f)
            {
                IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Normal from tangent.cross(up) is zero or tiny. Using fallback normal generation." );
                if (std::abs(tangent.x) > std::abs(tangent.z)) {
                    calculatedNormal = V3f(-tangent.y, tangent.x, 0.0f);
                } else {
                    calculatedNormal = V3f(0.0f, -tangent.z, tangent.y);
                }
                IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Fallback normal (before normalization): " + std::to_string(calculatedNormal.x) + ", " + std::to_string(calculatedNormal.y) + ", " + std::to_string(calculatedNormal.z) );
            }
            this->normal = safeNormalize(calculatedNormal);

            // Recompute bitangent to be orthogonal to both
            this->bitangent = safeNormalize(this->normal.cross(this->tangent));
            // Recompute normal to ensure orthogonality with new bitangent and original tangent
            this->normal = safeNormalize(this->tangent.cross(this->bitangent));

            IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Final tangent: " + std::to_string(this->tangent.x) + ", " + std::to_string(this->tangent.y) + ", " + std::to_string(this->tangent.z) );
            IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Final normal: " + std::to_string(this->normal.x) + ", " + std::to_string(this->normal.y) + ", " + std::to_string(this->normal.z) );
            IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::AttachFrame::orthonormalize", "Final bitangent: " + std::to_string(this->bitangent.x) + ", " + std::to_string(this->bitangent.y) + ", " + std::to_string(this->bitangent.z) );

            // Add Warning log for final frame components
            IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::AttachFrame::orthonormalize",
                boost::format("Final Output Frame: T(%f,%f,%f) N(%f,%f,%f) B(%f,%f,%f) Lengths T:%f N:%f B:%f Dots T.N:%f T.B:%f N.B:%f")
                % this->tangent.x % this->tangent.y % this->tangent.z
                % this->normal.x % this->normal.y % this->normal.z
                % this->bitangent.x % this->bitangent.y % this->bitangent.z
                % this->tangent.length() % this->normal.length() % this->bitangent.length()
                % this->tangent.dot(this->normal) % this->tangent.dot(this->bitangent) % this->normal.dot(this->bitangent)
            );
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
    const size_t defaultRootIdx = curveIndex < vertexOffsets.size() ? vertexOffsets[curveIndex] : 0;

    if (rootAttrName.empty() || curveIndex >= vertexOffsets.size())
    {
        return defaultRootIdx; // Default to first point if no attribute or invalid curveIndex
    }

    auto it = curves->variables.find(rootAttrName);
    if (it == curves->variables.end() || !it->second.data)
    {
        IECore::msg(
            IECore::Msg::Warning, "CurvesToCurvesAttach::findRootPointIndex",
            boost::format("Attribute '%s' not found on child curves. Defaulting to first point for curve %d.") % rootAttrName % curveIndex
        );
        return defaultRootIdx;
    }

    const FloatVectorData *paramData = runTimeCast<const FloatVectorData>(it->second.data.get());
    if (!paramData)
    {
        IECore::msg(
            IECore::Msg::Warning, "CurvesToCurvesAttach::findRootPointIndex",
            boost::format("Attribute '%s' is not FloatVectorData. Defaulting to first point for curve %d.") % rootAttrName % curveIndex
        );
        return defaultRootIdx;
    }

    if (it->second.interpolation != PrimitiveVariable::Vertex && it->second.interpolation != PrimitiveVariable::Varying)
    {
        IECore::msg(
            IECore::Msg::Warning, "CurvesToCurvesAttach::findRootPointIndex",
            boost::format("Attribute '%s' must have Vertex or Varying interpolation. Defaulting to first point for curve %d.") % rootAttrName % curveIndex
        );
        return defaultRootIdx;
    }

    const std::vector<float> &values = paramData->readable();
    const size_t startIdx = vertexOffsets[curveIndex];
    const size_t numVertsOnCurve = (curveIndex + 1 < vertexOffsets.size()) ? (vertexOffsets[curveIndex + 1] - startIdx) : (curves->variableSize(PrimitiveVariable::Vertex) - startIdx);

    if (numVertsOnCurve == 0)
    {
        IECore::msg(
            IECore::Msg::Warning, "CurvesToCurvesAttach::findRootPointIndex",
            boost::format("Curve %d has no vertices. Cannot find root point.") % curveIndex
        );
        return 0; // Or handle as an error appropriately
    }

    size_t rootIdx = startIdx; // Default to first point of this curve
    bool foundExactZero = false;
    float minAbsValue = std::numeric_limits<float>::max();

    for (size_t i = 0; i < numVertsOnCurve; ++i)
    {
        const size_t currentVertexIdx = startIdx + i;
        if (currentVertexIdx >= values.size())
        {
            // This should ideally not happen if primvar sizes are correct
            IECore::msg(
                IECore::Msg::Warning, "CurvesToCurvesAttach::findRootPointIndex",
                boost::format("Vertex index %d out of bounds for attribute '%s' on curve %d. Skipping remaining vertices for this curve.") % currentVertexIdx % rootAttrName % curveIndex
            );
            break; 
        }

        const float currentValue = values[currentVertexIdx];
        if (std::abs(currentValue - 0.0f) < 1e-6f) // Check for exact zero (with tolerance)
        {
            // If we find an exact zero, and haven't found one before, or if this is an earlier vertex
            // (prioritizing earlier vertices for multiple exact zeros)
            if (!foundExactZero || currentVertexIdx < rootIdx)
            {
                 rootIdx = currentVertexIdx;
                 foundExactZero = true;
                 // If we want the absolute first 0.0, we can break here.
                 // If we want to check all 0.0s and pick the one with the smallest vertex index, we continue.
                 // For now, let's take the first exact zero we find.
                 break;
            }
        }

        if (!foundExactZero) // Only look for closest if no exact zero has been found yet
        {
            if (std::abs(currentValue) < minAbsValue)
            {
                minAbsValue = std::abs(currentValue);
                rootIdx = currentVertexIdx;
            }
            // If two points are equally close to zero, prefer the one with lower index
            else if (std::abs(currentValue) == minAbsValue && currentVertexIdx < rootIdx)
            {
                 rootIdx = currentVertexIdx;
            }
        }
    }

    if (!foundExactZero && minAbsValue > 1e-5f) // Arbitrary threshold to warn if nothing is very close to 0
    {
         IECore::msg(
            IECore::Msg::Warning, "CurvesToCurvesAttach::findRootPointIndex",
            boost::format("No vertex with value ~0.0 for attribute '%s' on curve %d. Closest value was %f at vertex index %d (absolute index).") % rootAttrName % curveIndex % values[rootIdx] % rootIdx
        );
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

    IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::upVectorSetup", std::string("Use Attribute for UpVector: ") + (useUpVecAttr ? "True" : "False") + ". Attr name: '" + upVecAttrName + "'. Plug default: (" + std::to_string(defaultUpVectorFromPlug.x) + ", " + std::to_string(defaultUpVectorFromPlug.y) + ", " + std::to_string(defaultUpVectorFromPlug.z) + ")" );

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
                 IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::upVector", "Curve " + std::to_string(i) + ": Attribute '" + upVecAttrName + "' not found. Using plug." );
                // finalUpVectorForCurve remains defaultUpVectorFromPlug
            }
        }
        // No specific message if useUpVecAttr is true but attr name is empty, or if useUpVecAttr is false; it just uses the plug.

        IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::upVectorDetail", "Curve " + std::to_string(i) + ": Final up-vector source: " + upVectorSource + ". Value: (" + std::to_string(finalUpVectorForCurve.x) + ", " + std::to_string(finalUpVectorForCurve.y) + ", " + std::to_string(finalUpVectorForCurve.z) + ")" );
        binding.restFrame.orthonormalize(finalUpVectorForCurve);
        binding.rootPointOffset = childRootP - binding.restFrame.position;
        binding.valid = true;

        // Add Warning log for the computed binding restFrame
        IECore::msg( IECore::Msg::Warning, "CurvesToCurvesAttach::computeAndStoreBindings",
            boost::format("Curve %d: Computed Binding: Valid:%s P(%f,%f,%f) T(%f,%f,%f) N(%f,%f,%f) B(%f,%f,%f) RootOffset(%f,%f,%f) ParentCurveIdx:%d ParentU:%f UpVecSrc:%s UpVec(%f,%f,%f)")
            % i % (binding.valid ? "true" : "false")
            % binding.restFrame.position.x % binding.restFrame.position.y % binding.restFrame.position.z
            % binding.restFrame.tangent.x % binding.restFrame.tangent.y % binding.restFrame.tangent.z
            % binding.restFrame.normal.x % binding.restFrame.normal.y % binding.restFrame.normal.z
            % binding.restFrame.bitangent.x % binding.restFrame.bitangent.y % binding.restFrame.bitangent.z
            % binding.rootPointOffset.x % binding.rootPointOffset.y % binding.rootPointOffset.z
            % binding.parentDeformerCurveIndex % binding.parentDeformerCurveU
            % upVectorSource % finalUpVectorForCurve.x % finalUpVectorForCurve.y % finalUpVectorForCurve.z
        );
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