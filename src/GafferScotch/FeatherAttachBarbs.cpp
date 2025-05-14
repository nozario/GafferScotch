#include "GafferScotch/FeatherAttachBarbs.h"
#include "GafferScotch/ScenePathUtil.h"

#include "IECoreScene/CurvesPrimitive.h"
#include "IECoreScene/PrimitiveVariable.h"
#include "IECoreScene/CurvesPrimitiveEvaluator.h"

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <thread>

using namespace Gaffer;
using namespace GafferScene;
using namespace GafferScotch;
using namespace IECore;
using namespace IECoreScene;
using namespace Imath;
using namespace tbb;

namespace
{
    // Helper function for safe vector normalization
    inline V3f safeNormalize(const V3f &v)
    {
        float length = v.length();
        if (length > 0.0f)
        {
            return v / length;
        }
        return v;
    }

    // Helper to determine optimal batch size
    inline size_t calculateBatchSize(size_t numElements, size_t numThreads)
    {
        // Aim for ~4 batches per thread for good load balancing
        const size_t targetBatches = numThreads * 4;
        return std::max(size_t(1), numElements / targetBatches);
    }

    struct RestFrame
    {
        V3f position;
        V3f normal;
        V3f tangent;
        V3f bitangent;

        void orthonormalize()
        {
            // First, normalize the normal to ensure we have a unit vector
            normal = safeNormalize(normal);

            // Make tangent orthogonal to normal and normalize it
            tangent = safeNormalize(tangent - normal * (tangent.dot(normal)));

            // Use cross product to get a perfectly orthogonal bitangent
            bitangent = normal.cross(tangent);
        }
    };

    struct BarbBinding
    {
        int triangleIndex;
        V3f baryCoords;
        V2f uvCoords;
        int shaftHairId;
        int shaftCurveIndex;
        float shaftCurveV;  // Parametric position along shaft curve
        float barbParam;
        RestFrame restFrame;
        bool valid;

        BarbBinding() : triangleIndex(-1), shaftHairId(-1), shaftCurveIndex(-1), shaftCurveV(0.0f), barbParam(0.0f), valid(false) {}
    };

    // Helper to get hash for positions only
    void hashPositions(const Primitive *primitive, MurmurHash &h)
    {
        if (!primitive)
            return;

        auto it = primitive->variables.find("P");
        if (it == primitive->variables.end())
            return;

        const V3fVectorData *positions = runTimeCast<const V3fVectorData>(it->second.data.get());
        if (!positions)
            return;

        // Hash only the number of vertices and position data
        const std::vector<V3f> &pos = positions->readable();
        h.append(pos.size());
        if (!pos.empty())
        {
            h.append(&pos[0], pos.size());
        }
    }

    // Helper to hash curves topology
    void hashTopology(const CurvesPrimitive *curves, MurmurHash &h)
    {
        if (!curves)
            return;

        const std::vector<int> &verticesPerCurve = curves->verticesPerCurve()->readable();

        h.append(verticesPerCurve.size());
        if (!verticesPerCurve.empty())
        {
            h.append(&verticesPerCurve[0], verticesPerCurve.size());
        }
    }
}

IE_CORE_DEFINERUNTIMETYPED(FeatherAttachBarbs);

size_t FeatherAttachBarbs::g_firstPlugIndex = 0;

FeatherAttachBarbs::FeatherAttachBarbs(const std::string &name)
    : ObjectProcessor(name)
{
    storeIndexOfNextChild(g_firstPlugIndex);

    // Add shafts input
    addChild(new ScenePlug("inShafts", Plug::In));

    // Attribute names for binding
    addChild(new StringPlug("hairIdAttrName", Plug::In, "hairId"));
    addChild(new StringPlug("curveParamAttrName", Plug::In, "curveu"));

    // Orientation options
    addChild(new StringPlug("shaftPointOrientAttrName", Plug::In, "orient"));

    // Fast pass-throughs for things we don't modify
    outPlug()->attributesPlug()->setInput(inPlug()->attributesPlug());
    outPlug()->transformPlug()->setInput(inPlug()->transformPlug());
    outPlug()->boundPlug()->setInput(inPlug()->boundPlug());
}

FeatherAttachBarbs::~FeatherAttachBarbs()
{
}

ScenePlug *FeatherAttachBarbs::inShaftsPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

const ScenePlug *FeatherAttachBarbs::inShaftsPlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

StringPlug *FeatherAttachBarbs::hairIdAttrNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 1);
}

const StringPlug *FeatherAttachBarbs::hairIdAttrNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 1);
}

StringPlug *FeatherAttachBarbs::curveParamAttrNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 2);
}

const StringPlug *FeatherAttachBarbs::curveParamAttrNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 2);
}

StringPlug *FeatherAttachBarbs::shaftPointOrientAttrNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 3);
}

const StringPlug *FeatherAttachBarbs::shaftPointOrientAttrNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 3);
}

void FeatherAttachBarbs::affects(const Gaffer::Plug *input, AffectedPlugsContainer &outputs) const
{
    ObjectProcessor::affects(input, outputs);

    if (input == inShaftsPlug()->objectPlug() ||
        input == hairIdAttrNamePlug() ||
        input == curveParamAttrNamePlug() ||
        input == shaftPointOrientAttrNamePlug())
    {
        outputs.push_back(outPlug()->objectPlug());
    }
}

bool FeatherAttachBarbs::acceptsInput(const Gaffer::Plug *plug, const Gaffer::Plug *inputPlug) const
{
    if (!ObjectProcessor::acceptsInput(plug, inputPlug))
    {
        return false;
    }

    return true;
}

bool FeatherAttachBarbs::affectsProcessedObject(const Gaffer::Plug *input) const
{
    return input == inShaftsPlug()->objectPlug() ||
           input == hairIdAttrNamePlug() ||
           input == curveParamAttrNamePlug() ||
           input == shaftPointOrientAttrNamePlug();
}

void FeatherAttachBarbs::hashProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    // Get objects
    ConstObjectPtr inputObject = inPlug()->object(path);
    const CurvesPrimitive *barbs = runTimeCast<const CurvesPrimitive>(inputObject.get());

    if (!barbs)
    {
        h = inputObject->hash();
        return;
    }

    // Get shafts using path
    ConstObjectPtr shaftsObject = inShaftsPlug()->object(path);
    const CurvesPrimitive *shafts = runTimeCast<const CurvesPrimitive>(shaftsObject.get());

    if (!shafts)
    {
        h = inputObject->hash();
        return;
    }

    // Hash input curves - only positions and topology are needed
    hashPositions(barbs, h);
    hashTopology(barbs, h);

    // Hash shafts - only positions and topology are needed
    hashPositions(shafts, h);
    hashTopology(shafts, h);

    // Hash parameters
    hairIdAttrNamePlug()->hash(h);
    curveParamAttrNamePlug()->hash(h);
    shaftPointOrientAttrNamePlug()->hash(h);
}

IECore::ConstObjectPtr FeatherAttachBarbs::computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject) const
{
    // Early out if we don't have a valid input object
    const CurvesPrimitive *barbs = runTimeCast<const CurvesPrimitive>(inputObject);
    if (!barbs)
    {
        return inputObject;
    }

    // Get shafts using path
    ConstObjectPtr shaftsObject = inShaftsPlug()->object(path);
    const CurvesPrimitive *shafts = runTimeCast<const CurvesPrimitive>(shaftsObject.get());

    if (!shafts)
    {
        return inputObject;
    }

    // Get attribute names from plugs
    const std::string hairIdAttrName = hairIdAttrNamePlug()->getValue();
    const std::string curveParamAttrName = curveParamAttrNamePlug()->getValue();
    const std::string orientAttrName = shaftPointOrientAttrNamePlug()->getValue();

    // Extract data from barbs and shafts
    const IntVectorData *shaftHairIds = shafts->variableData<IntVectorData>(hairIdAttrName, PrimitiveVariable::Uniform);
    const IntVectorData *barbHairIds = barbs->variableData<IntVectorData>(hairIdAttrName, PrimitiveVariable::Uniform);
    const FloatVectorData *curveParams = barbs->variableData<FloatVectorData>(curveParamAttrName, PrimitiveVariable::Vertex);

    // Validate all required attributes are present
    if (!shaftHairIds)
    {
        IECore::msg(IECore::Msg::Warning, "FeatherAttachBarbs",
                    (boost::format("Hair ID attribute '%s' not found on shafts") % hairIdAttrName).str());
        return inputObject;
    }
    
    if (!barbHairIds)
    {
        IECore::msg(IECore::Msg::Warning, "FeatherAttachBarbs",
                    (boost::format("Hair ID attribute '%s' not found on barbs") % hairIdAttrName).str());
        return inputObject;
    }

    if (!curveParams)
    {
        IECore::msg(IECore::Msg::Warning, "FeatherAttachBarbs",
                    (boost::format("Curve parameter attribute '%s' not found on barbs") % curveParamAttrName).str());
        return inputObject;
    }

    // Get position data
    const V3fVectorData *shaftPositions = shafts->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);
    const V3fVectorData *barbPositions = barbs->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);

    if (!shaftPositions || !barbPositions)
    {
        IECore::msg(IECore::Msg::Warning, "FeatherAttachBarbs", "Missing position data on shafts or barbs");
        return inputObject;
    }

    // Get orientation quaternions if available
    const QuatfVectorData *orientations = nullptr;
    if (!orientAttrName.empty())
    {
        orientations = shafts->variableData<QuatfVectorData>(orientAttrName, PrimitiveVariable::Vertex);
    }

    // Create output curves with same topology
    CurvesPrimitivePtr outputBarbs = new CurvesPrimitive(
        barbs->verticesPerCurve(),
        barbs->basis(),
        barbs->periodic());

    // Copy primitive variables
    for (const auto &primVar : barbs->variables)
    {
        outputBarbs->variables[primVar.first] = primVar.second;
    }

    // Compute bindings
    try
    {
        // Compute rest frames and bindings
        computeBindings(
            shafts, barbs, outputBarbs.get(),
            shaftHairIds, barbHairIds, curveParams,
            shaftPositions, barbPositions,
            orientations
        );

        return outputBarbs;
    }
    catch (const std::exception &e)
    {
        IECore::msg(IECore::Msg::Error, "FeatherAttachBarbs",
                    std::string("Computation failed: ") + e.what());
        return inputObject;
    }
    catch (...)
    {
        IECore::msg(IECore::Msg::Error, "FeatherAttachBarbs", "Unknown error in computation");
        return inputObject;
    }
}

void GafferScotch::FeatherAttachBarbs::computeBindings(
    const IECoreScene::CurvesPrimitive *shafts,
    const IECoreScene::CurvesPrimitive *barbs,
    IECoreScene::CurvesPrimitive *outputBarbs,
    const IntVectorData *shaftHairIds,
    const IntVectorData *barbHairIds,
    const FloatVectorData *curveParams,
    const V3fVectorData *shaftPositions,
    const V3fVectorData *barbPositions,
    const QuatfVectorData *orientations) const
{
    // Get attribute names from plugs
    const std::string hairIdAttrName = hairIdAttrNamePlug()->getValue();
    const std::string curveParamAttrName = curveParamAttrNamePlug()->getValue();

    // Calculate shaft curve offsets
    const std::vector<int> &shaftVertsPerCurve = shafts->verticesPerCurve()->readable();
    std::vector<size_t> shaftOffsets;
    shaftOffsets.reserve(shaftVertsPerCurve.size());

    size_t offset = 0;
    for (int count : shaftVertsPerCurve)
    {
        shaftOffsets.push_back(offset);
        offset += count;
    }

    // Calculate barb curve offsets
    const std::vector<int> &barbVertsPerCurve = barbs->verticesPerCurve()->readable();
    std::vector<size_t> barbOffsets;
    barbOffsets.reserve(barbVertsPerCurve.size());

    offset = 0;
    for (int count : barbVertsPerCurve)
    {
        barbOffsets.push_back(offset);
        offset += count;
    }

    // Access data
    const std::vector<int> &shaftHairIdValues = shaftHairIds->readable();
    const std::vector<int> &barbHairIdValues = barbHairIds->readable();
    const std::vector<float> &curveParamValues = curveParams->readable();

    const std::vector<V3f> &shaftPositionValues = shaftPositions->readable();
    const std::vector<V3f> &barbPositionValues = barbPositions->readable();

    // Create map from hair ID to shaft curve index for quick lookup
    std::unordered_map<int, size_t> hairIdToShaftIndex;
    for (size_t i = 0; i < shaftHairIdValues.size(); ++i)
    {
        hairIdToShaftIndex[shaftHairIdValues[i]] = i;
    }

    // Create CurvesPrimitiveEvaluator for shafts
    CurvesPrimitiveEvaluatorPtr shaftEvaluator = new CurvesPrimitiveEvaluator(shafts);
    PrimitiveEvaluator::ResultPtr result = shaftEvaluator->createResult();

    // Process barbs in parallel
    const size_t numBarbs = barbVertsPerCurve.size();
    const size_t numThreads = std::thread::hardware_concurrency();
    const size_t batchSize = calculateBatchSize(numBarbs, numThreads);

    std::vector<BarbBinding> bindings(numBarbs);

    parallel_for(blocked_range<size_t>(0, numBarbs, batchSize),
                 [&](const blocked_range<size_t> &range)
                 {
                     // Each thread needs its own evaluator and result
                     CurvesPrimitiveEvaluatorPtr threadShaftEvaluator = new CurvesPrimitiveEvaluator(shafts);
                     PrimitiveEvaluator::ResultPtr threadResult = threadShaftEvaluator->createResult();
                     
                     for (size_t i = range.begin(); i != range.end(); ++i)
                     {
                         BarbBinding &binding = bindings[i];

                         // Find matching shaft using hair ID
                         const int barbHairId = barbHairIdValues[i];
                         auto shaftIt = hairIdToShaftIndex.find(barbHairId);

                         if (shaftIt == hairIdToShaftIndex.end())
                         {
                             // No matching shaft found
                             binding.valid = false;
                             continue;
                         }

                         const size_t shaftIndex = shaftIt->second;

                         // Find barb root point (where param == 0)
                         const size_t barbOffset = barbOffsets[i];
                         const int numBarbPoints = barbVertsPerCurve[i];
                         
                         // Find the point with minimum parametric value
                         float minParam = std::numeric_limits<float>::max();
                         size_t rootPointIndex = barbOffset;
                         
                         for (int j = 0; j < numBarbPoints; ++j)
                         {
                             const size_t pointIndex = barbOffset + j;
                             const float param = curveParamValues[pointIndex];
                             if (param < minParam)
                             {
                                 minParam = param;
                                 rootPointIndex = pointIndex;
                             }
                         }
                         
                         // Get barb root position
                         const V3f &barbRootPos = barbPositionValues[rootPointIndex];
                         
                         // Find closest point on the matching shaft curve using the evaluator
                         bool found = false;
                         
                         // We must look at the specific shaft curve matching this barb's hair ID
                         threadShaftEvaluator->closestPoint(barbRootPos, threadResult.get());
                         
                         // Verify this is the correct curve - if not, try a direct approach
                         if (threadResult->curveIndex() != shaftIndex)
                         {
                             // If the closest point is on the wrong curve, we'll try to find a point directly on the right curve
                             // Get parametric position along the correct curve - use 0.5 as a starting point
                             found = threadShaftEvaluator->pointAtV(shaftIndex, 0.5f, threadResult.get());
                             
                             // Now use the general closestPoint method which will find the closest point on all curves
                             if (found)
                             {
                                 threadShaftEvaluator->closestPoint(barbRootPos, threadResult.get());
                                 found = (threadResult->curveIndex() == shaftIndex);
                             }
                         }
                         else
                         {
                             found = true;
                         }

                         if (found)
                         {
                             // Get information about the closest point
                             const V3f shaftPointPos = threadResult->point();
                             const int curveIndex = threadResult->curveIndex();
                             const float v = threadResult->uv()[1];  // Parametric position (0-1) along the curve
                            
                             // Get tangent at the closest point
                             V3f tangent = threadResult->vTangent();
                            
                             // Compute normal using orientation quaternion
                             V3f normal;
                             if (orientations)
                             {
                                 // Find the nearest shaft point to get its orientation
                                 // This is a simple implementation - for production code, 
                                 // you might want to interpolate orientations based on v
                                 const size_t shaftOffset = shaftOffsets[curveIndex];
                                 const int numShaftPoints = shaftVertsPerCurve[curveIndex];
                                 
                                 // Simple approach: use the orientation from the nearest vertex
                                 int nearestPointIdx = static_cast<int>(v * (numShaftPoints - 1) + 0.5f);
                                 nearestPointIdx = std::min(std::max(0, nearestPointIdx), numShaftPoints - 1);
                                 
                                 const size_t orientIndex = shaftOffset + nearestPointIdx;
                                 
                                 const Quatf &orient = orientations->readable()[orientIndex];
                                 V3f localNormal(1, 0, 0);
                                 normal = orient.rotateVector(localNormal);
                             }
                             else
                             {
                                 // Orientation is required
                                 IECore::msg(IECore::Msg::Warning, "FeatherAttachBarbs",
                                             "Orientation attribute not found. This will result in invalid frames.");
                                 
                                 // Fallback to a default normal perpendicular to tangent
                                 if (tangent.length() > 0.0001f)
                                 {
                                     tangent.normalize();
                                     // Find a perpendicular vector
                                     V3f perp = tangent.cross(V3f(0, 1, 0));
                                     if (perp.length() < 0.0001f)
                                     {
                                         perp = tangent.cross(V3f(1, 0, 0));
                                     }
                                     normal = perp.normalized();
                                 }
                                 else
                                 {
                                     normal = V3f(1, 0, 0);
                                 }
                             }
                            
                             // Store binding data
                             binding.shaftHairId = barbHairId;
                             binding.shaftCurveIndex = curveIndex;
                             binding.shaftCurveV = v;
                             binding.barbParam = minParam;
                            
                             // Store rest frame
                             binding.restFrame.position = shaftPointPos;
                             binding.restFrame.normal = normal;
                             binding.restFrame.tangent = safeNormalize(tangent);
                             binding.restFrame.orthonormalize();
                            
                             // Store extra information for convenience
                             binding.triangleIndex = 0;     // Not using triangles for feather barbs
                             binding.baryCoords = V3f(1, 0, 0);  // Not using barycentric coords
                             binding.uvCoords = V2f(0, v);  // Store shaft curve v parameter
                             binding.valid = true;
                         }
                         else
                         {
                             // Failed to find closest point
                             binding.valid = false;
                             IECore::msg(IECore::Msg::Warning, "FeatherAttachBarbs",
                                         (boost::format("Failed to find closest point on shaft curve %d for barb %d")
                                          % shaftIndex % i).str());
                         }
                     }
                 });

    // Store binding data as primitive variables
    V3fVectorDataPtr restPositionsData = new V3fVectorData;
    V3fVectorDataPtr restNormalsData = new V3fVectorData;
    V3fVectorDataPtr restTangentsData = new V3fVectorData;
    V3fVectorDataPtr restBitangentsData = new V3fVectorData;

    std::vector<V3f> &restPositions = restPositionsData->writable();
    std::vector<V3f> &restNormals = restNormalsData->writable();
    std::vector<V3f> &restTangents = restTangentsData->writable();
    std::vector<V3f> &restBitangents = restBitangentsData->writable();

    // Pre-allocate arrays
    restPositions.resize(numBarbs);
    restNormals.resize(numBarbs);
    restTangents.resize(numBarbs);
    restBitangents.resize(numBarbs);

    // Store optimization data
    IntVectorDataPtr shaftHairIdsData = new IntVectorData;
    IntVectorDataPtr shaftCurveIndicesData = new IntVectorData;
    FloatVectorDataPtr shaftCurveVParams = new FloatVectorData;
    FloatVectorDataPtr barbParamsData = new FloatVectorData;

    std::vector<int> &outShaftHairIds = shaftHairIdsData->writable();
    std::vector<int> &outShaftCurveIndices = shaftCurveIndicesData->writable();
    std::vector<float> &outShaftCurveVs = shaftCurveVParams->writable();
    std::vector<float> &outBarbParams = barbParamsData->writable();

    outShaftHairIds.resize(numBarbs);
    outShaftCurveIndices.resize(numBarbs);
    outShaftCurveVs.resize(numBarbs);
    outBarbParams.resize(numBarbs);

    // Copy data from bindings
    for (size_t i = 0; i < numBarbs; ++i)
    {
        const BarbBinding &binding = bindings[i];
        if (!binding.valid)
            continue;

        restPositions[i] = binding.restFrame.position;
        restNormals[i] = binding.restFrame.normal;
        restTangents[i] = binding.restFrame.tangent;
        restBitangents[i] = binding.restFrame.bitangent;

        outShaftHairIds[i] = binding.shaftHairId;
        outShaftCurveIndices[i] = binding.shaftCurveIndex;
        outShaftCurveVs[i] = binding.shaftCurveV;
        outBarbParams[i] = binding.barbParam;
    }

    // Store all data as primitive variables with consistent naming
    outputBarbs->variables["restPosition"] = PrimitiveVariable(PrimitiveVariable::Uniform, restPositionsData);
    outputBarbs->variables["restNormal"] = PrimitiveVariable(PrimitiveVariable::Uniform, restNormalsData);
    outputBarbs->variables["restTangent"] = PrimitiveVariable(PrimitiveVariable::Uniform, restTangentsData);
    outputBarbs->variables["restBitangent"] = PrimitiveVariable(PrimitiveVariable::Uniform, restBitangentsData);

    // Store curve-specific data for later use by deformers
    outputBarbs->variables["shaftHairId"] = PrimitiveVariable(PrimitiveVariable::Uniform, shaftHairIdsData);
    outputBarbs->variables["shaftCurveIndex"] = PrimitiveVariable(PrimitiveVariable::Uniform, shaftCurveIndicesData);
    outputBarbs->variables["shaftCurveV"] = PrimitiveVariable(PrimitiveVariable::Uniform, shaftCurveVParams);
    outputBarbs->variables["barbParam"] = PrimitiveVariable(PrimitiveVariable::Uniform, barbParamsData);
}
