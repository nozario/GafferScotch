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
    ScenePath shaftsPath = path;
    ConstObjectPtr shaftsObject = inShaftsPlug()->object(shaftsPath);
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
    ScenePath shaftsPath = path;
    ConstObjectPtr shaftsObject = inShaftsPlug()->object(shaftsPath);
    const CurvesPrimitive *shafts = runTimeCast<const CurvesPrimitive>(shaftsObject.get());

    if (!shafts)
    {
        return inputObject;
    }

    IECore::msg(IECore::Msg::Warning, "FeatherAttachBarbs",
                (boost::format("Just before creating output curves")));
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
    IECore::msg(IECore::Msg::Warning, "FeatherAttachBarbs",
            (boost::format("Just before computeBindings")));
    try
    {
        // Compute rest frames and bindings
        computeBindings(shafts, barbs, outputBarbs.get());

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
    IECoreScene::CurvesPrimitive *outputBarbs) const
{
    // Get attribute names from plugs
    const std::string hairIdAttrName = hairIdAttrNamePlug()->getValue();
    const std::string curveParamAttrName = curveParamAttrNamePlug()->getValue();
    const std::string orientAttrName = shaftPointOrientAttrNamePlug()->getValue();

    // Try to find hairIdAttrName in shafts
    auto shaftHairIdIt = shafts->variables.find(hairIdAttrName);
    if (shaftHairIdIt == shafts->variables.end())
    {
        IECore::msg(IECore::Msg::Warning, "FeatherAttachBarbs",
                    (boost::format("Hair ID attribute '%s' not found on shafts") % hairIdAttrName).str());
        return;
    }

    // Try to find hairIdAttrName in barbs
    auto barbHairIdIt = barbs->variables.find(hairIdAttrName);
    if (barbHairIdIt == barbs->variables.end())
    {
        IECore::msg(IECore::Msg::Warning, "FeatherAttachBarbs",
                    (boost::format("Hair ID attribute '%s' not found on barbs") % hairIdAttrName).str());
        return;
    }

    // Try to find curveParamAttrName in barbs
    auto curveParamIt = barbs->variables.find(curveParamAttrName);
    if (curveParamIt == barbs->variables.end())
    {
        IECore::msg(IECore::Msg::Warning, "FeatherAttachBarbs",
                    (boost::format("Curve parameter attribute '%s' not found on barbs") % curveParamAttrName).str());   
        return;
    }

    // Get position data for shafts
    auto shaftPosIt = shafts->variables.find("P");
    if (shaftPosIt == shafts->variables.end())
    {
        IECore::msg(IECore::Msg::Warning, "FeatherAttachBarbs",
                    "Position attribute 'P' not found on shafts");   
        return;
    }

    // Get position data for barbs
    auto barbPosIt = barbs->variables.find("P");
    if (barbPosIt == barbs->variables.end())
    {
        IECore::msg(IECore::Msg::Warning, "FeatherAttachBarbs",
                    "Position attribute 'P' not found on barbs");   
        return;
    }

    // Get orientation quaternions from shafts
    auto shaftOrientIt = shafts->variables.find(orientAttrName);
    if (shaftOrientIt == shafts->variables.end())
    {
        IECore::msg(IECore::Msg::Warning, "FeatherAttachBarbs",
                    (boost::format("Orientation attribute '%s' not found on shafts") % orientAttrName).str());
        return;
    }

    // Get orientation quaternions from barbs
    auto barbOrientIt = barbs->variables.find(orientAttrName);
    if (barbOrientIt == barbs->variables.end())
    {
        IECore::msg(IECore::Msg::Warning, "FeatherAttachBarbs",
                    (boost::format("Orientation attribute '%s' not found on barbs") % orientAttrName).str());
        return;
    }

    // Now that we have the attributes, get the data
    const IntVectorData *shaftHairIds = runTimeCast<const IntVectorData>(shaftHairIdIt->second.data.get());
    const IntVectorData *barbHairIds = runTimeCast<const IntVectorData>(barbHairIdIt->second.data.get());
    const FloatVectorData *curveParams = runTimeCast<const FloatVectorData>(curveParamIt->second.data.get());
    const V3fVectorData *shaftPositions = runTimeCast<const V3fVectorData>(shaftPosIt->second.data.get());
    const V3fVectorData *barbPositions = runTimeCast<const V3fVectorData>(barbPosIt->second.data.get());
    const QuatfVectorData *shaftOrientations = runTimeCast<const QuatfVectorData>(shaftOrientIt->second.data.get());
    const QuatfVectorData *barbOrientations = runTimeCast<const QuatfVectorData>(barbOrientIt->second.data.get());


    IECore::msg(IECore::Msg::Warning, "FeatherAttachBarbs",
                (boost::format("Just before calculating shaft curve offsets")));
                
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

    IECore::msg(IECore::Msg::Warning, "FeatherAttachBarbs",
                (boost::format("Just before calculating barb curve offsets")));
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
    
    // Create a CurvesPrimitiveEvaluator for the shafts
    CurvesPrimitiveEvaluatorPtr shaftEvaluator = new CurvesPrimitiveEvaluator(shafts);
    PrimitiveEvaluator::ResultPtr shaftResult = shaftEvaluator->createResult();
    
    // Build map from hair ID to shaft curve indices
    std::map<int, std::vector<size_t>> hairIdToShaftCurveIndices;
    const std::vector<int> &shaftHairIdsArray = shaftHairIds->readable();
    
    // Hair ID is always a uniform int attribute - one per curve
    if (shaftHairIdsArray.size() == shaftVertsPerCurve.size())
    {
        for (size_t i = 0; i < shaftVertsPerCurve.size(); ++i)
        {
            hairIdToShaftCurveIndices[shaftHairIdsArray[i]].push_back(i);
        }
    }
    else
    {
        IECore::msg(IECore::Msg::Warning, "FeatherAttachBarbs",
                    (boost::format("Hair ID attribute size mismatch: expected %d, got %d") 
                     % shaftVertsPerCurve.size() % shaftHairIdsArray.size()).str());
        return;
    }
    
    // Prepare for barb processing
    const std::vector<int> &barbHairIdsArray = barbHairIds->readable();
    const std::vector<float> &curveParamsArray = curveParams->readable();
    const std::vector<V3f> &shaftPositionsArray = shaftPositions->readable();
    const std::vector<V3f> &barbPositionsArray = barbPositions->readable();
    const std::vector<Quatf> &shaftOrientationsArray = shaftOrientations->readable();
    const std::vector<Quatf> &barbOrientationsArray = barbOrientations->readable();
    
    // Create output arrays for binding data
    V3fVectorDataPtr restPositionsData = new V3fVectorData;
    V3fVectorDataPtr restNormalsData = new V3fVectorData;
    V3fVectorDataPtr restTangentsData = new V3fVectorData;
    V3fVectorDataPtr restBitangentsData = new V3fVectorData;
    IntVectorDataPtr shaftCurveIndicesData = new IntVectorData;
    FloatVectorDataPtr shaftCurveVParamsData = new FloatVectorData;
    IntVectorDataPtr shaftHairIdsData = new IntVectorData;
    V3fVectorDataPtr rootPointOffsetsData = new V3fVectorData;
    
    std::vector<V3f> &restPositions = restPositionsData->writable();
    std::vector<V3f> &restNormals = restNormalsData->writable();
    std::vector<V3f> &restTangents = restTangentsData->writable();
    std::vector<V3f> &restBitangents = restBitangentsData->writable();
    std::vector<int> &shaftCurveIndices = shaftCurveIndicesData->writable();
    std::vector<float> &shaftCurveVParams = shaftCurveVParamsData->writable();
    std::vector<int> &shaftHairIdsOut = shaftHairIdsData->writable();
    std::vector<V3f> &rootPointOffsets = rootPointOffsetsData->writable();
    
    // Pre-allocate arrays
    const size_t numBarbCurves = barbVertsPerCurve.size();
    restPositions.resize(numBarbCurves);
    restNormals.resize(numBarbCurves);
    restTangents.resize(numBarbCurves);
    restBitangents.resize(numBarbCurves);
    shaftCurveIndices.resize(numBarbCurves, -1);
    shaftCurveVParams.resize(numBarbCurves, 0.0f);
    shaftHairIdsOut.resize(numBarbCurves, -1);
    rootPointOffsets.resize(numBarbCurves);
    
    // Determine the interpolation for barb parameters
    PrimitiveVariable::Interpolation barbHairIdInterpolation = barbHairIdIt->second.interpolation;
    
    // Process each barb curve to find its binding
    const size_t numThreads = std::thread::hardware_concurrency();
    const size_t batchSize = calculateBatchSize(numBarbCurves, numThreads);
    
    parallel_for(blocked_range<size_t>(0, numBarbCurves, batchSize),
                [&](const blocked_range<size_t> &range)
                {
                    // Create thread-local evaluator result
                    PrimitiveEvaluator::ResultPtr localResult = shaftEvaluator->createResult();
                    CurvesPrimitiveEvaluator::Result *curveResult = static_cast<CurvesPrimitiveEvaluator::Result *>(localResult.get());
                    
                    for (size_t barbCurveIdx = range.begin(); barbCurveIdx != range.end(); ++barbCurveIdx)
                    {
                        // Find the root point of the barb (where curveu == 0)
                        const size_t barbOffset = barbOffsets[barbCurveIdx];
                        const int barbVertsCount = barbVertsPerCurve[barbCurveIdx];
                        size_t rootPointIdx = barbOffset;
                        
                        // Find the point with lowest curve parameter (closest to 0)
                        // curveParam is always a vertex float attribute
                        float minParam = 1.0f;
                        for (int v = 0; v < barbVertsCount; ++v)
                        {
                            size_t idx = barbOffset + v;
                            if (idx < curveParamsArray.size() && curveParamsArray[idx] < minParam)
                            {
                                minParam = curveParamsArray[idx];
                                rootPointIdx = idx;
                            }
                        }
                        
                        // Get hair ID for this barb - always uniform (one per curve)
                        int barbHairId = -1;
                        if (barbCurveIdx < barbHairIdsArray.size())
                        {
                            barbHairId = barbHairIdsArray[barbCurveIdx];
                        }
                        else
                        {
                            // Skip barbs with missing hair IDs
                            continue;
                        }
                        
                        // If we don't have a valid hairId, skip this barb
                        if (barbHairId < 0 || hairIdToShaftCurveIndices.find(barbHairId) == hairIdToShaftCurveIndices.end())
                        {
                            continue;
                        }
                        
                        // Get the root point position
                        const V3f &rootPointPos = (rootPointIdx < barbPositionsArray.size()) ? 
                                                   barbPositionsArray[rootPointIdx] : V3f(0);
                        
                        // Find the matching shaft curves for this hairId
                        const std::vector<size_t> &shaftCurveIndices = hairIdToShaftCurveIndices[barbHairId];
                        
                        // If no shaft curves match, skip this barb
                        if (shaftCurveIndices.empty())
                        {
                            continue;
                        }
                        
                        // Find the closest point on any of the matching shaft curves
                        float closestDist = std::numeric_limits<float>::max();
                        int bestShaftCurveIdx = -1;
                        float bestV = 0.0f;
                        V3f closestPoint;
                        V3f closestTangent;
                        
                        for (size_t shaftCurveIdx : shaftCurveIndices)
                        {
                            // Try to find closest point on this shaft curve
                            if (shaftEvaluator->closestPoint(rootPointPos, localResult.get(), shaftCurveIdx))
                            {
                                float dist = (curveResult->point() - rootPointPos).length();
                                if (dist < closestDist)
                                {
                                    closestDist = dist;
                                    bestShaftCurveIdx = static_cast<int>(shaftCurveIdx);
                                    bestV = curveResult->v();
                                    closestPoint = curveResult->point();
                                    // Get tangent from curve evaluation
                                    closestTangent = curveResult->vTangent().normalized();
                                }
                            }
                        }
                        
                        // If we found a binding point
                        if (bestShaftCurveIdx >= 0)
                        {
                            // Get orientation quaternion for this shaft point
                            Quatf orientation;
                            
                            // Get the vertex offset for this shaft curve
                            size_t shaftCurveVertexOffset = shaftOffsets[bestShaftCurveIdx];
                            int shaftCurveVerts = shaftVertsPerCurve[bestShaftCurveIdx];
                            
                            // Find or interpolate orientation at this point
                            if (shaftOrientationsArray.size() == shaftPositionsArray.size()) 
                            {
                                // Orientation is per-vertex
                                // Find nearest vertex for now (could do better interpolation)
                                int nearestVertexIndex = shaftCurveVertexOffset + 
                                                        std::min(static_cast<int>(bestV * (shaftCurveVerts - 1)),
                                                                shaftCurveVerts - 1);
                                                                
                                if (nearestVertexIndex < shaftOrientationsArray.size())
                                {
                                    orientation = shaftOrientationsArray[nearestVertexIndex];
                                }
                                else 
                                {
                                    // Fallback to identity quaternion
                                    orientation = Quatf();
                                }
                            }
                            else if (shaftOrientationsArray.size() == shaftVertsPerCurve.size()) 
                            {
                                // Orientation is per-curve (uniform)
                                if (bestShaftCurveIdx < shaftOrientationsArray.size())
                                {
                                    orientation = shaftOrientationsArray[bestShaftCurveIdx];
                                }
                                else
                                {
                                    // Fallback to identity quaternion
                                    orientation = Quatf();
                                }
                            }
                            else
                            {
                                // Fallback to identity quaternion
                                orientation = Quatf();
                            }
                            
                            // Build rest frame using orientation quaternion and tangent
                            RestFrame restFrame;
                            restFrame.position = closestPoint;
                            restFrame.tangent = closestTangent;
                            
                            // Apply the orientation quaternion to default up vector to get normal
                            V3f defaultNormal(0, 1, 0);
                            restFrame.normal = orientation.rotateVector(defaultNormal);
                            // Make sure normal is perpendicular to tangent
                            restFrame.normal = (restFrame.normal - restFrame.tangent * restFrame.tangent.dot(restFrame.normal)).normalized();
                            
                            // Complete the frame with the bitangent
                            restFrame.bitangent = restFrame.tangent.cross(restFrame.normal).normalized();
                            
                            // Ensure the frame is properly orthonormalized
                            restFrame.orthonormalize();
                            
                            // Calculate root point offset relative to binding point
                            V3f rootOffset = rootPointPos - closestPoint;
                            
                            // Store data
                            shaftCurveIndices[barbCurveIdx] = bestShaftCurveIdx;
                            shaftCurveVParams[barbCurveIdx] = bestV;
                            shaftHairIdsOut[barbCurveIdx] = barbHairId;
                            restPositions[barbCurveIdx] = restFrame.position;
                            restNormals[barbCurveIdx] = restFrame.normal;
                            restTangents[barbCurveIdx] = restFrame.tangent;
                            restBitangents[barbCurveIdx] = restFrame.bitangent;
                            rootPointOffsets[barbCurveIdx] = rootOffset;
                        }
                    }
                });
    
    // Store all binding data as primitive variables
    outputBarbs->variables["restPosition"] = PrimitiveVariable(PrimitiveVariable::Uniform, restPositionsData);
    outputBarbs->variables["restNormal"] = PrimitiveVariable(PrimitiveVariable::Uniform, restNormalsData);
    outputBarbs->variables["restTangent"] = PrimitiveVariable(PrimitiveVariable::Uniform, restTangentsData);
    outputBarbs->variables["restBitangent"] = PrimitiveVariable(PrimitiveVariable::Uniform, restBitangentsData);
    outputBarbs->variables["shaftCurveIndex"] = PrimitiveVariable(PrimitiveVariable::Uniform, shaftCurveIndicesData);
    outputBarbs->variables["shaftCurveV"] = PrimitiveVariable(PrimitiveVariable::Uniform, shaftCurveVParamsData);
    outputBarbs->variables["shaftHairId"] = PrimitiveVariable(PrimitiveVariable::Uniform, shaftHairIdsData);
    outputBarbs->variables["rootPointOffset"] = PrimitiveVariable(PrimitiveVariable::Uniform, rootPointOffsetsData);
    
    IECore::msg(IECore::Msg::Warning, "FeatherAttachBarbs",
                (boost::format("Completed barb binding - bound %d barbs") % numBarbCurves).str());
}
