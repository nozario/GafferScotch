#include "GafferScotch/FeatherAttachBarbs.h"
#include "GafferScotch/ScenePathUtil.h"

#include "IECoreScene/CurvesPrimitive.h"
#include "IECoreScene/PrimitiveVariable.h"

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
        int shaftPointId;
        float barbParam;
        RestFrame restFrame;
        bool valid;

        BarbBinding() : triangleIndex(-1), shaftHairId(-1), shaftPointId(-1), barbParam(0.0f), valid(false) {}
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

    // Helper to hash mesh topology
    void hashTopology(const MeshPrimitive *mesh, MurmurHash &h)
    {
        if (!mesh)
            return;

        const std::vector<int> &verticesPerFace = mesh->verticesPerFace()->readable();
        const std::vector<int> &vertexIds = mesh->vertexIds()->readable();

        h.append(verticesPerFace.size());
        if (!verticesPerFace.empty())
        {
            h.append(&verticesPerFace[0], verticesPerFace.size());
        }

        h.append(vertexIds.size());
        if (!vertexIds.empty())
        {
            h.append(&vertexIds[0], vertexIds.size());
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
    
    // Add barbs input
    addChild(new ScenePlug("inBarbs", Plug::In));

    // Attribute names for binding
    addChild(new StringPlug("hairIdAttrName", Plug::In, "hairId"));
    addChild(new StringPlug("shaftPointIdAttrName", Plug::In, "shaftPointId"));
    addChild(new StringPlug("barbParamAttrName", Plug::In, "barbParam"));
    
    // Orientation options
    addChild(new StringPlug("shaftUpVectorPrimVarName", Plug::In, "up"));
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

ScenePlug *FeatherAttachBarbs::inBarbsPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex + 1);
}

const ScenePlug *FeatherAttachBarbs::inBarbsPlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex + 1);
}

StringPlug *FeatherAttachBarbs::hairIdAttrNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 2);
}

const StringPlug *FeatherAttachBarbs::hairIdAttrNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 2);
}

StringPlug *FeatherAttachBarbs::shaftPointIdAttrNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 3);
}

const StringPlug *FeatherAttachBarbs::shaftPointIdAttrNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 3);
}

StringPlug *FeatherAttachBarbs::barbParamAttrNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 4);
}

const StringPlug *FeatherAttachBarbs::barbParamAttrNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 4);
}

StringPlug *FeatherAttachBarbs::shaftUpVectorPrimVarNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 5);
}

const StringPlug *FeatherAttachBarbs::shaftUpVectorPrimVarNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 5);
}

StringPlug *FeatherAttachBarbs::shaftPointOrientAttrNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 6);
}

const StringPlug *FeatherAttachBarbs::shaftPointOrientAttrNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 6);
}

void FeatherAttachBarbs::affects(const Gaffer::Plug *input, AffectedPlugsContainer &outputs) const
{
    ObjectProcessor::affects(input, outputs);

    if (input == inShaftsPlug()->objectPlug() ||
        input == inBarbsPlug()->objectPlug() ||
        input == hairIdAttrNamePlug() ||
        input == shaftPointIdAttrNamePlug() ||
        input == barbParamAttrNamePlug() ||
        input == shaftUpVectorPrimVarNamePlug() ||
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
           input == inBarbsPlug()->objectPlug() ||
           input == hairIdAttrNamePlug() ||
           input == shaftPointIdAttrNamePlug() ||
           input == barbParamAttrNamePlug() ||
           input == shaftUpVectorPrimVarNamePlug() ||
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
        // Try to get shafts from barbs input
        ConstObjectPtr barbsObject = inBarbsPlug()->object(path);
        const CurvesPrimitive *altBarbs = runTimeCast<const CurvesPrimitive>(barbsObject.get());
        
        if (!altBarbs)
        {
            h = inputObject->hash();
            return;
        }
    }

    // Hash input curves
    h.append(inPlug()->objectHash(path));
    
    // Hash shafts
    h.append(inShaftsPlug()->objectHash(path));
    
    // Hash barbs if different from input
    h.append(inBarbsPlug()->objectHash(path));

    // Hash parameters
    hairIdAttrNamePlug()->hash(h);
    shaftPointIdAttrNamePlug()->hash(h);
    barbParamAttrNamePlug()->hash(h);
    shaftUpVectorPrimVarNamePlug()->hash(h);
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
        // Try to get shafts from barbs input
        ConstObjectPtr barbsObject = inBarbsPlug()->object(path);
        const CurvesPrimitive *altBarbs = runTimeCast<const CurvesPrimitive>(barbsObject.get());
        
        if (!altBarbs)
        {
            return inputObject;
        }
        
        // Use alternative barbs as shafts
        shafts = altBarbs;
    }

    // Validate shafts and barbs have the necessary attributes
    const std::string hairIdAttrName = hairIdAttrNamePlug()->getValue();
    const std::string shaftPointIdAttrName = shaftPointIdAttrNamePlug()->getValue();
    const std::string barbParamAttrName = barbParamAttrNamePlug()->getValue();
    
    // Validate hairId on both shafts and barbs
    auto shaftHairIdIt = shafts->variables.find(hairIdAttrName);
    auto barbHairIdIt = barbs->variables.find(hairIdAttrName);
    if (shaftHairIdIt == shafts->variables.end() || barbHairIdIt == barbs->variables.end())
    {
        return inputObject;
    }
    
    // Validate shaft point ids on barbs
    auto barbShaftPointIdIt = barbs->variables.find(shaftPointIdAttrName);
    if (barbShaftPointIdIt == barbs->variables.end())
    {
        return inputObject;
    }
    
    // Validate barb parameter
    auto barbParamIt = barbs->variables.find(barbParamAttrName);
    if (barbParamIt == barbs->variables.end())
    {
        return inputObject;
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
        // Extract necessary data from shafts and barbs
        const IntVectorData *shaftHairIds = runTimeCast<const IntVectorData>(shaftHairIdIt->second.data.get());
        const IntVectorData *barbHairIds = runTimeCast<const IntVectorData>(barbHairIdIt->second.data.get());
        const IntVectorData *barbShaftPointIds = runTimeCast<const IntVectorData>(barbShaftPointIdIt->second.data.get());
        const FloatVectorData *barbParams = runTimeCast<const FloatVectorData>(barbParamIt->second.data.get());
        
        if (!shaftHairIds || !barbHairIds || !barbShaftPointIds || !barbParams)
        {
            return inputObject;
        }
        
        // Get position data
        const V3fVectorData *shaftPositions = shafts->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);
        const V3fVectorData *barbPositions = barbs->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);
        
        if (!shaftPositions || !barbPositions)
        {
            return inputObject;
        }
        
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

void FeatherAttachBarbs::computeBindings(
    const IECoreScene::CurvesPrimitive *shafts,
    const IECoreScene::CurvesPrimitive *barbs,
    IECoreScene::CurvesPrimitive *outputBarbs) const
{
    // Get attribute names from plugs
    const std::string hairIdAttrName = this->hairIdAttrNamePlug()->getValue();
    const std::string shaftPointIdAttrName = this->shaftPointIdAttrNamePlug()->getValue();
    const std::string barbParamAttrName = this->barbParamAttrNamePlug()->getValue();
    const std::string upVectorName = this->shaftUpVectorPrimVarNamePlug()->getValue();
    const std::string orientAttrName = this->shaftPointOrientAttrNamePlug()->getValue();
    
    // Extract data from barbs and shafts
    const IntVectorData *shaftHairIds = shafts->variableData<IntVectorData>(hairIdAttrName, PrimitiveVariable::Uniform);
    const IntVectorData *barbHairIds = barbs->variableData<IntVectorData>(hairIdAttrName, PrimitiveVariable::Uniform);
    const IntVectorData *barbShaftPointIds = barbs->variableData<IntVectorData>(shaftPointIdAttrName, PrimitiveVariable::Uniform);
    const FloatVectorData *barbParams = barbs->variableData<FloatVectorData>(barbParamAttrName, PrimitiveVariable::Uniform);
    
    if (!shaftHairIds || !barbHairIds || !barbShaftPointIds || !barbParams)
    {
        throw IECore::Exception("Missing required attribute data for binding calculation");
    }
    
    // Get positions
    const V3fVectorData *shaftPositions = shafts->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);
    const V3fVectorData *barbPositions = barbs->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);
    
    if (!shaftPositions || !barbPositions)
    {
        throw IECore::Exception("Missing position data");
    }
    
    // Get up vectors if available
    const V3fVectorData *upVectors = nullptr;
    if (!upVectorName.empty())
    {
        upVectors = shafts->variableData<V3fVectorData>(upVectorName, PrimitiveVariable::Vertex);
    }
    
    // Get orientation quaternions if available
    const QuatfVectorData *orientations = nullptr;
    if (!orientAttrName.empty())
    {
        orientations = shafts->variableData<QuatfVectorData>(orientAttrName, PrimitiveVariable::Vertex);
    }
    
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
    const std::vector<int> &barbShaftPointIdValues = barbShaftPointIds->readable();
    const std::vector<float> &barbParamValues = barbParams->readable();
    
    const std::vector<V3f> &shaftPositionValues = shaftPositions->readable();
    const std::vector<V3f> &barbPositionValues = barbPositions->readable();
    
    // Create map from hair ID to shaft curve index for quick lookup
    std::unordered_map<int, size_t> hairIdToShaftIndex;
    for (size_t i = 0; i < shaftHairIdValues.size(); ++i)
    {
        hairIdToShaftIndex[shaftHairIdValues[i]] = i;
    }
    
    // Process barbs in parallel
    const size_t numBarbs = barbVertsPerCurve.size();
    const size_t numThreads = std::thread::hardware_concurrency();
    const size_t batchSize = calculateBatchSize(numBarbs, numThreads);
    
    std::vector<BarbBinding> bindings(numBarbs);
    
    parallel_for(blocked_range<size_t>(0, numBarbs, batchSize),
                [&](const blocked_range<size_t> &range)
                {
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
                        const int shaftPointId = barbShaftPointIdValues[i];
                        const float barbParam = barbParamValues[i];
                        
                        // Validate shaft point index
                        if (shaftPointId < 0 || 
                            shaftPointId >= shaftVertsPerCurve[shaftIndex])
                        {
                            binding.valid = false;
                            continue;
                        }
                        
                        // Get shaft point global index
                        const size_t shaftPointGlobalIndex = shaftOffsets[shaftIndex] + shaftPointId;
                        
                        // Get barb root point
                        const size_t barbRootIndex = barbOffsets[i];
                        const V3f &barbRootPos = barbPositionValues[barbRootIndex];
                        
                        // Get shaft point and its local frame
                        const V3f &shaftPointPos = shaftPositionValues[shaftPointGlobalIndex];
                        
                        // Calculate tangent vector along shaft
                        V3f tangent;
                        if (shaftPointId + 1 < shaftVertsPerCurve[shaftIndex])
                        {
                            const size_t nextPointIndex = shaftOffsets[shaftIndex] + shaftPointId + 1;
                            tangent = shaftPositionValues[nextPointIndex] - shaftPointPos;
                        }
                        else if (shaftPointId > 0)
                        {
                            const size_t prevPointIndex = shaftOffsets[shaftIndex] + shaftPointId - 1;
                            tangent = shaftPointPos - shaftPositionValues[prevPointIndex];
                        }
                        else
                        {
                            // Single-point shaft, use up direction
                            tangent = V3f(0, 1, 0);
                        }
                        
                        // Compute normal (perpendicular to tangent)
                        V3f normal;
                        if (orientations)
                        {
                            // Use orientation quaternion (preferred method)
                            const Quatf &orient = orientations->readable()[shaftPointGlobalIndex];
                            V3f localNormal(1, 0, 0);
                            normal = orient.rotateVector(localNormal);
                        }
                        else
                        {
                            // Orientation is required
                            IECore::msg(IECore::Msg::Warning, "FeatherAttachBarbs", 
                                        "Orientation attribute not found. This will result in invalid frames.");
                            
                            // Use a default normal as fallback, but this won't be correct
                            normal = V3f(1, 0, 0);
                        }
                        
                        // Store binding data
                        binding.shaftHairId = barbHairId;
                        binding.shaftPointId = shaftPointId;
                        binding.barbParam = barbParam;
                        
                        // Store rest frame
                        binding.restFrame.position = shaftPointPos;
                        binding.restFrame.normal = normal;
                        binding.restFrame.tangent = safeNormalize(tangent);
                        binding.restFrame.orthonormalize();
                        
                        // Calculate barb root offset from shaft point
                        V3f rootOffset = barbRootPos - shaftPointPos;
                        
                        // Store barb root point
                        binding.triangleIndex = 0; // Not using triangles for feather barbs
                        binding.baryCoords = V3f(1, 0, 0); // Not using barycentric coords
                        binding.uvCoords = V2f(0, barbParam); // Store param in uv.y
                        binding.valid = true;
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
    IntVectorDataPtr shaftPointIdsData = new IntVectorData;
    FloatVectorDataPtr barbParamsData = new FloatVectorData;
    
    std::vector<int> &outShaftHairIds = shaftHairIdsData->writable();
    std::vector<int> &outShaftPointIds = shaftPointIdsData->writable();
    std::vector<float> &outBarbParams = barbParamsData->writable();
    
    outShaftHairIds.resize(numBarbs);
    outShaftPointIds.resize(numBarbs);
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
        outShaftPointIds[i] = binding.shaftPointId;
        outBarbParams[i] = binding.barbParam;
    }
    
    // Store all data as primitive variables
    outputBarbs->variables["restPosition"] = PrimitiveVariable(PrimitiveVariable::Uniform, restPositionsData);
    outputBarbs->variables["restNormal"] = PrimitiveVariable(PrimitiveVariable::Uniform, restNormalsData);
    outputBarbs->variables["restTangent"] = PrimitiveVariable(PrimitiveVariable::Uniform, restTangentsData);
    outputBarbs->variables["restBitangent"] = PrimitiveVariable(PrimitiveVariable::Uniform, restBitangentsData);
    
    // Store binding data
    outputBarbs->variables["shaftHairId"] = PrimitiveVariable(PrimitiveVariable::Uniform, shaftHairIdsData);
    outputBarbs->variables["shaftPointId"] = PrimitiveVariable(PrimitiveVariable::Uniform, shaftPointIdsData);
    outputBarbs->variables["barbParam"] = PrimitiveVariable(PrimitiveVariable::Uniform, barbParamsData);
}
