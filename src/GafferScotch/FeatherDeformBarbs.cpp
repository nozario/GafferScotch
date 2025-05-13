#include "GafferScotch/FeatherDeformBarbs.h"
#include "GafferScotch/ScenePathUtil.h"

#include "IECoreScene/CurvesPrimitive.h"
#include "IECoreScene/PrimitiveVariable.h"

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

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
            normal = safeNormalize(normal);
            tangent = safeNormalize(tangent - normal * (tangent.dot(normal)));
            bitangent = normal.cross(tangent);
        }

        M44f toMatrix() const
        {
            M44f matrix;
            // Set rotation part (column-major order)
            matrix[0][0] = tangent.x;
            matrix[1][0] = tangent.y;
            matrix[2][0] = tangent.z;
            matrix[3][0] = 0;
            matrix[0][1] = bitangent.x;
            matrix[1][1] = bitangent.y;
            matrix[2][1] = bitangent.z;
            matrix[3][1] = 0;
            matrix[0][2] = normal.x;
            matrix[1][2] = normal.y;
            matrix[2][2] = normal.z;
            matrix[3][2] = 0;
            // Set translation part
            matrix[0][3] = position.x;
            matrix[1][3] = position.y;
            matrix[2][3] = position.z;
            matrix[3][3] = 1;
            return matrix;
        }
    };

    // Helper to transform a point from one frame to another
    V3f transformPoint(const V3f &point, const M44f &sourceToWorld, const M44f &worldToTarget)
    {
        // Transform to world space
        V3f worldPoint;
        sourceToWorld.multVecMatrix(point, worldPoint);
        worldPoint += V3f(sourceToWorld[0][3], sourceToWorld[1][3], sourceToWorld[2][3]);

        // Transform to target space
        V3f targetPoint;
        worldToTarget.multVecMatrix(worldPoint - V3f(worldToTarget[0][3], worldToTarget[1][3], worldToTarget[2][3]), targetPoint);
        targetPoint += V3f(worldToTarget[0][3], worldToTarget[1][3], worldToTarget[2][3]);

        return targetPoint;
    }

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

    // Helper to hash binding data
    void hashBindingData(const Primitive *primitive, MurmurHash &h)
    {
        if (!primitive)
            return;

        // Hash all binding attributes
        const char *bindingAttrs[] = {
            "restPosition", "restNormal", "restTangent", "restBitangent",
            "shaftHairId", "shaftPointId", "barbParam"};

        for (const char *attrName : bindingAttrs)
        {
            auto it = primitive->variables.find(attrName);
            if (it != primitive->variables.end())
            {
                it->second.data->hash(h);
            }
        }
    }
}

IE_CORE_DEFINERUNTIMETYPED(FeatherDeformBarbs);

size_t FeatherDeformBarbs::g_firstPlugIndex = 0;

FeatherDeformBarbs::FeatherDeformBarbs(const std::string &name)
    : Deformer(name)
{
    storeIndexOfNextChild(g_firstPlugIndex);

    // Add animated shafts input
    addChild(new ScenePlug("animatedShafts", Plug::In));

    // Attribute names for binding
    addChild(new StringPlug("hairIdAttrName", Plug::In, "hairId"));
    
    // Orientation options
    addChild(new StringPlug("shaftUpVectorPrimVarName", Plug::In, "up"));
    addChild(new StringPlug("shaftPointOrientAttrName", Plug::In, "orient"));
    
    // Cleanup option
    addChild(new BoolPlug("cleanupBindAttributes", Plug::In, true));

    // Fast pass-throughs for things we don't modify
    outPlug()->attributesPlug()->setInput(inPlug()->attributesPlug());
    outPlug()->transformPlug()->setInput(inPlug()->transformPlug());
    outPlug()->boundPlug()->setInput(inPlug()->boundPlug());
}

FeatherDeformBarbs::~FeatherDeformBarbs()
{
}

ScenePlug *FeatherDeformBarbs::animatedShaftsPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

const ScenePlug *FeatherDeformBarbs::animatedShaftsPlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

StringPlug *FeatherDeformBarbs::hairIdAttrNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 1);
}

const StringPlug *FeatherDeformBarbs::hairIdAttrNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 1);
}

StringPlug *FeatherDeformBarbs::shaftUpVectorPrimVarNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 2);
}

const StringPlug *FeatherDeformBarbs::shaftUpVectorPrimVarNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 2);
}

StringPlug *FeatherDeformBarbs::shaftPointOrientAttrNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 3);
}

const StringPlug *FeatherDeformBarbs::shaftPointOrientAttrNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 3);
}

BoolPlug *FeatherDeformBarbs::cleanupBindAttributesPlug()
{
    return getChild<BoolPlug>(g_firstPlugIndex + 4);
}

const BoolPlug *FeatherDeformBarbs::cleanupBindAttributesPlug() const
{
    return getChild<BoolPlug>(g_firstPlugIndex + 4);
}

void FeatherDeformBarbs::affects(const Gaffer::Plug *input, AffectedPlugsContainer &outputs) const
{
    Deformer::affects(input, outputs);

    if (input == animatedShaftsPlug()->objectPlug() ||
        input == hairIdAttrNamePlug() ||
        input == shaftUpVectorPrimVarNamePlug() ||
        input == shaftPointOrientAttrNamePlug() ||
        input == cleanupBindAttributesPlug())
    {
        outputs.push_back(outPlug()->objectPlug());
    }
}

bool FeatherDeformBarbs::acceptsInput(const Gaffer::Plug *plug, const Gaffer::Plug *inputPlug) const
{
    if (!Deformer::acceptsInput(plug, inputPlug))
    {
        return false;
    }

    return true;
}

bool FeatherDeformBarbs::affectsProcessedObject(const Gaffer::Plug *input) const
{
    return input == animatedShaftsPlug()->objectPlug() ||
           input == hairIdAttrNamePlug() ||
           input == shaftUpVectorPrimVarNamePlug() ||
           input == shaftPointOrientAttrNamePlug() ||
           input == cleanupBindAttributesPlug();
}

bool FeatherDeformBarbs::affectsProcessedObjectBound(const Gaffer::Plug *input) const
{
    return input == animatedShaftsPlug()->objectPlug() ||
           input == hairIdAttrNamePlug() ||
           input == shaftUpVectorPrimVarNamePlug() ||
           input == shaftPointOrientAttrNamePlug();
}

void FeatherDeformBarbs::hashProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    // Get objects
    ConstObjectPtr inputObject = inPlug()->object(path);
    const CurvesPrimitive *barbs = runTimeCast<const CurvesPrimitive>(inputObject.get());

    if (!barbs)
    {
        h = inputObject->hash();
        return;
    }

    // Get animated shafts
    ConstObjectPtr animatedShaftsObj = animatedShaftsPlug()->object(path);
    if (!animatedShaftsObj)
    {
        h = inputObject->hash();
        return;
    }
    
    const CurvesPrimitive *animatedShafts = runTimeCast<const CurvesPrimitive>(animatedShaftsObj.get());
    if (!animatedShafts)
    {
        h = inputObject->hash();
        return;
    }

    // Hash input barbs
    h.append(inPlug()->objectHash(path));
    
    // Hash animated shafts
    h.append(animatedShaftsPlug()->objectHash(path));
    
    // Hash binding data
    hashBindingData(barbs, h);

    // Hash parameters
    hairIdAttrNamePlug()->hash(h);
    shaftUpVectorPrimVarNamePlug()->hash(h);
    shaftPointOrientAttrNamePlug()->hash(h);
    cleanupBindAttributesPlug()->hash(h);
}

void FeatherDeformBarbs::hashProcessedObjectBound(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    // Get objects
    ConstObjectPtr inputObject = inPlug()->object(path);
    const CurvesPrimitive *barbs = runTimeCast<const CurvesPrimitive>(inputObject.get());

    if (!barbs)
    {
        h = inPlug()->boundHash(path);
        return;
    }

    // Get animated shafts
    ConstObjectPtr animatedShaftsObj = animatedShaftsPlug()->object(path);
    if (!animatedShaftsObj)
    {
        h = inPlug()->boundHash(path);
        return;
    }
    
    const CurvesPrimitive *animatedShafts = runTimeCast<const CurvesPrimitive>(animatedShaftsObj.get());
    if (!animatedShafts)
    {
        h = inPlug()->boundHash(path);
        return;
    }

    // Hash input bound
    h.append(inPlug()->boundHash(path));
    
    // Hash animated shafts
    h.append(animatedShaftsPlug()->objectHash(path));
    h.append(animatedShaftsPlug()->boundHash(path));
    
    // Hash binding data
    hashBindingData(barbs, h);

    // Hash parameters
    hairIdAttrNamePlug()->hash(h);
    shaftUpVectorPrimVarNamePlug()->hash(h);
    shaftPointOrientAttrNamePlug()->hash(h);
}

Imath::Box3f FeatherDeformBarbs::computeProcessedObjectBound(const ScenePath &path, const Gaffer::Context *context) const
{
    // Get objects
    ConstObjectPtr inputObject = inPlug()->object(path);
    const CurvesPrimitive *barbs = runTimeCast<const CurvesPrimitive>(inputObject.get());

    if (!barbs)
    {
        return inPlug()->bound(path);
    }

    // Get animated shafts
    ConstObjectPtr animatedShaftsObj = animatedShaftsPlug()->object(path);
    if (!animatedShaftsObj)
    {
        return inPlug()->bound(path);
    }
    
    const CurvesPrimitive *animatedShafts = runTimeCast<const CurvesPrimitive>(animatedShaftsObj.get());
    if (!animatedShafts)
    {
        return inPlug()->bound(path);
    }

    // Combine both bounds to ensure deformed barbs are contained
    Box3f combinedBound = inPlug()->bound(path);
    combinedBound.extendBy(animatedShaftsPlug()->bound(path));
    
    // Add some margin for safety (could be parameterized)
    combinedBound.min -= V3f(0.1, 0.1, 0.1);
    combinedBound.max += V3f(0.1, 0.1, 0.1);
    
    return combinedBound;
}

IECore::ConstObjectPtr FeatherDeformBarbs::computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject) const
{
    IECore::msg(IECore::Msg::Info, "FeatherDeformBarbs", "Starting computeProcessedObject");

    try
    {
        // Early out if we don't have a valid input object
        if (!inputObject)
        {
            IECore::msg(IECore::Msg::Warning, "FeatherDeformBarbs", "Null input object");
            return inputObject;
        }

        const CurvesPrimitive *barbs = runTimeCast<const CurvesPrimitive>(inputObject);
        if (!barbs)
        {
            IECore::msg(IECore::Msg::Warning, "FeatherDeformBarbs", "Input is not a CurvesPrimitive");
            return inputObject;
        }

        IECore::msg(IECore::Msg::Info, "FeatherDeformBarbs", "Input validation passed");

        // Get animated shafts
        ConstObjectPtr animatedShaftsObj = animatedShaftsPlug()->object(path);
        if (!animatedShaftsObj)
        {
            IECore::msg(IECore::Msg::Warning, "FeatherDeformBarbs", "Null animated shafts object");
            return inputObject;
        }
        
        const CurvesPrimitive *animatedShafts = runTimeCast<const CurvesPrimitive>(animatedShaftsObj.get());
        if (!animatedShafts)
        {
            IECore::msg(IECore::Msg::Warning, "FeatherDeformBarbs", "Animated shafts is not a CurvesPrimitive");
            return inputObject;
        }

        // Validate binding data
        // Check that binding data exists on barbs
        const std::string hairIdAttrName = hairIdAttrNamePlug()->getValue();
        
        auto barbHairIdIt = barbs->variables.find("shaftHairId");
        auto barbShaftPointIdIt = barbs->variables.find("shaftPointId");
        auto barbParamIt = barbs->variables.find("barbParam");
        auto restPositionIt = barbs->variables.find("restPosition");
        auto restNormalIt = barbs->variables.find("restNormal");
        auto restTangentIt = barbs->variables.find("restTangent");
        auto restBitangentIt = barbs->variables.find("restBitangent");
        
        if (barbHairIdIt == barbs->variables.end() ||
            barbShaftPointIdIt == barbs->variables.end() ||
            barbParamIt == barbs->variables.end() ||
            restPositionIt == barbs->variables.end() ||
            restNormalIt == barbs->variables.end() ||
            restTangentIt == barbs->variables.end() ||
            restBitangentIt == barbs->variables.end())
        {
            IECore::msg(IECore::Msg::Warning, "FeatherDeformBarbs", "Missing binding data on barbs");
            return inputObject;
        }
        
        // Check that hairId exists on shafts
        auto shaftHairIdIt = animatedShafts->variables.find(hairIdAttrName);
        if (shaftHairIdIt == animatedShafts->variables.end())
        {
            IECore::msg(IECore::Msg::Warning, "FeatherDeformBarbs", 
                        (boost::format("Hair ID attribute '%s' not found on shafts") % hairIdAttrName).str());
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

        IECore::msg(IECore::Msg::Info, "FeatherDeformBarbs", "Starting deformation");

        // Deform barbs
        try
        {
            deformBarbs(barbs, animatedShafts, outputBarbs.get());
        }
        catch (const std::exception &e)
        {
            IECore::msg(IECore::Msg::Error, "FeatherDeformBarbs",
                        (boost::format("Deformation failed: %s") % e.what()).str());
            return inputObject;
        }

        // Clean up binding attributes if requested
        if (cleanupBindAttributesPlug()->getValue())
        {
            const std::vector<std::string> bindingAttrs = {
                "restPosition", "restNormal", "restTangent", "restBitangent",
                "shaftHairId", "shaftPointId", "barbParam"};

            for (const std::string &attr : bindingAttrs)
            {
                auto it = outputBarbs->variables.find(attr);
                if (it != outputBarbs->variables.end())
                {
                    outputBarbs->variables.erase(it);
                }
            }
        }

        IECore::msg(IECore::Msg::Info, "FeatherDeformBarbs", "Deformation completed successfully");
        return outputBarbs;
    }
    catch (const std::exception &e)
    {
        IECore::msg(IECore::Msg::Error, "FeatherDeformBarbs",
                    (boost::format("Computation failed: %s") % e.what()).str());
        return inputObject;
    }
    catch (...)
    {
        IECore::msg(IECore::Msg::Error, "FeatherDeformBarbs", "Unknown error in computation");
        return inputObject;
    }
}

void FeatherDeformBarbs::deformBarbs(
    const IECoreScene::CurvesPrimitive *barbs,
    const IECoreScene::CurvesPrimitive *animatedShafts,
    IECoreScene::CurvesPrimitive *outputBarbs) const
{
    if (!barbs || !animatedShafts || !outputBarbs)
    {
        throw IECore::Exception("Null input to deformBarbs");
    }

    IECore::msg(IECore::Msg::Info, "FeatherDeformBarbs", "Starting deformation process");

    try
    {
        // Validate barbs data
        if (!barbs->verticesPerCurve() || barbs->verticesPerCurve()->readable().empty())
        {
            throw IECore::Exception("Invalid barbs topology");
        }

        // Log input stats
        IECore::msg(IECore::Msg::Info, "FeatherDeformBarbs",
                    (boost::format("Input: %d barbs curves, Animated shafts: %d curves") 
                     % barbs->verticesPerCurve()->readable().size()
                     % animatedShafts->verticesPerCurve()->readable().size()).str());

        IECore::msg(IECore::Msg::Info, "FeatherDeformBarbs", "Reading binding data");

        // Get attribute names
        const std::string hairIdAttrName = hairIdAttrNamePlug()->getValue();
        const std::string upVectorName = shaftUpVectorPrimVarNamePlug()->getValue();
        const std::string orientAttrName = shaftPointOrientAttrNamePlug()->getValue();

        // Read binding data
        const IntVectorData *shaftHairIds = animatedShafts->variableData<IntVectorData>(hairIdAttrName, PrimitiveVariable::Uniform);
        const IntVectorData *barbShaftHairIds = barbs->variableData<IntVectorData>("shaftHairId", PrimitiveVariable::Uniform);
        const IntVectorData *barbShaftPointIds = barbs->variableData<IntVectorData>("shaftPointId", PrimitiveVariable::Uniform);
        const FloatVectorData *barbParams = barbs->variableData<FloatVectorData>("barbParam", PrimitiveVariable::Uniform);
        
        const V3fVectorData *restPositionsData = barbs->variableData<V3fVectorData>("restPosition", PrimitiveVariable::Uniform);
        const V3fVectorData *restNormalsData = barbs->variableData<V3fVectorData>("restNormal", PrimitiveVariable::Uniform);
        const V3fVectorData *restTangentsData = barbs->variableData<V3fVectorData>("restTangent", PrimitiveVariable::Uniform);
        const V3fVectorData *restBitangentsData = barbs->variableData<V3fVectorData>("restBitangent", PrimitiveVariable::Uniform);

        if (!shaftHairIds || !barbShaftHairIds || !barbShaftPointIds || !barbParams ||
            !restPositionsData || !restNormalsData || !restTangentsData || !restBitangentsData)
        {
            IECore::msg(IECore::Msg::Error, "FeatherDeformBarbs", "Missing binding data attributes");
            throw IECore::Exception("Missing binding data");
        }

        // Get positions
        const V3fVectorData *shaftPositions = animatedShafts->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);
        const V3fVectorData *barbPositions = barbs->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);
        
        if (!shaftPositions || !barbPositions)
        {
            throw IECore::Exception("Missing position data");
        }
        
        // Get optional shaft orientation attributes
        const V3fVectorData *upVectors = nullptr;
        if (!upVectorName.empty())
        {
            upVectors = animatedShafts->variableData<V3fVectorData>(upVectorName, PrimitiveVariable::Vertex);
        }
        
        const QuatfVectorData *orientations = nullptr;
        if (!orientAttrName.empty())
        {
            orientations = animatedShafts->variableData<QuatfVectorData>(orientAttrName, PrimitiveVariable::Vertex);
        }

        // Access data
        const std::vector<int> &shaftHairIdValues = shaftHairIds->readable();
        const std::vector<int> &barbShaftHairIdValues = barbShaftHairIds->readable();
        const std::vector<int> &barbShaftPointIdValues = barbShaftPointIds->readable();
        const std::vector<float> &barbParamValues = barbParams->readable();
        
        const std::vector<V3f> &restPositions = restPositionsData->readable();
        const std::vector<V3f> &restNormals = restNormalsData->readable();
        const std::vector<V3f> &restTangents = restTangentsData->readable();
        const std::vector<V3f> &restBitangents = restBitangentsData->readable();
        
        const std::vector<V3f> &shaftPositionValues = shaftPositions->readable();
        const std::vector<V3f> &barbPositionValues = barbPositions->readable();

        // Calculate shaft curve offsets
        const std::vector<int> &shaftVertsPerCurve = animatedShafts->verticesPerCurve()->readable();
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

        // Create map from hair ID to shaft curve index for quick lookup
        std::unordered_map<int, size_t> hairIdToShaftIndex;
        for (size_t i = 0; i < shaftHairIdValues.size(); ++i)
        {
            hairIdToShaftIndex[shaftHairIdValues[i]] = i;
        }
        
        // Initialize position data for output
        V3fVectorDataPtr positionData = new V3fVectorData;
        std::vector<V3f> &positions = positionData->writable();
        positions.resize(barbs->variableSize(PrimitiveVariable::Vertex));
        
        // Copy initial positions (will be modified)
        const V3fVectorData *inputPositions = barbs->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);
        if (inputPositions)
        {
            positions = inputPositions->readable();
        }
        
        // Process barbs in parallel
        const size_t numBarbs = barbVertsPerCurve.size();
        const size_t numThreads = std::thread::hardware_concurrency();
        const size_t batchSize = calculateBatchSize(numBarbs, numThreads);
        
        tbb::atomic<size_t> degenerateFrames{0};
        tbb::atomic<size_t> processedBarbs{0};
        tbb::atomic<size_t> errorCount{0};
        
        parallel_for(blocked_range<size_t>(0, numBarbs, batchSize),
                    [&](const blocked_range<size_t> &range)
                    {
                        for (size_t i = range.begin(); i != range.end(); ++i)
                        {
                            try
                            {
                                // Get binding data for this barb
                                const int barbShaftHairId = barbShaftHairIdValues[i];
                                const int barbShaftPointId = barbShaftPointIdValues[i];
                                const float barbParam = barbParamValues[i];
                                
                                // Find matching shaft using hair ID
                                auto shaftIt = hairIdToShaftIndex.find(barbShaftHairId);
                                if (shaftIt == hairIdToShaftIndex.end())
                                {
                                    ++errorCount;
                                    continue; // No matching shaft found
                                }
                                
                                const size_t shaftIndex = shaftIt->second;
                                
                                // Validate shaft point index
                                if (barbShaftPointId < 0 || 
                                    barbShaftPointId >= shaftVertsPerCurve[shaftIndex])
                                {
                                    ++errorCount;
                                    continue;
                                }
                                
                                // Get shaft point global index
                                const size_t shaftPointGlobalIndex = shaftOffsets[shaftIndex] + barbShaftPointId;
                                
                                // Get barb vertex range
                                const size_t startIdx = barbOffsets[i];
                                const size_t endIdx = (i + 1 < barbOffsets.size()) ? barbOffsets[i + 1] : positions.size();
                                
                                // Get barb root point (first vertex)
                                const V3f &barbRootPos = positions[startIdx];
                                
                                // Get rest and deformed shaft point positions
                                const V3f &restShaftPos = restPositions[i];
                                const V3f &deformedShaftPos = shaftPositionValues[shaftPointGlobalIndex];
                                
                                // Build rest frame from binding data
                                RestFrame restFrame;
                                restFrame.position = restPositions[i];
                                restFrame.normal = restNormals[i];
                                restFrame.tangent = restTangents[i];
                                restFrame.bitangent = restBitangents[i];
                                
                                // Calculate tangent along shaft at point
                                V3f deformedTangent;
                                if (barbShaftPointId + 1 < shaftVertsPerCurve[shaftIndex])
                                {
                                    const size_t nextPointIndex = shaftOffsets[shaftIndex] + barbShaftPointId + 1;
                                    deformedTangent = shaftPositionValues[nextPointIndex] - deformedShaftPos;
                                }
                                else if (barbShaftPointId > 0)
                                {
                                    const size_t prevPointIndex = shaftOffsets[shaftIndex] + barbShaftPointId - 1;
                                    deformedTangent = deformedShaftPos - shaftPositionValues[prevPointIndex];
                                }
                                else
                                {
                                    // Single-point shaft, use up direction
                                    deformedTangent = V3f(0, 1, 0);
                                }
                                
                                // Build deformed frame with appropriate normal calculation
                                RestFrame deformedFrame;
                                deformedFrame.position = deformedShaftPos;
                                deformedFrame.tangent = safeNormalize(deformedTangent);
                                
                                // Calculate normal for deformed frame
                                V3f deformedNormal;
                                if (upVectors)
                                {
                                    // Use provided up vector
                                    const V3f &up = upVectors->readable()[shaftPointGlobalIndex];
                                    deformedNormal = safeNormalize(up - deformedFrame.tangent * up.dot(deformedFrame.tangent));
                                }
                                else if (orientations)
                                {
                                    // Use orientation quaternion
                                    const Quatf &orient = orientations->readable()[shaftPointGlobalIndex];
                                    V3f localNormal(1, 0, 0);
                                    deformedNormal = orient.rotateVector(localNormal);
                                }
                                else
                                {
                                    // Try to preserve normal direction from rest pose
                                    V3f worldUp = restFrame.normal;
                                    
                                    // If normal is too aligned with tangent, use a different direction
                                    float upDotTangent = deformedFrame.tangent.dot(worldUp);
                                    if (std::abs(upDotTangent) > 0.95f)
                                    {
                                        worldUp = V3f(1, 0, 0);
                                    }
                                    
                                    deformedNormal = worldUp - deformedFrame.tangent * worldUp.dot(deformedFrame.tangent);
                                }
                                
                                deformedFrame.normal = safeNormalize(deformedNormal);
                                deformedFrame.orthonormalize();
                                
                                // Build matrices for transformation
                                M44f restMatrix = restFrame.toMatrix();
                                M44f deformedMatrix = deformedFrame.toMatrix();
                                
                                // Calculate transform matrix
                                M44f transformMatrix = restMatrix.inverse() * deformedMatrix;
                                
                                // Check for degenerate transformation
                                bool useFallback = false;
                                const float maxAllowedScale = 10.0f;
                                
                                // Check scale factors from transformation matrix
                                V3f scaleX(transformMatrix[0][0], transformMatrix[1][0], transformMatrix[2][0]);
                                V3f scaleY(transformMatrix[0][1], transformMatrix[1][1], transformMatrix[2][1]);
                                V3f scaleZ(transformMatrix[0][2], transformMatrix[1][2], transformMatrix[2][2]);
                                
                                float maxScale = std::max({scaleX.length(), scaleY.length(), scaleZ.length()});
                                
                                if (maxScale > maxAllowedScale)
                                {
                                    useFallback = true;
                                    ++degenerateFrames;
                                }
                                
                                // Get barb root offset from rest position
                                V3f rootOffset = barbRootPos - restShaftPos;
                                
                                // Transform barb vertices
                                if (useFallback)
                                {
                                    // Use simple translation for degenerate cases
                                    V3f translation = deformedShaftPos - restShaftPos;
                                    
                                    for (size_t j = startIdx; j < endIdx; ++j)
                                    {
                                        positions[j] += translation;
                                    }
                                }
                                else
                                {
                                    // Apply full rigid transformation
                                    for (size_t j = startIdx; j < endIdx; ++j)
                                    {
                                        // Get point relative to rest shaft position
                                        V3f localP = positions[j] - restShaftPos;
                                        
                                        // Apply transformation
                                        V3f transformedP;
                                        transformMatrix.multVecMatrix(localP, transformedP);
                                        
                                        // Set transformed position
                                        positions[j] = transformedP + deformedShaftPos;
                                    }
                                }
                                
                                ++processedBarbs;
                            }
                            catch (const std::exception &e)
                            {
                                ++errorCount;
                                IECore::msg(IECore::Msg::Error, "FeatherDeformBarbs",
                                            (boost::format("Error processing barb %d: %s") % i % e.what()).str());
                            }
                        }
                    });
        
        IECore::msg(IECore::Msg::Info, "FeatherDeformBarbs",
                    (boost::format("Deformation complete - Processed: %d/%d barbs, Degenerate frames: %d, Errors: %d") 
                     % processedBarbs.load() % numBarbs % degenerateFrames.load() % errorCount.load()).str());
        
        // Update output positions
        outputBarbs->variables["P"] = PrimitiveVariable(PrimitiveVariable::Vertex, positionData);
    }
    catch (const std::exception &e)
    {
        IECore::msg(IECore::Msg::Error, "FeatherDeformBarbs",
                    (boost::format("Deformation failed: %s") % e.what()).str());
        throw;
    }
}