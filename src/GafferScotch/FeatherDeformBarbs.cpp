#include "GafferScotch/FeatherDeformBarbs.h"
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

IE_CORE_DEFINERUNTIMETYPED(FeatherDeformBarbs);

size_t FeatherDeformBarbs::g_firstPlugIndex = 0;

FeatherDeformBarbs::FeatherDeformBarbs(const std::string &name)
    : Deformer(name)
{
    storeIndexOfNextChild(g_firstPlugIndex);

    // Add animated shafts input
    addChild(new ScenePlug("animatedShafts", Plug::In));
    
    // Add rest shafts input
    addChild(new ScenePlug("restShafts", Plug::In));

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

ScenePlug *FeatherDeformBarbs::restShaftsPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex + 1);
}

const ScenePlug *FeatherDeformBarbs::restShaftsPlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex + 1);
}

StringPlug *FeatherDeformBarbs::hairIdAttrNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 2);
}

const StringPlug *FeatherDeformBarbs::hairIdAttrNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 2);
}

StringPlug *FeatherDeformBarbs::shaftUpVectorPrimVarNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 3);
}

const StringPlug *FeatherDeformBarbs::shaftUpVectorPrimVarNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 3);
}

StringPlug *FeatherDeformBarbs::shaftPointOrientAttrNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 4);
}

const StringPlug *FeatherDeformBarbs::shaftPointOrientAttrNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 4);
}

BoolPlug *FeatherDeformBarbs::cleanupBindAttributesPlug()
{
    return getChild<BoolPlug>(g_firstPlugIndex + 5);
}

const BoolPlug *FeatherDeformBarbs::cleanupBindAttributesPlug() const
{
    return getChild<BoolPlug>(g_firstPlugIndex + 5);
}

void FeatherDeformBarbs::affects(const Gaffer::Plug *input, AffectedPlugsContainer &outputs) const
{
    Deformer::affects(input, outputs);

    if (input == animatedShaftsPlug()->objectPlug() ||
        input == restShaftsPlug()->objectPlug() ||
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
           input == restShaftsPlug()->objectPlug() ||
           input == hairIdAttrNamePlug() ||
           input == shaftUpVectorPrimVarNamePlug() ||
           input == shaftPointOrientAttrNamePlug() ||
           input == cleanupBindAttributesPlug();
}

bool FeatherDeformBarbs::affectsProcessedObjectBound(const Gaffer::Plug *input) const
{
    return input == animatedShaftsPlug()->objectPlug() ||
           input == restShaftsPlug()->objectPlug() ||
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
    
    // Get rest shafts
    ConstObjectPtr restShaftsObj = restShaftsPlug()->object(path);
    if (!restShaftsObj)
    {
        h = inputObject->hash();
        return;
    }

    const CurvesPrimitive *restShafts = runTimeCast<const CurvesPrimitive>(restShaftsObj.get());
    if (!restShafts)
    {
        h = inputObject->hash();
        return;
    }

    // Hash input barbs - only positions and topology are needed
    hashPositions(barbs, h);
    hashTopology(barbs, h);

    // Hash animated shafts - only positions and topology are needed
    hashPositions(animatedShafts, h);
    hashTopology(animatedShafts, h);
    
    // Hash rest shafts - only positions and topology are needed
    hashPositions(restShafts, h);
    hashTopology(restShafts, h);

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
    
    // Get rest shafts
    ConstObjectPtr restShaftsObj = restShaftsPlug()->object(path);
    if (!restShaftsObj)
    {
        h = inPlug()->boundHash(path);
        return;
    }

    const CurvesPrimitive *restShafts = runTimeCast<const CurvesPrimitive>(restShaftsObj.get());
    if (!restShafts)
    {
        h = inPlug()->boundHash(path);
        return;
    }

    // Hash input barbs - only positions and topology are needed for bound calculation
    hashPositions(barbs, h);
    hashTopology(barbs, h);

    // Hash animated shafts - only positions and topology are needed for bound calculation
    hashPositions(animatedShafts, h);
    hashTopology(animatedShafts, h);
    
    // Hash rest shafts - only positions and topology are needed for bound calculation
    hashPositions(restShafts, h);
    hashTopology(restShafts, h);

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
    
    // Get rest shafts
    ConstObjectPtr restShaftsObj = restShaftsPlug()->object(path);
    if (!restShaftsObj)
    {
        return inPlug()->bound(path);
    }

    const CurvesPrimitive *restShafts = runTimeCast<const CurvesPrimitive>(restShaftsObj.get());
    if (!restShafts)
    {
        return inPlug()->bound(path);
    }

    // Combine bounds to ensure deformed barbs are contained
    Box3f combinedBound = inPlug()->bound(path);
    combinedBound.extendBy(animatedShaftsPlug()->bound(path));
    combinedBound.extendBy(restShaftsPlug()->bound(path));

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
        
        // Get rest shafts
        ConstObjectPtr restShaftsObj = restShaftsPlug()->object(path);
        if (!restShaftsObj)
        {
            IECore::msg(IECore::Msg::Warning, "FeatherDeformBarbs", "Null rest shafts object");
            return inputObject;
        }

        const CurvesPrimitive *restShafts = runTimeCast<const CurvesPrimitive>(restShaftsObj.get());
        if (!restShafts)
        {
            IECore::msg(IECore::Msg::Warning, "FeatherDeformBarbs", "Rest shafts is not a CurvesPrimitive");
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

        // Check that hairId exists on both animated and rest shafts
        auto animatedShaftHairIdIt = animatedShafts->variables.find(hairIdAttrName);
        auto restShaftHairIdIt = restShafts->variables.find(hairIdAttrName);
        
        if (animatedShaftHairIdIt == animatedShafts->variables.end())
        {
            IECore::msg(IECore::Msg::Warning, "FeatherDeformBarbs",
                        (boost::format("Hair ID attribute '%s' not found on animated shafts") % hairIdAttrName).str());
            return inputObject;
        }
        
        if (restShaftHairIdIt == restShafts->variables.end())
        {
            IECore::msg(IECore::Msg::Warning, "FeatherDeformBarbs",
                        (boost::format("Hair ID attribute '%s' not found on rest shafts") % hairIdAttrName).str());
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
            deformBarbs(barbs, animatedShafts, restShafts, outputBarbs.get());
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
    const IECoreScene::CurvesPrimitive *restShafts,
    IECoreScene::CurvesPrimitive *outputBarbs) const
{
    if (!barbs || !animatedShafts || !restShafts || !outputBarbs)
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
                    (boost::format("Input: %d barbs curves, Animated shafts: %d curves, Rest shafts: %d curves") 
                    % barbs->verticesPerCurve()->readable().size() 
                    % animatedShafts->verticesPerCurve()->readable().size()
                    % restShafts->verticesPerCurve()->readable().size()).str());

        IECore::msg(IECore::Msg::Info, "FeatherDeformBarbs", "Reading binding data");

        // Get attribute names
        const std::string hairIdAttrName = hairIdAttrNamePlug()->getValue();
        const std::string upVectorName = shaftUpVectorPrimVarNamePlug()->getValue();
        const std::string orientAttrName = shaftPointOrientAttrNamePlug()->getValue();

        // Read binding data
        const IntVectorData *restShaftHairIds = restShafts->variableData<IntVectorData>(hairIdAttrName, PrimitiveVariable::Uniform);
        const IntVectorData *animatedShaftHairIds = animatedShafts->variableData<IntVectorData>(hairIdAttrName, PrimitiveVariable::Uniform);
        const IntVectorData *barbShaftHairIds = barbs->variableData<IntVectorData>("shaftHairId", PrimitiveVariable::Uniform);
        const IntVectorData *barbShaftPointIds = barbs->variableData<IntVectorData>("shaftPointId", PrimitiveVariable::Uniform);
        const FloatVectorData *barbParams = barbs->variableData<FloatVectorData>("barbParam", PrimitiveVariable::Uniform);

        if (!restShaftHairIds || !animatedShaftHairIds || !barbShaftHairIds || !barbShaftPointIds || !barbParams)
        {
            IECore::msg(IECore::Msg::Error, "FeatherDeformBarbs", "Missing binding data attributes");
            throw IECore::Exception("Missing binding data");
        }

        // Get positions
        const V3fVectorData *animatedShaftPositions = animatedShafts->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);
        const V3fVectorData *restShaftPositions = restShafts->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);
        const V3fVectorData *barbPositions = barbs->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);

        if (!animatedShaftPositions || !restShaftPositions || !barbPositions)
        {
            throw IECore::Exception("Missing position data");
        }

        // Get optional orientation attributes
        const V3fVectorData *restUpVectors = nullptr;
        if (!upVectorName.empty())
        {
            restUpVectors = restShafts->variableData<V3fVectorData>(upVectorName, PrimitiveVariable::Vertex);
        }

        const QuatfVectorData *restOrientations = nullptr;
        if (!orientAttrName.empty())
        {
            restOrientations = restShafts->variableData<QuatfVectorData>(orientAttrName, PrimitiveVariable::Vertex);
        }
        
        const V3fVectorData *animatedUpVectors = nullptr;
        if (!upVectorName.empty())
        {
            animatedUpVectors = animatedShafts->variableData<V3fVectorData>(upVectorName, PrimitiveVariable::Vertex);
        }

        const QuatfVectorData *animatedOrientations = nullptr;
        if (!orientAttrName.empty())
        {
            animatedOrientations = animatedShafts->variableData<QuatfVectorData>(orientAttrName, PrimitiveVariable::Vertex);
        }

        // Access data
        const std::vector<int> &restShaftHairIdValues = restShaftHairIds->readable();
        const std::vector<int> &animatedShaftHairIdValues = animatedShaftHairIds->readable();
        const std::vector<int> &barbShaftHairIdValues = barbShaftHairIds->readable();
        const std::vector<int> &barbShaftPointIdValues = barbShaftPointIds->readable();
        const std::vector<float> &barbParamValues = barbParams->readable();

        const std::vector<V3f> &animatedShaftPositionValues = animatedShaftPositions->readable();
        const std::vector<V3f> &restShaftPositionValues = restShaftPositions->readable();
        const std::vector<V3f> &barbPositionValues = barbPositions->readable();

        // Calculate shaft curve offsets for rest shafts
        const std::vector<int> &restShaftVertsPerCurve = restShafts->verticesPerCurve()->readable();
        std::vector<size_t> restShaftOffsets;
        restShaftOffsets.reserve(restShaftVertsPerCurve.size());

        size_t offset = 0;
        for (int count : restShaftVertsPerCurve)
        {
            restShaftOffsets.push_back(offset);
            offset += count;
        }
        
        // Calculate shaft curve offsets for animated shafts
        const std::vector<int> &animatedShaftVertsPerCurve = animatedShafts->verticesPerCurve()->readable();
        std::vector<size_t> animatedShaftOffsets;
        animatedShaftOffsets.reserve(animatedShaftVertsPerCurve.size());

        offset = 0;
        for (int count : animatedShaftVertsPerCurve)
        {
            animatedShaftOffsets.push_back(offset);
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

        // Create maps from hair ID to shaft curve index for quick lookup
        std::unordered_map<int, size_t> restHairIdToShaftIndex;
        for (size_t i = 0; i < restShaftHairIdValues.size(); ++i)
        {
            restHairIdToShaftIndex[restShaftHairIdValues[i]] = i;
        }
        
        std::unordered_map<int, size_t> animatedHairIdToShaftIndex;
        for (size_t i = 0; i < animatedShaftHairIdValues.size(); ++i)
        {
            animatedHairIdToShaftIndex[animatedShaftHairIdValues[i]] = i;
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

                                 // Find matching shaft in rest shafts using hair ID
                                 auto restShaftIt = restHairIdToShaftIndex.find(barbShaftHairId);
                                 if (restShaftIt == restHairIdToShaftIndex.end())
                                 {
                                     ++errorCount;
                                     continue; // No matching rest shaft found
                                 }
                                 
                                 // Find matching shaft in animated shafts using hair ID
                                 auto animatedShaftIt = animatedHairIdToShaftIndex.find(barbShaftHairId);
                                 if (animatedShaftIt == animatedHairIdToShaftIndex.end())
                                 {
                                     ++errorCount;
                                     continue; // No matching animated shaft found
                                 }

                                 const size_t restShaftIndex = restShaftIt->second;
                                 const size_t animatedShaftIndex = animatedShaftIt->second;

                                 // Validate shaft point index against rest shaft
                                 if (barbShaftPointId < 0 ||
                                     barbShaftPointId >= restShaftVertsPerCurve[restShaftIndex])
                                 {
                                     ++errorCount;
                                     continue;
                                 }
                                 
                                 // Validate shaft point index against animated shaft
                                 if (barbShaftPointId < 0 ||
                                     barbShaftPointId >= animatedShaftVertsPerCurve[animatedShaftIndex])
                                 {
                                     ++errorCount;
                                     continue;
                                 }

                                 // Get shaft point global indices
                                 const size_t restShaftPointGlobalIndex = restShaftOffsets[restShaftIndex] + barbShaftPointId;
                                 const size_t animatedShaftPointGlobalIndex = animatedShaftOffsets[animatedShaftIndex] + barbShaftPointId;

                                 // Get barb vertex range
                                 const size_t startIdx = barbOffsets[i];
                                 const size_t endIdx = (i + 1 < barbOffsets.size()) ? barbOffsets[i + 1] : positions.size();

                                 // Get barb root point (first vertex)
                                 const V3f &barbRootPos = positions[startIdx];

                                 // Get rest and deformed shaft point positions
                                 const V3f &restShaftPos = restShaftPositionValues[restShaftPointGlobalIndex];
                                 const V3f &deformedShaftPos = animatedShaftPositionValues[animatedShaftPointGlobalIndex];

                                 // Build rest frame
                                 RestFrame restFrame;
                                 restFrame.position = restShaftPos;
                                 
                                 // Calculate tangent along rest shaft at point
                                 V3f restTangent;
                                 if (barbShaftPointId + 1 < restShaftVertsPerCurve[restShaftIndex])
                                 {
                                     const size_t nextPointIndex = restShaftOffsets[restShaftIndex] + barbShaftPointId + 1;
                                     restTangent = restShaftPositionValues[nextPointIndex] - restShaftPos;
                                 }
                                 else if (barbShaftPointId > 0)
                                 {
                                     const size_t prevPointIndex = restShaftOffsets[restShaftIndex] + barbShaftPointId - 1;
                                     restTangent = restShaftPos - restShaftPositionValues[prevPointIndex];
                                 }
                                 else
                                 {
                                     // Single-point shaft, use up direction
                                     restTangent = V3f(0, 1, 0);
                                 }
                                 
                                 restFrame.tangent = safeNormalize(restTangent);
                                 
                                 // Calculate normal for rest frame
                                 V3f restNormal;
                                 if (restOrientations)
                                 {
                                     // Use orientation quaternion (preferred method)
                                     const Quatf &orient = restOrientations->readable()[restShaftPointGlobalIndex];
                                     V3f localNormal(1, 0, 0);
                                     restNormal = orient.rotateVector(localNormal);
                                 }
                                 else
                                 {
                                     // Orientation is required
                                     IECore::msg(IECore::Msg::Warning, "FeatherDeformBarbs",
                                                 "Orientation attribute not found on rest shafts. This will result in invalid frames.");

                                     // Use a default normal as fallback, but this won't be correct
                                     restNormal = V3f(1, 0, 0);
                                 }
                                 
                                 restFrame.normal = safeNormalize(restNormal);
                                 restFrame.orthonormalize();

                                 // Calculate tangent along animated shaft at point
                                 V3f deformedTangent;
                                 if (barbShaftPointId + 1 < animatedShaftVertsPerCurve[animatedShaftIndex])
                                 {
                                     const size_t nextPointIndex = animatedShaftOffsets[animatedShaftIndex] + barbShaftPointId + 1;
                                     deformedTangent = animatedShaftPositionValues[nextPointIndex] - deformedShaftPos;
                                 }
                                 else if (barbShaftPointId > 0)
                                 {
                                     const size_t prevPointIndex = animatedShaftOffsets[animatedShaftIndex] + barbShaftPointId - 1;
                                     deformedTangent = deformedShaftPos - animatedShaftPositionValues[prevPointIndex];
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
                                 if (animatedOrientations)
                                 {
                                     // Use orientation quaternion (preferred method)
                                     const Quatf &orient = animatedOrientations->readable()[animatedShaftPointGlobalIndex];
                                     V3f localNormal(1, 0, 0);
                                     deformedNormal = orient.rotateVector(localNormal);
                                 }
                                 else
                                 {
                                     // Orientation is required
                                     IECore::msg(IECore::Msg::Warning, "FeatherDeformBarbs",
                                                 "Orientation attribute not found on animated shafts. This will result in invalid frames.");

                                     // Use a default normal as fallback, but this won't be correct
                                     deformedNormal = V3f(1, 0, 0);
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
                    (boost::format("Deformation complete - Processed: %d/%d barbs, Degenerate frames: %d, Errors: %d") % processedBarbs.load() % numBarbs % degenerateFrames.load() % errorCount.load()).str());

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