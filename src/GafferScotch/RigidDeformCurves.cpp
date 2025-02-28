#include "GafferScotch/RigidDeformCurves.h"
#include "GafferScotch/ScenePathUtil.h"

#include "IECore/NullObject.h"
#include "IECoreScene/CurvesPrimitive.h"
#include "IECoreScene/MeshPrimitive.h"
#include "IECoreScene/MeshPrimitiveEvaluator.h"
#include "IECoreScene/PrimitiveVariable.h"
#include "IECore/VectorTypedData.h"
#include "IECore/StringAlgo.h"
#include "Imath/ImathVec.h"
#include "IECoreScene/MeshAlgo.h"

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/cache_aligned_allocator.h>

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
    inline size_t calculateBatchSize(size_t numCurves, size_t numThreads)
    {
        // Aim for ~4 batches per thread for good load balancing
        const size_t targetBatches = numThreads * 4;
        return std::max(size_t(1), numCurves / targetBatches);
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
            "triangleIndex", "barycentricCoords", "uvCoords", "rootPoint"};

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

IE_CORE_DEFINERUNTIMETYPED(RigidDeformCurves);

size_t RigidDeformCurves::g_firstPlugIndex = 0;

RigidDeformCurves::RigidDeformCurves(const std::string &name)
    : Deformer(name)
{
    storeIndexOfNextChild(g_firstPlugIndex);

    // Add mesh inputs
    addChild(new ScenePlug("staticDeformer", Plug::In));
    addChild(new ScenePlug("animatedDeformer", Plug::In));

    // Binding mode
    addChild(new BoolPlug("useBindAttr", Plug::In, false));
    addChild(new StringPlug("deformerPath", Plug::In, ""));           // Used when useBindAttr=false
    addChild(new StringPlug("bindAttr", Plug::In, ""));               // Used when useBindAttr=true
    addChild(new BoolPlug("cleanupBindAttributes", Plug::In, false)); // Whether to remove binding attributes from output

    // Fast pass-throughs for things we don't modify
    outPlug()->attributesPlug()->setInput(inPlug()->attributesPlug());
    outPlug()->transformPlug()->setInput(inPlug()->transformPlug());
    outPlug()->boundPlug()->setInput(inPlug()->boundPlug());
}

RigidDeformCurves::~RigidDeformCurves()
{
}

ScenePlug *RigidDeformCurves::staticDeformerPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

const ScenePlug *RigidDeformCurves::staticDeformerPlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

ScenePlug *RigidDeformCurves::animatedDeformerPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex + 1);
}

const ScenePlug *RigidDeformCurves::animatedDeformerPlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex + 1);
}

BoolPlug *RigidDeformCurves::useBindAttrPlug()
{
    return getChild<BoolPlug>(g_firstPlugIndex + 2);
}

const BoolPlug *RigidDeformCurves::useBindAttrPlug() const
{
    return getChild<BoolPlug>(g_firstPlugIndex + 2);
}

StringPlug *RigidDeformCurves::deformerPathPlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 3);
}

const StringPlug *RigidDeformCurves::deformerPathPlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 3);
}

StringPlug *RigidDeformCurves::bindAttrPlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 4);
}

const StringPlug *RigidDeformCurves::bindAttrPlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 4);
}

BoolPlug *RigidDeformCurves::cleanupBindAttributesPlug()
{
    return getChild<BoolPlug>(g_firstPlugIndex + 5);
}

const BoolPlug *RigidDeformCurves::cleanupBindAttributesPlug() const
{
    return getChild<BoolPlug>(g_firstPlugIndex + 5);
}

void RigidDeformCurves::affects(const Gaffer::Plug *input, AffectedPlugsContainer &outputs) const
{
    Deformer::affects(input, outputs);

    if (input == staticDeformerPlug()->objectPlug() ||
        input == animatedDeformerPlug()->objectPlug() ||
        input == useBindAttrPlug() ||
        input == deformerPathPlug() ||
        input == bindAttrPlug() ||
        input == cleanupBindAttributesPlug())
    {
        outputs.push_back(outPlug()->objectPlug());
    }
}

bool RigidDeformCurves::acceptsInput(const Gaffer::Plug *plug, const Gaffer::Plug *inputPlug) const
{
    if (!Deformer::acceptsInput(plug, inputPlug))
    {
        return false;
    }

    return true;
}

bool RigidDeformCurves::affectsProcessedObject(const Gaffer::Plug *input) const
{
    return input == staticDeformerPlug()->objectPlug() ||
           input == animatedDeformerPlug()->objectPlug() ||
           input == useBindAttrPlug() ||
           input == deformerPathPlug() ||
           input == bindAttrPlug() ||
           input == cleanupBindAttributesPlug();
}

void RigidDeformCurves::hashProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    // Get objects
    ConstObjectPtr inputObject = inPlug()->object(path);
    const CurvesPrimitive *curves = runTimeCast<const CurvesPrimitive>(inputObject.get());

    if (!curves)
    {
        h = inputObject->hash();
        return;
    }

    // Get the target path based on mode
    ScenePath meshPath;
    const bool useBindAttr = useBindAttrPlug()->getValue();
    if (useBindAttr)
    {
        // Try to get path from attribute
        const std::string bindAttrName = bindAttrPlug()->getValue();
        if (!bindAttrName.empty())
        {
            auto it = curves->variables.find(bindAttrName);
            if (it != curves->variables.end())
            {
                if (const StringData *pathData = runTimeCast<const StringData>(it->second.data.get()))
                {
                    meshPath = GafferScotch::makeScenePath(pathData->readable());
                }
                else if (const StringVectorData *pathVectorData = runTimeCast<const StringVectorData>(it->second.data.get()))
                {
                    const std::vector<std::string> &paths = pathVectorData->readable();
                    if (!paths.empty())
                    {
                        meshPath = GafferScotch::makeScenePath(paths[0]); // Use first path for now
                    }
                }
            }
        }
    }
    else
    {
        // Use path from plug
        meshPath = GafferScotch::makeScenePath(deformerPathPlug()->getValue());
    }

    // Hash meshes
    h.append(staticDeformerPlug()->objectHash(meshPath));
    h.append(animatedDeformerPlug()->objectHash(meshPath));

    // Hash input curves and binding data
    h.append(inPlug()->objectHash(path));
    hashBindingData(curves, h);

    // Hash parameters
    useBindAttrPlug()->hash(h);
    deformerPathPlug()->hash(h);
    bindAttrPlug()->hash(h);
}

IECore::ConstObjectPtr RigidDeformCurves::computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject) const
{
    IECore::msg(IECore::Msg::Info, "RigidDeformCurves", "Starting computeProcessedObject");

    try
    {
        // Early out if we don't have a valid input object
        if (!inputObject)
        {
            IECore::msg(IECore::Msg::Warning, "RigidDeformCurves", "Null input object");
            return inputObject;
        }

        const CurvesPrimitive *curves = runTimeCast<const CurvesPrimitive>(inputObject);
        if (!curves)
        {
            IECore::msg(IECore::Msg::Warning, "RigidDeformCurves", "Input is not a CurvesPrimitive");
            return inputObject;
        }

        IECore::msg(IECore::Msg::Info, "RigidDeformCurves", "Input validation passed");

        // Get the target path based on mode
        ScenePath meshPath;
        const bool useBindAttr = useBindAttrPlug()->getValue();

        if (useBindAttr)
        {
            IECore::msg(IECore::Msg::Info, "RigidDeformCurves", "Using bind attribute mode");
            // Try to get path from attribute
            const std::string bindAttrName = bindAttrPlug()->getValue();
            if (bindAttrName.empty())
            {
                IECore::msg(IECore::Msg::Warning, "RigidDeformCurves", "Empty bind attribute name");
                return inputObject;
            }

            auto it = curves->variables.find(bindAttrName);
            if (it == curves->variables.end())
            {
                IECore::msg(IECore::Msg::Warning, "RigidDeformCurves",
                            (boost::format("Bind attribute '%s' not found") % bindAttrName).str());
                return inputObject;
            }

            if (!it->second.data)
            {
                IECore::msg(IECore::Msg::Warning, "RigidDeformCurves", "Null bind attribute data");
                return inputObject;
            }

            if (const StringData *pathData = runTimeCast<const StringData>(it->second.data.get()))
            {
                meshPath = GafferScotch::makeScenePath(pathData->readable());
            }
            else if (const StringVectorData *pathVectorData = runTimeCast<const StringVectorData>(it->second.data.get()))
            {
                const std::vector<std::string> &paths = pathVectorData->readable();
                if (!paths.empty())
                {
                    meshPath = GafferScotch::makeScenePath(paths[0]); // Use first path for now
                }
            }
        }
        else
        {
            IECore::msg(IECore::Msg::Info, "RigidDeformCurves", "Using direct bind path");
            meshPath = GafferScotch::makeScenePath(deformerPathPlug()->getValue());
        }

        if (meshPath.empty())
        {
            // Convert path to string for logging
            std::string pathStr;
            for (const auto &element : meshPath)
            {
                if (!pathStr.empty())
                    pathStr += "/";
                pathStr += element;
            }

            IECore::msg(IECore::Msg::Warning, "RigidDeformCurves",
                        (boost::format("Resolved mesh path: %s") % pathStr).str());

            return inputObject;
        }

        // Get meshes using resolved path
        ConstObjectPtr staticDeformerObj = staticDeformerPlug()->object(meshPath);
        if (!staticDeformerObj)
        {
            IECore::msg(IECore::Msg::Warning, "RigidDeformCurves", "Null static deformer object");
            return inputObject;
        }

        const MeshPrimitive *staticDeformer = runTimeCast<const MeshPrimitive>(staticDeformerObj.get());
        if (!staticDeformer)
        {
            IECore::msg(IECore::Msg::Warning, "RigidDeformCurves", "Static deformer is not a MeshPrimitive");
            return inputObject;
        }

        ConstObjectPtr animatedDeformerObj = animatedDeformerPlug()->object(meshPath);
        if (!animatedDeformerObj)
        {
            IECore::msg(IECore::Msg::Warning, "RigidDeformCurves", "Null animated deformer object");
            return inputObject;
        }

        const MeshPrimitive *animatedDeformer = runTimeCast<const MeshPrimitive>(animatedDeformerObj.get());
        if (!animatedDeformer)
        {
            IECore::msg(IECore::Msg::Warning, "RigidDeformCurves", "Animated deformer is not a MeshPrimitive");
            return inputObject;
        }

        IECore::msg(IECore::Msg::Info, "RigidDeformCurves", "Mesh objects validated");

        // Triangulate meshes to match RigidAttachCurves behavior
        MeshPrimitivePtr triangulatedStaticDeformer = MeshAlgo::triangulate(staticDeformer);
        MeshPrimitivePtr triangulatedAnimatedDeformer = MeshAlgo::triangulate(animatedDeformer);

        // Validate mesh topology
        if (!triangulatedStaticDeformer->vertexIds() || !triangulatedAnimatedDeformer->vertexIds())
        {
            IECore::msg(IECore::Msg::Warning, "RigidDeformCurves", "Missing vertex IDs");
            return inputObject;
        }

        if (!triangulatedStaticDeformer->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex) ||
            !triangulatedAnimatedDeformer->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex))
        {
            IECore::msg(IECore::Msg::Warning, "RigidDeformCurves", "Missing position data");
            return inputObject;
        }

        // Create output curves with same topology
        CurvesPrimitivePtr outputCurves = new CurvesPrimitive(
            curves->verticesPerCurve(),
            curves->basis(),
            curves->periodic());

        // Copy primitive variables
        for (const auto &primVar : curves->variables)
        {
            outputCurves->variables[primVar.first] = primVar.second;
        }

        IECore::msg(IECore::Msg::Info, "RigidDeformCurves", "Starting deformation");

        // Deform curves
        try
        {
            deformCurves(curves, triangulatedStaticDeformer.get(), triangulatedAnimatedDeformer.get(), outputCurves.get());
        }
        catch (const std::exception &e)
        {
            IECore::msg(IECore::Msg::Error, "RigidDeformCurves",
                        (boost::format("Deformation failed: %s") % e.what()).str());
            return inputObject;
        }

        // Clean up binding attributes if requested
        if (cleanupBindAttributesPlug()->getValue())
        {
            const std::vector<std::string> bindingAttrs = {
                "restPosition", "restNormal", "restTangent", "restBitangent",
                "triangleIndex", "barycentricCoords", "uvCoords", "rootPoint"};

            for (const std::string &attr : bindingAttrs)
            {
                auto it = outputCurves->variables.find(attr);
                if (it != outputCurves->variables.end())
                {
                    outputCurves->variables.erase(it);
                }
            }

            // Also remove the bind path attribute if it exists
            const std::string bindAttrName = bindAttrPlug()->getValue();
            if (!bindAttrName.empty())
            {
                auto it = outputCurves->variables.find(bindAttrName);
                if (it != outputCurves->variables.end())
                {
                    outputCurves->variables.erase(it);
                }
            }
        }

        IECore::msg(IECore::Msg::Info, "RigidDeformCurves", "Deformation completed successfully");
        return outputCurves;
    }
    catch (const std::exception &e)
    {
        IECore::msg(IECore::Msg::Error, "RigidDeformCurves",
                    (boost::format("Computation failed: %s") % e.what()).str());
        return inputObject;
    }
    catch (...)
    {
        IECore::msg(IECore::Msg::Error, "RigidDeformCurves", "Unknown error in computation");
        return inputObject;
    }
}

bool RigidDeformCurves::affectsProcessedObjectBound(const Gaffer::Plug *input) const
{
    return input == staticDeformerPlug()->objectPlug() ||
           input == animatedDeformerPlug()->objectPlug() ||
           input == useBindAttrPlug() ||
           input == deformerPathPlug() ||
           input == bindAttrPlug();
}

void RigidDeformCurves::hashProcessedObjectBound(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    // Get objects
    ConstObjectPtr inputObject = inPlug()->object(path);
    const CurvesPrimitive *curves = runTimeCast<const CurvesPrimitive>(inputObject.get());

    if (!curves)
    {
        h = inPlug()->boundHash(path);
        return;
    }

    // Get the target path based on mode
    ScenePath meshPath;
    const bool useBindAttr = useBindAttrPlug()->getValue();
    if (useBindAttr)
    {
        // Try to get path from attribute
        const std::string bindAttrName = bindAttrPlug()->getValue();
        if (!bindAttrName.empty())
        {
            auto it = curves->variables.find(bindAttrName);
            if (it != curves->variables.end())
            {
                if (const StringData *pathData = runTimeCast<const StringData>(it->second.data.get()))
                {
                    meshPath = GafferScotch::makeScenePath(pathData->readable());
                }
                else if (const StringVectorData *pathVectorData = runTimeCast<const StringVectorData>(it->second.data.get()))
                {
                    const std::vector<std::string> &paths = pathVectorData->readable();
                    if (!paths.empty())
                    {
                        meshPath = GafferScotch::makeScenePath(paths[0]); // Use first path for now
                    }
                }
            }
        }
    }
    else
    {
        // Use path from plug
        meshPath = GafferScotch::makeScenePath(deformerPathPlug()->getValue());
    }

    // Hash meshes
    h.append(staticDeformerPlug()->objectHash(meshPath));
    h.append(animatedDeformerPlug()->objectHash(meshPath));

    // Hash input curves and binding data
    h.append(inPlug()->boundHash(path));
    hashBindingData(curves, h);

    // Hash parameters
    useBindAttrPlug()->hash(h);
    deformerPathPlug()->hash(h);
    bindAttrPlug()->hash(h);
}

Imath::Box3f RigidDeformCurves::computeProcessedObjectBound(const ScenePath &path, const Gaffer::Context *context) const
{
    // Get objects
    ConstObjectPtr inputObject = inPlug()->object(path);
    const CurvesPrimitive *curves = runTimeCast<const CurvesPrimitive>(inputObject.get());

    if (!curves)
    {
        return inPlug()->bound(path);
    }

    // Get the target path based on mode
    ScenePath meshPath;
    const bool useBindAttr = useBindAttrPlug()->getValue();
    if (useBindAttr)
    {
        // Try to get path from attribute
        const std::string bindAttrName = bindAttrPlug()->getValue();
        if (!bindAttrName.empty())
        {
            auto it = curves->variables.find(bindAttrName);
            if (it != curves->variables.end())
            {
                if (const StringData *pathData = runTimeCast<const StringData>(it->second.data.get()))
                {
                    meshPath = GafferScotch::makeScenePath(pathData->readable());
                }
                else if (const StringVectorData *pathVectorData = runTimeCast<const StringVectorData>(it->second.data.get()))
                {
                    const std::vector<std::string> &paths = pathVectorData->readable();
                    if (!paths.empty())
                    {
                        meshPath = GafferScotch::makeScenePath(paths[0]); // Use first path for now
                    }
                }
            }
        }
    }
    else
    {
        // Use path from plug
        meshPath = GafferScotch::makeScenePath(deformerPathPlug()->getValue());
    }

    // Get animated mesh using resolved path
    ConstObjectPtr animatedDeformerObj = animatedDeformerPlug()->object(meshPath);
    if (!animatedDeformerObj)
    {
        return inPlug()->bound(path);
    }

    const MeshPrimitive *animatedDeformer = runTimeCast<const MeshPrimitive>(animatedDeformerObj.get());
    if (!animatedDeformer)
    {
        return inPlug()->bound(path);
    }

    // Use the animated mesh's bounds directly since curves are bound to its surface
    return animatedDeformer->bound();
}

void RigidDeformCurves::deformCurves(
    const IECoreScene::CurvesPrimitive *curves,
    const IECoreScene::MeshPrimitive *staticDeformer,
    const IECoreScene::MeshPrimitive *animatedDeformer,
    IECoreScene::CurvesPrimitive *outputCurves) const
{
    if (!curves || !staticDeformer || !animatedDeformer || !outputCurves)
    {
        throw IECore::Exception("Null input to deformCurves");
    }

    IECore::msg(IECore::Msg::Info, "RigidDeformCurves", "Starting deformation process");

    try
    {
        // Validate curves data
        if (!curves->verticesPerCurve() || curves->verticesPerCurve()->readable().empty())
        {
            throw IECore::Exception("Invalid curves topology");
        }

        // Log input stats
        IECore::msg(IECore::Msg::Info, "RigidDeformCurves",
                    (boost::format("Input: %d curves, Static deformer: %d verts, %d faces, Animated deformer: %d verts, %d faces") % curves->verticesPerCurve()->readable().size() % staticDeformer->variableSize(PrimitiveVariable::Vertex) % (staticDeformer->vertexIds()->readable().size() / 3) % animatedDeformer->variableSize(PrimitiveVariable::Vertex) % (animatedDeformer->vertexIds()->readable().size() / 3)).str());

        IECore::msg(IECore::Msg::Info, "RigidDeformCurves", "Reading binding data");

        // Read binding data
        const V3fVectorData *restPositionsData = runTimeCast<const V3fVectorData>(curves->variables.at("restPosition").data.get());
        const V3fVectorData *restNormalsData = runTimeCast<const V3fVectorData>(curves->variables.at("restNormal").data.get());
        const V3fVectorData *restTangentsData = runTimeCast<const V3fVectorData>(curves->variables.at("restTangent").data.get());
        const V3fVectorData *restBitangentsData = runTimeCast<const V3fVectorData>(curves->variables.at("restBitangent").data.get());
        const IntVectorData *triangleIndicesData = runTimeCast<const IntVectorData>(curves->variables.at("triangleIndex").data.get());
        const V3fVectorData *barycentricCoordsData = runTimeCast<const V3fVectorData>(curves->variables.at("barycentricCoords").data.get());

        if (!restPositionsData || !restNormalsData || !restTangentsData || !restBitangentsData || !triangleIndicesData || !barycentricCoordsData)
        {
            IECore::msg(IECore::Msg::Error, "RigidDeformCurves", "Missing binding data attributes");
            throw IECore::Exception("Missing binding data");
        }

        const std::vector<V3f> &restPositions = restPositionsData->readable();
        const std::vector<V3f> &restNormals = restNormalsData->readable();
        const std::vector<V3f> &restTangents = restTangentsData->readable();
        const std::vector<V3f> &restBitangents = restBitangentsData->readable();
        const std::vector<int> &triangleIndices = triangleIndicesData->readable();
        const std::vector<V3f> &barycentricCoords = barycentricCoordsData->readable();

        // Calculate vertex offsets
        std::vector<size_t> vertexOffsets;
        vertexOffsets.reserve(curves->verticesPerCurve()->readable().size());
        size_t offset = 0;
        for (int count : curves->verticesPerCurve()->readable())
        {
            vertexOffsets.push_back(offset);
            offset += count;
        }

        // Initialize position data
        V3fVectorDataPtr positionData = new V3fVectorData;
        std::vector<V3f> &positions = positionData->writable();
        positions.resize(curves->variableSize(PrimitiveVariable::Vertex));

        // Copy initial positions
        const V3fVectorData *inputPositions = curves->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);
        if (inputPositions)
        {
            positions = inputPositions->readable();
        }

        IECore::msg(IECore::Msg::Info, "RigidDeformCurves", "Setting up mesh data access");

        const std::vector<int> &vertexIds = animatedDeformer->vertexIds()->readable();
        const size_t numTriangles = vertexIds.size() / 3;
        const std::vector<V3f> &meshPoints = animatedDeformer->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex)->readable();

        // Calculate tangents for animated mesh
        std::pair<PrimitiveVariable, PrimitiveVariable> animatedTangents;

        // Try UV-based tangents first
        auto uvIt = animatedDeformer->variables.find("uv");
        if (uvIt == animatedDeformer->variables.end())
            uvIt = animatedDeformer->variables.find("st");
        if (uvIt == animatedDeformer->variables.end())
            uvIt = animatedDeformer->variables.find("UV");

        if (uvIt != animatedDeformer->variables.end())
        {
            animatedTangents = MeshAlgo::calculateTangents(animatedDeformer, uvIt->first, true, "P");
        }
        else
        {
            // Try edge-based tangents
            auto normalIt = animatedDeformer->variables.find("N");
            if (normalIt == animatedDeformer->variables.end())
            {
                PrimitiveVariable normals = MeshAlgo::calculateNormals(animatedDeformer);
                const_cast<MeshPrimitive *>(animatedDeformer)->variables["N"] = normals;
                normalIt = animatedDeformer->variables.find("N");
            }

            if (normalIt != animatedDeformer->variables.end())
            {
                animatedTangents = MeshAlgo::calculateTangentsFromTwoEdges(animatedDeformer, "P", normalIt->first, true, false);
            }
            else
            {
                throw IECore::Exception("Failed to calculate tangents - no UVs or normals available");
            }
        }

        IECore::msg(IECore::Msg::Info, "RigidDeformCurves", "Starting parallel curve processing");

        const size_t numCurves = curves->verticesPerCurve()->readable().size();
        const size_t numThreads = std::thread::hardware_concurrency();
        const size_t batchSize = calculateBatchSize(numCurves, numThreads);

        tbb::atomic<size_t> degenerateFrames{0};
        tbb::atomic<size_t> processedCurves{0};
        tbb::atomic<size_t> errorCount{0};

        parallel_for(blocked_range<size_t>(0, numCurves, batchSize),
                     [&](const blocked_range<size_t> &range)
                     {
                         for (size_t i = range.begin(); i != range.end(); ++i)
                         {
                             try
                             {
                                 // Get binding data
                                 const int triangleIndex = triangleIndices[i];
                                 const V3f &baryCoord = barycentricCoords[i];

                                 // Get vertex indices for the triangle
                                 const std::vector<int> &triangleVertices = {
                                     vertexIds[triangleIndex * 3],
                                     vertexIds[triangleIndex * 3 + 1],
                                     vertexIds[triangleIndex * 3 + 2]};

                                 // Get positions for the triangle vertices
                                 const std::vector<V3f> &trianglePoints = {
                                     meshPoints[triangleVertices[0]],
                                     meshPoints[triangleVertices[1]],
                                     meshPoints[triangleVertices[2]]};

                                 // Interpolate position using barycentric coordinates
                                 V3f deformedPosition = trianglePoints[0] * baryCoord[0] +
                                                        trianglePoints[1] * baryCoord[1] +
                                                        trianglePoints[2] * baryCoord[2];

                                 // Get normals for the triangle vertices
                                 V3f deformedNormal;
                                 const V3fVectorData *normalsData = animatedDeformer->variableData<V3fVectorData>("N", PrimitiveVariable::Vertex);
                                 if (normalsData)
                                 {
                                     const std::vector<V3f> &meshNormals = normalsData->readable();
                                     deformedNormal = meshNormals[triangleVertices[0]] * baryCoord[0] +
                                                      meshNormals[triangleVertices[1]] * baryCoord[1] +
                                                      meshNormals[triangleVertices[2]] * baryCoord[2];
                                 }
                                 else
                                 {
                                     // Calculate face normal
                                     deformedNormal = (trianglePoints[1] - trianglePoints[0]).cross(trianglePoints[2] - trianglePoints[0]);
                                 }

                                 // Get tangent using barycentric interpolation
                                 V3f deformedTangent = Detail::primVar<V3f>(animatedTangents.first,
                                                                            &baryCoord[0],
                                                                            triangleIndex,
                                                                            V3i(triangleVertices[0],
                                                                                triangleVertices[1],
                                                                                triangleVertices[2]));

                                 // Build deformed frame with more robust orthonormalization
                                 RestFrame deformedFrame;
                                 deformedFrame.position = deformedPosition;
                                 deformedFrame.normal = deformedNormal;
                                 deformedFrame.tangent = deformedTangent;
                                 deformedFrame.orthonormalize();

                                 // Build matrices exactly like VEX implementation
                                 M44f StaticMatrix(1); // Initialize with identity
                                 // Tangent row
                                 StaticMatrix[0][0] = restTangents[i].x;
                                 StaticMatrix[0][1] = restTangents[i].y;
                                 StaticMatrix[0][2] = restTangents[i].z;
                                 StaticMatrix[0][3] = 0;

                                 // Bitangent row
                                 StaticMatrix[1][0] = restBitangents[i].x;
                                 StaticMatrix[1][1] = restBitangents[i].y;
                                 StaticMatrix[1][2] = restBitangents[i].z;
                                 StaticMatrix[1][3] = 0;

                                 // Normal row
                                 StaticMatrix[2][0] = restNormals[i].x;
                                 StaticMatrix[2][1] = restNormals[i].y;
                                 StaticMatrix[2][2] = restNormals[i].z;
                                 StaticMatrix[2][3] = 0;

                                 // Position row
                                 StaticMatrix[3][0] = restPositions[i].x;
                                 StaticMatrix[3][1] = restPositions[i].y;
                                 StaticMatrix[3][2] = restPositions[i].z;
                                 StaticMatrix[3][3] = 1;

                                 M44f AnimMatrix(1); // Initialize with identity
                                 // Tangent row
                                 AnimMatrix[0][0] = deformedFrame.tangent.x;
                                 AnimMatrix[0][1] = deformedFrame.tangent.y;
                                 AnimMatrix[0][2] = deformedFrame.tangent.z;
                                 AnimMatrix[0][3] = 0;

                                 // Bitangent row
                                 AnimMatrix[1][0] = deformedFrame.bitangent.x;
                                 AnimMatrix[1][1] = deformedFrame.bitangent.y;
                                 AnimMatrix[1][2] = deformedFrame.bitangent.z;
                                 AnimMatrix[1][3] = 0;

                                 // Normal row
                                 AnimMatrix[2][0] = deformedFrame.normal.x;
                                 AnimMatrix[2][1] = deformedFrame.normal.y;
                                 AnimMatrix[2][2] = deformedFrame.normal.z;
                                 AnimMatrix[2][3] = 0;

                                 // Position row
                                 AnimMatrix[3][0] = deformedFrame.position.x;
                                 AnimMatrix[3][1] = deformedFrame.position.y;
                                 AnimMatrix[3][2] = deformedFrame.position.z;
                                 AnimMatrix[3][3] = 1;

                                 // Calculate transform exactly like VEX
                                 M44f Transform = StaticMatrix.inverse() * AnimMatrix;

                                 // Check for extreme transformations
                                 bool useStableTransform = false;
                                 const float maxAllowedScale = 10.0f; // Adjust this threshold as needed

                                 // Check scale factors from the transformation matrix
                                 V3f scaleX(Transform[0][0], Transform[1][0], Transform[2][0]);
                                 V3f scaleY(Transform[0][1], Transform[1][1], Transform[2][1]);
                                 V3f scaleZ(Transform[0][2], Transform[1][2], Transform[2][2]);

                                 float maxScale = std::max({scaleX.length(), scaleY.length(), scaleZ.length()});

                                 if (maxScale > maxAllowedScale)
                                 {
                                     useStableTransform = true;
                                     ++degenerateFrames;
                                 }

                                 // Transform points with fallback for extreme deformations
                                 const size_t startIdx = vertexOffsets[i];
                                 const size_t endIdx = (i + 1 < vertexOffsets.size()) ? vertexOffsets[i + 1] : positions.size();

                                 if (useStableTransform)
                                 {
                                     // Calculate a more stable transformation that preserves shape
                                     V3f restToDeformedOffset = deformedFrame.position - restPositions[i];

                                     // Calculate rotation between rest and deformed normals
                                     V3f restN = restNormals[i];
                                     V3f deformedN = deformedFrame.normal;

                                     // Normalize vectors to ensure valid rotation
                                     restN.normalize();
                                     deformedN.normalize();

                                     // Calculate rotation axis and angle
                                     V3f rotAxis = restN.cross(deformedN);
                                     float rotAngle = std::acos(std::max(-1.0f, std::min(1.0f, restN.dot(deformedN))));

                                     // Build stable rotation matrix
                                     M44f stableRotation;
                                     if (rotAxis.length() > 1e-6f && std::abs(rotAngle) > 1e-6f)
                                     {
                                         rotAxis.normalize();
                                         stableRotation.setAxisAngle(rotAxis, rotAngle);
                                     }

                                     // Apply stable transformation
                                     for (size_t j = startIdx; j < endIdx; ++j)
                                     {
                                         // Get point relative to rest position
                                         V3f localP = positions[j] - restPositions[i];

                                         // Apply rotation
                                         V3f rotatedP;
                                         stableRotation.multDirMatrix(localP, rotatedP);

                                         // Translate to final position
                                         positions[j] = rotatedP + deformedFrame.position;
                                     }
                                 }
                                 else
                                 {
                                     // Use original transformation
                                     for (size_t j = startIdx; j < endIdx; ++j)
                                     {
                                         V3f p = positions[j];
                                         Transform.multVecMatrix(p, positions[j]);
                                     }
                                 }

                                 ++processedCurves;
                             }
                             catch (const std::exception &e)
                             {
                                 ++errorCount;
                                 IECore::msg(IECore::Msg::Error, "RigidDeformCurves",
                                             (boost::format("Error processing curve %d: %s") % i % e.what()).str());
                             }
                         }
                     });

        IECore::msg(IECore::Msg::Info, "RigidDeformCurves",
                    (boost::format("Deformation complete - Processed: %d/%d curves, Degenerate frames: %d, Errors: %d") % processedCurves.load() % numCurves % degenerateFrames.load() % errorCount.load()).str());

        // Update output positions
        outputCurves->variables["P"] = PrimitiveVariable(PrimitiveVariable::Vertex, positionData);
    }
    catch (const std::exception &e)
    {
        IECore::msg(IECore::Msg::Error, "RigidDeformCurves",
                    (boost::format("Deformation failed: %s") % e.what()).str());
        throw;
    }
}