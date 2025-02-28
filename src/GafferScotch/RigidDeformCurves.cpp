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
    addChild(new ScenePlug("restMesh", Plug::In));
    addChild(new ScenePlug("animatedMesh", Plug::In));

    // Binding mode
    addChild(new BoolPlug("useBindAttr", Plug::In, false));
    addChild(new StringPlug("bindPath", Plug::In, "")); // Used when useBindAttr=false
    addChild(new StringPlug("bindAttr", Plug::In, "")); // Used when useBindAttr=true

    // Fast pass-throughs for things we don't modify
    outPlug()->attributesPlug()->setInput(inPlug()->attributesPlug());
    outPlug()->transformPlug()->setInput(inPlug()->transformPlug());
    outPlug()->boundPlug()->setInput(inPlug()->boundPlug());
}

RigidDeformCurves::~RigidDeformCurves()
{
}

ScenePlug *RigidDeformCurves::restMeshPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

const ScenePlug *RigidDeformCurves::restMeshPlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

ScenePlug *RigidDeformCurves::animatedMeshPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex + 1);
}

const ScenePlug *RigidDeformCurves::animatedMeshPlug() const
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

StringPlug *RigidDeformCurves::bindPathPlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 3);
}

const StringPlug *RigidDeformCurves::bindPathPlug() const
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

void RigidDeformCurves::affects(const Gaffer::Plug *input, AffectedPlugsContainer &outputs) const
{
    Deformer::affects(input, outputs);

    if (input == restMeshPlug()->objectPlug() ||
        input == animatedMeshPlug()->objectPlug() ||
        input == useBindAttrPlug() ||
        input == bindPathPlug() ||
        input == bindAttrPlug())
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
    return input == restMeshPlug()->objectPlug() ||
           input == animatedMeshPlug()->objectPlug() ||
           input == useBindAttrPlug() ||
           input == bindPathPlug() ||
           input == bindAttrPlug();
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
        meshPath = GafferScotch::makeScenePath(bindPathPlug()->getValue());
    }

    // Hash meshes
    h.append(restMeshPlug()->objectHash(meshPath));
    h.append(animatedMeshPlug()->objectHash(meshPath));

    // Hash input curves and binding data
    h.append(inPlug()->objectHash(path));
    hashBindingData(curves, h);

    // Hash parameters
    useBindAttrPlug()->hash(h);
    bindPathPlug()->hash(h);
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
                            boost::format("Bind attribute '%s' not found") % bindAttrName);
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
            meshPath = GafferScotch::makeScenePath(bindPathPlug()->getValue());
        }

        if (meshPath.empty())
        {
            IECore::msg(IECore::Msg::Warning, "RigidDeformCurves", "Empty mesh path");
            return inputObject;
        }

        IECore::msg(IECore::Msg::Info, "RigidDeformCurves",
                    boost::format("Resolved mesh path: %s") % boost::join(meshPath, "/"));

        // Get meshes using resolved path
        ConstObjectPtr restMeshObj = restMeshPlug()->object(meshPath);
        if (!restMeshObj)
        {
            IECore::msg(IECore::Msg::Warning, "RigidDeformCurves", "Null rest mesh object");
            return inputObject;
        }

        const MeshPrimitive *restMesh = runTimeCast<const MeshPrimitive>(restMeshObj.get());
        if (!restMesh)
        {
            IECore::msg(IECore::Msg::Warning, "RigidDeformCurves", "Rest mesh is not a MeshPrimitive");
            return inputObject;
        }

        ConstObjectPtr animatedMeshObj = animatedMeshPlug()->object(meshPath);
        if (!animatedMeshObj)
        {
            IECore::msg(IECore::Msg::Warning, "RigidDeformCurves", "Null animated mesh object");
            return inputObject;
        }

        const MeshPrimitive *animatedMesh = runTimeCast<const MeshPrimitive>(animatedMeshObj.get());
        if (!animatedMesh)
        {
            IECore::msg(IECore::Msg::Warning, "RigidDeformCurves", "Animated mesh is not a MeshPrimitive");
            return inputObject;
        }

        IECore::msg(IECore::Msg::Info, "RigidDeformCurves", "Mesh objects validated");

        // Validate mesh topology
        if (!restMesh->vertexIds() || !animatedMesh->vertexIds())
        {
            IECore::msg(IECore::Msg::Warning, "RigidDeformCurves", "Missing vertex IDs");
            return inputObject;
        }

        if (!restMesh->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex) ||
            !animatedMesh->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex))
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
            deformCurves(curves, restMesh, animatedMesh, outputCurves.get());
        }
        catch (const std::exception &e)
        {
            IECore::msg(IECore::Msg::Error, "RigidDeformCurves",
                        boost::format("Deformation failed: %s") % e.what());
            return inputObject;
        }

        IECore::msg(IECore::Msg::Info, "RigidDeformCurves", "Deformation completed successfully");
        return outputCurves;
    }
    catch (const std::exception &e)
    {
        IECore::msg(IECore::Msg::Error, "RigidDeformCurves",
                    boost::format("Computation failed: %s") % e.what());
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
    return input == restMeshPlug()->objectPlug() ||
           input == animatedMeshPlug()->objectPlug() ||
           input == useBindAttrPlug() ||
           input == bindPathPlug() ||
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
        meshPath = GafferScotch::makeScenePath(bindPathPlug()->getValue());
    }

    // Hash meshes
    h.append(restMeshPlug()->objectHash(meshPath));
    h.append(animatedMeshPlug()->objectHash(meshPath));

    // Hash input curves and binding data
    h.append(inPlug()->boundHash(path));
    hashBindingData(curves, h);

    // Hash parameters
    useBindAttrPlug()->hash(h);
    bindPathPlug()->hash(h);
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
        meshPath = GafferScotch::makeScenePath(bindPathPlug()->getValue());
    }

    // Get meshes using resolved path
    ConstObjectPtr restMeshObj = restMeshPlug()->object(meshPath);
    const MeshPrimitive *restMesh = runTimeCast<const MeshPrimitive>(restMeshObj.get());

    ConstObjectPtr animatedMeshObj = animatedMeshPlug()->object(meshPath);
    const MeshPrimitive *animatedMesh = runTimeCast<const MeshPrimitive>(animatedMeshObj.get());

    if (!restMesh || !animatedMesh)
    {
        return inPlug()->bound(path);
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

    // Deform curves to get accurate bounds
    deformCurves(curves, restMesh, animatedMesh, outputCurves.get());

    return outputCurves->bound();
}

void RigidDeformCurves::deformCurves(
    const IECoreScene::CurvesPrimitive *curves,
    const IECoreScene::MeshPrimitive *restMesh,
    const IECoreScene::MeshPrimitive *animatedMesh,
    IECoreScene::CurvesPrimitive *outputCurves) const
{
    if (!curves || !restMesh || !animatedMesh || !outputCurves)
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
                    boost::format("Input: %d curves, Rest mesh: %d verts, %d faces, Animated mesh: %d verts, %d faces") % curves->verticesPerCurve()->readable().size() % restMesh->variableSize(PrimitiveVariable::Vertex) % (restMesh->vertexIds()->readable().size() / 3) % animatedMesh->variableSize(PrimitiveVariable::Vertex) % (animatedMesh->vertexIds()->readable().size() / 3));

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

        IECore::msg(IECore::Msg::Info, "RigidDeformCurves", "Validating binding data sizes");

        // Log binding data sizes
        IECore::msg(IECore::Msg::Info, "RigidDeformCurves",
                    boost::format("Binding data sizes - Rest positions: %d, Normals: %d, Tangents: %d, Bitangents: %d, Triangle indices: %d, Barycentric coords: %d") % restPositionsData->readable().size() % restNormalsData->readable().size() % restTangentsData->readable().size() % restBitangentsData->readable().size() % triangleIndicesData->readable().size() % barycentricCoordsData->readable().size());

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

        const std::vector<int> &vertexIds = animatedMesh->vertexIds()->readable();
        const size_t numTriangles = vertexIds.size() / 3;
        const std::vector<V3f> &meshPoints = animatedMesh->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex)->readable();

        IECore::msg(IECore::Msg::Info, "RigidDeformCurves", "Calculating parallel processing parameters");

        const size_t numCurves = curves->verticesPerCurve()->readable().size();
        const size_t numThreads = std::thread::hardware_concurrency();
        const size_t batchSize = calculateBatchSize(numCurves, numThreads);

        IECore::msg(IECore::Msg::Info, "RigidDeformCurves",
                    boost::format("Processing setup - Curves: %d, Threads: %d, Batch size: %d") % numCurves % numThreads % batchSize);

        IECore::msg(IECore::Msg::Info, "RigidDeformCurves", "Validating triangle indices");

        size_t invalidTriangles = 0;
        size_t invalidVertices = 0;

        // Pre-validate triangle indices
        for (size_t i = 0; i < triangleIndices.size(); ++i)
        {
            const int triangleIndex = triangleIndices[i];
            if (triangleIndex < 0 || triangleIndex >= numTriangles)
            {
                invalidTriangles++;
                IECore::msg(IECore::Msg::Error, "RigidDeformCurves",
                            boost::format("Invalid triangle index %d for curve %d (max: %d)") % triangleIndex % i % (numTriangles - 1));
                continue;
            }

            // Check vertex indices
            const int *triangleVertices = &vertexIds[triangleIndex * 3];
            for (int j = 0; j < 3; ++j)
            {
                if (triangleVertices[j] < 0 || triangleVertices[j] >= meshPoints.size())
                {
                    invalidVertices++;
                    IECore::msg(IECore::Msg::Error, "RigidDeformCurves",
                                boost::format("Invalid vertex index %d for triangle %d vertex %d (max: %d)") % triangleVertices[j] % triangleIndex % j % (meshPoints.size() - 1));
                }
            }
        }

        if (invalidTriangles > 0 || invalidVertices > 0)
        {
            IECore::msg(IECore::Msg::Error, "RigidDeformCurves",
                        boost::format("Validation failed - Invalid triangles: %d, Invalid vertices: %d") % invalidTriangles % invalidVertices);
            throw IECore::Exception("Invalid topology data");
        }

        IECore::msg(IECore::Msg::Info, "RigidDeformCurves", "Calculating tangents for animated mesh");

        // Calculate tangents for animated mesh
        std::pair<PrimitiveVariable, PrimitiveVariable> animatedTangents;
        bool hasTangents = false;

        // Try UV-based tangents first
        auto uvIt = animatedMesh->variables.find("uv");
        if (uvIt == animatedMesh->variables.end())
            uvIt = animatedMesh->variables.find("st");
        if (uvIt == animatedMesh->variables.end())
            uvIt = animatedMesh->variables.find("UV");

        if (uvIt != animatedMesh->variables.end())
        {
            IECore::msg(IECore::Msg::Info, "RigidDeformCurves",
                        boost::format("Computing UV-based tangents using '%s' attribute") % uvIt->first);
            animatedTangents = MeshAlgo::calculateTangents(animatedMesh, uvIt->first, true, "P");
            hasTangents = true;
        }
        else
        {
            IECore::msg(IECore::Msg::Info, "RigidDeformCurves", "No UVs found, computing edge-based tangents");

            // Try edge-based tangents
            auto normalIt = animatedMesh->variables.find("N");
            if (normalIt == animatedMesh->variables.end())
            {
                IECore::msg(IECore::Msg::Info, "RigidDeformCurves", "Computing vertex normals");
                PrimitiveVariable normals = MeshAlgo::calculateNormals(animatedMesh);
                const_cast<MeshPrimitive *>(animatedMesh)->variables["N"] = normals;
                normalIt = animatedMesh->variables.find("N");
            }

            if (normalIt != animatedMesh->variables.end())
            {
                animatedTangents = MeshAlgo::calculateTangentsFromTwoEdges(animatedMesh, "P", normalIt->first, true, false);
                hasTangents = true;
            }
        }

        if (!hasTangents)
        {
            IECore::msg(IECore::Msg::Error, "RigidDeformCurves", "Failed to calculate tangents");
            throw IECore::Exception("Failed to calculate tangents for animated mesh");
        }

        IECore::msg(IECore::Msg::Info, "RigidDeformCurves", "Starting parallel curve processing");

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
                                 // Process curve
                                 RestFrame restFrame;
                                 restFrame.position = restPositions[i];
                                 restFrame.normal = restNormals[i];
                                 restFrame.tangent = restTangents[i];
                                 restFrame.bitangent = restBitangents[i];
                                 restFrame.orthonormalize();

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

                                 // Get normals for the triangle vertices (if available)
                                 V3f deformedNormal;
                                 const V3fVectorData *normalsData = animatedMesh->variableData<V3fVectorData>("N", PrimitiveVariable::Vertex);
                                 if (normalsData)
                                 {
                                     const std::vector<V3f> &meshNormals = normalsData->readable();
                                     deformedNormal = safeNormalize(meshNormals[triangleVertices[0]] * baryCoord[0] +
                                                                    meshNormals[triangleVertices[1]] * baryCoord[1] +
                                                                    meshNormals[triangleVertices[2]] * baryCoord[2]);
                                 }
                                 else
                                 {
                                     // Calculate face normal if vertex normals not available
                                     deformedNormal = safeNormalize((trianglePoints[1] - trianglePoints[0]).cross(trianglePoints[2] - trianglePoints[0]));
                                 }

                                 // Get tangent using barycentric interpolation
                                 V3f deformedTangent = Detail::primVar<V3f>(animatedTangents.first,
                                                                            &baryCoord[0],
                                                                            triangleIndex,
                                                                            V3i(triangleVertices[0],
                                                                                triangleVertices[1],
                                                                                triangleVertices[2]));

                                 // Build deformed frame
                                 RestFrame deformedFrame;
                                 deformedFrame.position = deformedPosition;
                                 deformedFrame.normal = deformedNormal;
                                 deformedFrame.tangent = deformedTangent;
                                 deformedFrame.orthonormalize();

                                 // Check for degenerate frame
                                 M44f deformedMatrix = deformedFrame.toMatrix();
                                 if (std::abs(deformedMatrix.determinant()) < 1e-6)
                                 {
                                     ++degenerateFrames;
                                     IECore::msg(IECore::Msg::Warning, "RigidDeformCurves",
                                                 boost::format("Near-singular matrix for curve %d") % i);
                                     continue;
                                 }

                                 // Transform points
                                 M44f restToWorld = restFrame.toMatrix();
                                 M44f worldToDeformed = deformedMatrix.inverse();

                                 const size_t startIdx = vertexOffsets[i];
                                 const size_t endIdx = (i + 1 < vertexOffsets.size()) ? vertexOffsets[i + 1] : positions.size();

                                 for (size_t j = startIdx; j < endIdx; ++j)
                                 {
                                     positions[j] = transformPoint(positions[j], restToWorld, worldToDeformed);
                                 }

                                 ++processedCurves;
                             }
                             catch (const std::exception &e)
                             {
                                 ++errorCount;
                                 IECore::msg(IECore::Msg::Error, "RigidDeformCurves",
                                             boost::format("Error processing curve %d: %s") % i % e.what());
                             }
                         }
                     });

        IECore::msg(IECore::Msg::Info, "RigidDeformCurves",
                    boost::format("Deformation complete - Processed: %d/%d curves, Degenerate frames: %d, Errors: %d") % processedCurves.load() % numCurves % degenerateFrames.load() % errorCount.load());

        // Update output positions
        outputCurves->variables["P"] = PrimitiveVariable(PrimitiveVariable::Vertex, positionData);
    }
    catch (const std::exception &e)
    {
        IECore::msg(IECore::Msg::Error, "RigidDeformCurves",
                    boost::format("Deformation failed: %s") % e.what());
        throw;
    }
}