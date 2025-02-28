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
    // Early out if we don't have a valid input object
    const CurvesPrimitive *curves = runTimeCast<const CurvesPrimitive>(inputObject);
    if (!curves)
    {
        return inputObject;
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

    // Read binding data
    const V3fVectorData *restPositionsData = runTimeCast<const V3fVectorData>(curves->variables.at("restPosition").data.get());
    const V3fVectorData *restNormalsData = runTimeCast<const V3fVectorData>(curves->variables.at("restNormal").data.get());
    const V3fVectorData *restTangentsData = runTimeCast<const V3fVectorData>(curves->variables.at("restTangent").data.get());
    const V3fVectorData *restBitangentsData = runTimeCast<const V3fVectorData>(curves->variables.at("restBitangent").data.get());
    const IntVectorData *triangleIndicesData = runTimeCast<const IntVectorData>(curves->variables.at("triangleIndex").data.get());
    const V3fVectorData *barycentricCoordsData = runTimeCast<const V3fVectorData>(curves->variables.at("barycentricCoords").data.get());

    if (!restPositionsData || !restNormalsData || !restTangentsData || !restBitangentsData || !triangleIndicesData || !barycentricCoordsData)
    {
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

    // Calculate tangents for animated mesh
    std::pair<PrimitiveVariable, PrimitiveVariable> animatedTangents;
    bool hasTangents = false;

    // Try to calculate tangents from UVs first
    auto uvIt = animatedMesh->variables.find("uv");
    if (uvIt == animatedMesh->variables.end())
    {
        uvIt = animatedMesh->variables.find("st");
    }
    if (uvIt == animatedMesh->variables.end())
    {
        uvIt = animatedMesh->variables.find("UV");
    }

    if (uvIt != animatedMesh->variables.end())
    {
        animatedTangents = MeshAlgo::calculateTangents(animatedMesh, uvIt->first, true, "P");
        hasTangents = true;
    }
    else
    {
        // If no UVs, calculate tangents from edges
        auto normalIt = animatedMesh->variables.find("N");
        if (normalIt == animatedMesh->variables.end())
        {
            // Calculate vertex normals if not present
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
        throw IECore::Exception("Failed to calculate tangents for animated mesh");
    }

    // Process curves in parallel
    const size_t numCurves = curves->verticesPerCurve()->readable().size();
    const size_t numThreads = std::thread::hardware_concurrency();
    const size_t batchSize = calculateBatchSize(numCurves, numThreads);

    parallel_for(blocked_range<size_t>(0, numCurves, batchSize),
                 [&](const blocked_range<size_t> &range)
                 {
                     for (size_t i = range.begin(); i != range.end(); ++i)
                     {
                         // Reconstruct rest frame
                         RestFrame restFrame;
                         restFrame.position = restPositions[i];
                         restFrame.normal = restNormals[i];
                         restFrame.tangent = restTangents[i];
                         restFrame.bitangent = restBitangents[i];
                         restFrame.orthonormalize();

                         // Get binding data
                         const int triangleIndex = triangleIndices[i];
                         const V3f &baryCoord = barycentricCoords[i];

                         // Evaluate deformed position
                         V3f deformedPosition;
                         V3f deformedNormal;
                         V3f deformedTangent;

                         // Get vertex indices for the triangle
                         const std::vector<int> &vertexIds = animatedMesh->vertexIds()->readable();
                         const int *triangleVertices = &vertexIds[triangleIndex * 3];

                         // Get positions for the triangle vertices
                         const std::vector<V3f> &meshPoints = animatedMesh->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex)->readable();
                         const V3f &p0 = meshPoints[triangleVertices[0]];
                         const V3f &p1 = meshPoints[triangleVertices[1]];
                         const V3f &p2 = meshPoints[triangleVertices[2]];

                         // Interpolate position using barycentric coordinates
                         deformedPosition = p0 * baryCoord[0] + p1 * baryCoord[1] + p2 * baryCoord[2];

                         // Get normals for the triangle vertices (if available)
                         const V3fVectorData *normalsData = animatedMesh->variableData<V3fVectorData>("N", PrimitiveVariable::Vertex);
                         if (normalsData)
                         {
                             const std::vector<V3f> &meshNormals = normalsData->readable();
                             const V3f &n0 = meshNormals[triangleVertices[0]];
                             const V3f &n1 = meshNormals[triangleVertices[1]];
                             const V3f &n2 = meshNormals[triangleVertices[2]];

                             // Interpolate normal using barycentric coordinates
                             deformedNormal = safeNormalize(n0 * baryCoord[0] + n1 * baryCoord[1] + n2 * baryCoord[2]);
                         }
                         else
                         {
                             // Calculate face normal if vertex normals not available
                             deformedNormal = safeNormalize((p1 - p0).cross(p2 - p0));
                         }

                         // Get tangent using barycentric interpolation
                         deformedTangent = Detail::primVar<V3f>(animatedTangents.first,
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

                         // Get transformation matrices
                         M44f restToWorld = restFrame.toMatrix();
                         M44f worldToDeformed = deformedFrame.toMatrix().inverse();

                         // Transform points for this curve
                         const size_t startIdx = vertexOffsets[i];
                         const size_t endIdx = (i + 1 < vertexOffsets.size()) ? vertexOffsets[i + 1] : positions.size();

                         for (size_t j = startIdx; j < endIdx; ++j)
                         {
                             positions[j] = transformPoint(positions[j], restToWorld, worldToDeformed);
                         }
                     }
                 });

    // Update positions in output curves
    outputCurves->variables["P"] = PrimitiveVariable(PrimitiveVariable::Vertex, positionData);

    return outputCurves;
}