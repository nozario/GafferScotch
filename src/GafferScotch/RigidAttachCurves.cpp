#include "GafferScotch/RigidAttachCurves.h"
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
    };

    struct CurveBinding
    {
        int triangleIndex;
        V3f baryCoords;
        V2f uvCoords;
        V3f rootPointOffset;
        RestFrame restFrame;
        bool valid;

        CurveBinding() : triangleIndex(-1), valid(false) {}
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

    // Helper to hash normal data
    void hashNormals(const Primitive *primitive, MurmurHash &h)
    {
        if (!primitive)
            return;

        auto it = primitive->variables.find("N");
        if (it == primitive->variables.end())
            return;

        const V3fVectorData *normals = runTimeCast<const V3fVectorData>(it->second.data.get());
        if (!normals)
            return;

        // Hash only the normal data
        const std::vector<V3f> &norms = normals->readable();
        h.append(norms.size());
        if (!norms.empty())
        {
            h.append(&norms[0], norms.size());
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

IE_CORE_DEFINERUNTIMETYPED(RigidAttachCurves);

size_t RigidAttachCurves::g_firstPlugIndex = 0;

RigidAttachCurves::RigidAttachCurves(const std::string &name)
    : ObjectProcessor(name)
{
    storeIndexOfNextChild(g_firstPlugIndex);

    // Add rest mesh input
    addChild(new ScenePlug("restMesh", Plug::In));

    // Root finding
    addChild(new StringPlug("rootAttr", Plug::In, ""));

    // Binding mode
    addChild(new BoolPlug("useBindAttr", Plug::In, false));
    addChild(new StringPlug("bindPath", Plug::In, "")); // Used when useBindAttr=false
    addChild(new StringPlug("bindAttr", Plug::In, "")); // Used when useBindAttr=true

    // Fast pass-throughs for things we don't modify
    outPlug()->attributesPlug()->setInput(inPlug()->attributesPlug());
    outPlug()->transformPlug()->setInput(inPlug()->transformPlug());
    outPlug()->boundPlug()->setInput(inPlug()->boundPlug());
}

RigidAttachCurves::~RigidAttachCurves()
{
}

ScenePlug *RigidAttachCurves::restMeshPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

const ScenePlug *RigidAttachCurves::restMeshPlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

StringPlug *RigidAttachCurves::rootAttrPlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 1);
}

const StringPlug *RigidAttachCurves::rootAttrPlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 1);
}

BoolPlug *RigidAttachCurves::useBindAttrPlug()
{
    return getChild<BoolPlug>(g_firstPlugIndex + 2);
}

const BoolPlug *RigidAttachCurves::useBindAttrPlug() const
{
    return getChild<BoolPlug>(g_firstPlugIndex + 2);
}

StringPlug *RigidAttachCurves::bindPathPlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 3);
}

const StringPlug *RigidAttachCurves::bindPathPlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 3);
}

StringPlug *RigidAttachCurves::bindAttrPlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 4);
}

const StringPlug *RigidAttachCurves::bindAttrPlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 4);
}

void RigidAttachCurves::affects(const Gaffer::Plug *input, AffectedPlugsContainer &outputs) const
{
    ObjectProcessor::affects(input, outputs);

    if (input == restMeshPlug()->objectPlug() ||
        input == rootAttrPlug() ||
        input == useBindAttrPlug() ||
        input == bindPathPlug() ||
        input == bindAttrPlug())
    {
        outputs.push_back(outPlug()->objectPlug());
    }
}

bool RigidAttachCurves::acceptsInput(const Gaffer::Plug *plug, const Gaffer::Plug *inputPlug) const
{
    if (!ObjectProcessor::acceptsInput(plug, inputPlug))
    {
        return false;
    }

    return true;
}

bool RigidAttachCurves::affectsProcessedObject(const Gaffer::Plug *input) const
{
    return input == restMeshPlug()->objectPlug() ||
           input == rootAttrPlug() ||
           input == useBindAttrPlug() ||
           input == bindPathPlug() ||
           input == bindAttrPlug();
}

void RigidAttachCurves::hashProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
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
    ScenePath restPath;
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
                    restPath = GafferScotch::makeScenePath(pathData->readable());
                }
                else if (const StringVectorData *pathVectorData = runTimeCast<const StringVectorData>(it->second.data.get()))
                {
                    const std::vector<std::string> &paths = pathVectorData->readable();
                    if (!paths.empty())
                    {
                        restPath = GafferScotch::makeScenePath(paths[0]); // Use first path for now
                    }
                }
            }
        }
    }
    else
    {
        // Use path from plug
        restPath = GafferScotch::makeScenePath(bindPathPlug()->getValue());
    }

    // Hash rest mesh
    h.append(restMeshPlug()->objectHash(restPath));

    // Hash input curves
    h.append(inPlug()->objectHash(path));

    // Hash parameters
    rootAttrPlug()->hash(h);
    useBindAttrPlug()->hash(h);
    bindPathPlug()->hash(h);
    bindAttrPlug()->hash(h);
}

size_t RigidAttachCurves::findRootPointIndex(
    const CurvesPrimitive *curves,
    const std::vector<size_t> &vertexOffsets,
    size_t curveIndex) const
{
    // Get root attribute name
    const std::string rootAttrName = rootAttrPlug()->getValue();
    if (rootAttrName.empty())
    {
        // If no attribute specified, use first point
        return vertexOffsets[curveIndex];
    }

    // Try to find root attribute
    auto it = curves->variables.find(rootAttrName);
    if (it == curves->variables.end())
    {
        // Attribute not found, fall back to first point
        return vertexOffsets[curveIndex];
    }

    // Get attribute data
    const FloatVectorData *rootData = runTimeCast<const FloatVectorData>(it->second.data.get());
    if (!rootData)
    {
        // Invalid data type, fall back to first point
        return vertexOffsets[curveIndex];
    }

    // Get vertex range for this curve
    const size_t startIdx = vertexOffsets[curveIndex];
    const size_t numVerts = (curveIndex + 1 < vertexOffsets.size()) ? vertexOffsets[curveIndex + 1] - startIdx : rootData->readable().size() - startIdx;

    // Find highest value in range (root point has value 1)
    size_t rootIdx = startIdx;
    float maxVal = 0.0f;
    const std::vector<float> &values = rootData->readable();

    for (size_t i = 0; i < numVerts; ++i)
    {
        const size_t idx = startIdx + i;
        if (idx < values.size() && values[idx] > maxVal)
        {
            maxVal = values[idx];
            rootIdx = idx;
        }
    }

    return rootIdx;
}

void RigidAttachCurves::computeBindings(const MeshPrimitive *restMesh,
                                        const CurvesPrimitive *curves,
                                        CurvesPrimitive *outputCurves) const
{
    // Create evaluator for rest mesh
    MeshPrimitivePtr triangulatedRestMesh = MeshAlgo::triangulate(restMesh);
    MeshPrimitiveEvaluatorPtr restEvaluator = new MeshPrimitiveEvaluator(triangulatedRestMesh);

    // Calculate tangents for rest mesh
    std::pair<PrimitiveVariable, PrimitiveVariable> restTangents;
    bool hasTangents = false;

    // Try to calculate tangents from UVs first
    auto uvIt = triangulatedRestMesh->variables.find("uv");
    if (uvIt == triangulatedRestMesh->variables.end())
    {
        uvIt = triangulatedRestMesh->variables.find("st");
    }
    if (uvIt == triangulatedRestMesh->variables.end())
    {
        uvIt = triangulatedRestMesh->variables.find("UV");
    }

    if (uvIt != triangulatedRestMesh->variables.end())
    {
        restTangents = MeshAlgo::calculateTangents(triangulatedRestMesh.get(), uvIt->first, true, "P");
        hasTangents = true;
    }
    else
    {
        // If no UVs, calculate tangents from edges
        auto normalIt = triangulatedRestMesh->variables.find("N");
        if (normalIt == triangulatedRestMesh->variables.end())
        {
            // Calculate vertex normals if not present
            PrimitiveVariable normals = MeshAlgo::calculateNormals(triangulatedRestMesh.get());
            triangulatedRestMesh->variables["N"] = normals;
            normalIt = triangulatedRestMesh->variables.find("N");
        }

        if (normalIt != triangulatedRestMesh->variables.end())
        {
            restTangents = MeshAlgo::calculateTangentsFromTwoEdges(triangulatedRestMesh.get(), "P", normalIt->first, true, false);
            hasTangents = true;
        }
    }

    if (!hasTangents)
    {
        throw IECore::Exception("Failed to calculate tangents for rest mesh");
    }

    // Get curve data
    const std::vector<int> &vertsPerCurve = curves->verticesPerCurve()->readable();
    std::vector<size_t> vertexOffsets;
    vertexOffsets.reserve(vertsPerCurve.size());

    size_t offset = 0;
    for (int count : vertsPerCurve)
    {
        vertexOffsets.push_back(offset);
        offset += count;
    }

    auto pIt = curves->variables.find("P");
    if (pIt == curves->variables.end())
        return;

    const V3fVectorData *posData = runTimeCast<const V3fVectorData>(pIt->second.data.get());
    if (!posData)
        return;

    const std::vector<V3f> &points = posData->readable();

    // Process curves in parallel
    const size_t numCurves = vertsPerCurve.size();
    const size_t numThreads = std::thread::hardware_concurrency();
    const size_t batchSize = calculateBatchSize(numCurves, numThreads);

    std::vector<CurveBinding> bindings(numCurves);

    parallel_for(blocked_range<size_t>(0, numCurves, batchSize),
                 [&](const blocked_range<size_t> &range)
                 {
                     // Thread-local evaluator result
                     PrimitiveEvaluator::ResultPtr result = restEvaluator->createResult();
                     MeshPrimitiveEvaluator::Result *meshResult = static_cast<MeshPrimitiveEvaluator::Result *>(result.get());

                     for (size_t i = range.begin(); i != range.end(); ++i)
                     {
                         CurveBinding &binding = bindings[i];

                         // Find root point using attribute or first point
                         const size_t rootIdx = findRootPointIndex(curves, vertexOffsets, i);
                         const V3f &rootP = points[rootIdx];

                         // Find closest point on rest mesh for the root point only
                         if (!restEvaluator->closestPoint(rootP, meshResult))
                         {
                             binding.valid = false;
                             continue;
                         }

                         // Store binding data
                         binding.triangleIndex = meshResult->triangleIndex();
                         binding.baryCoords = meshResult->barycentricCoordinates();
                         binding.uvCoords = meshResult->uv();
                         binding.rootPointOffset = rootP - meshResult->point();

                         // Build and store rest frame
                         binding.restFrame.position = meshResult->point();
                         binding.restFrame.normal = meshResult->normal();

                         // Get tangent using barycentric interpolation
                         binding.restFrame.tangent = Detail::primVar<V3f>(restTangents.first,
                                                                          &binding.baryCoords[0],
                                                                          binding.triangleIndex,
                                                                          V3i(meshResult->vertexIds()[0],
                                                                              meshResult->vertexIds()[1],
                                                                              meshResult->vertexIds()[2]));

                         binding.restFrame.orthonormalize();
                         binding.valid = true;
                     }
                 });

    // Store binding data as primitive variables
    V3fVectorDataPtr restPositionsData = new V3fVectorData;
    V3fVectorDataPtr restNormalsData = new V3fVectorData;
    V3fVectorDataPtr restTangentsData = new V3fVectorData;
    V3fVectorDataPtr restBitangentsData = new V3fVectorData;
    V3fVectorDataPtr rootPointsData = new V3fVectorData;

    std::vector<V3f> &restPositions = restPositionsData->writable();
    std::vector<V3f> &restNormals = restNormalsData->writable();
    std::vector<V3f> &restTangents = restTangentsData->writable();
    std::vector<V3f> &restBitangents = restBitangentsData->writable();
    std::vector<V3f> &rootPoints = rootPointsData->writable();

    // Pre-allocate arrays
    restPositions.resize(numCurves);
    restNormals.resize(numCurves);
    restTangents.resize(numCurves);
    restBitangents.resize(numCurves);
    rootPoints.resize(numCurves);

    // Store optimization data
    IntVectorDataPtr triangleIndicesData = new IntVectorData;
    V3fVectorDataPtr barycentricCoordsData = new V3fVectorData;
    V2fVectorDataPtr uvCoordsData = new V2fVectorData;

    std::vector<int> &triangleIndices = triangleIndicesData->writable();
    std::vector<V3f> &barycentricCoords = barycentricCoordsData->writable();
    std::vector<V2f> &uvCoords = uvCoordsData->writable();

    triangleIndices.resize(numCurves);
    barycentricCoords.resize(numCurves);
    uvCoords.resize(numCurves);

    // Copy data from bindings
    for (size_t i = 0; i < numCurves; ++i)
    {
        const CurveBinding &binding = bindings[i];
        if (!binding.valid)
            continue;

        restPositions[i] = binding.restFrame.position;
        restNormals[i] = binding.restFrame.normal;
        restTangents[i] = binding.restFrame.tangent;
        restBitangents[i] = binding.restFrame.bitangent;
        rootPoints[i] = binding.restFrame.position + binding.rootPointOffset;

        triangleIndices[i] = binding.triangleIndex;
        barycentricCoords[i] = binding.baryCoords;
        uvCoords[i] = binding.uvCoords;
    }

    // Store all data as primitive variables
    outputCurves->variables["restPosition"] = PrimitiveVariable(PrimitiveVariable::Uniform, restPositionsData);
    outputCurves->variables["restNormal"] = PrimitiveVariable(PrimitiveVariable::Uniform, restNormalsData);
    outputCurves->variables["restTangent"] = PrimitiveVariable(PrimitiveVariable::Uniform, restTangentsData);
    outputCurves->variables["restBitangent"] = PrimitiveVariable(PrimitiveVariable::Uniform, restBitangentsData);
    outputCurves->variables["rootPoint"] = PrimitiveVariable(PrimitiveVariable::Uniform, rootPointsData);

    // Store optimization data
    outputCurves->variables["triangleIndex"] = PrimitiveVariable(PrimitiveVariable::Uniform, triangleIndicesData);
    outputCurves->variables["barycentricCoords"] = PrimitiveVariable(PrimitiveVariable::Uniform, barycentricCoordsData);
    outputCurves->variables["uvCoords"] = PrimitiveVariable(PrimitiveVariable::Uniform, uvCoordsData);
}

IECore::ConstObjectPtr RigidAttachCurves::computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject) const
{
    // Early out if we don't have a valid input object
    const CurvesPrimitive *curves = runTimeCast<const CurvesPrimitive>(inputObject);
    if (!curves)
    {
        return inputObject;
    }

    // Get the target path based on mode
    ScenePath restPath;
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
                    restPath = GafferScotch::makeScenePath(pathData->readable());
                }
                else if (const StringVectorData *pathVectorData = runTimeCast<const StringVectorData>(it->second.data.get()))
                {
                    const std::vector<std::string> &paths = pathVectorData->readable();
                    if (!paths.empty())
                    {
                        restPath = GafferScotch::makeScenePath(paths[0]); // Use first path for now
                    }
                }
            }
        }
    }
    else
    {
        // Use path from plug
        restPath = GafferScotch::makeScenePath(bindPathPlug()->getValue());
    }

    // Get rest mesh using resolved path
    ConstObjectPtr restMeshObj = restMeshPlug()->object(restPath);
    const MeshPrimitive *restMesh = runTimeCast<const MeshPrimitive>(restMeshObj.get());

    if (!restMesh)
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

    // Compute and store bindings
    computeBindings(restMesh, curves, outputCurves.get());

    return outputCurves;
}