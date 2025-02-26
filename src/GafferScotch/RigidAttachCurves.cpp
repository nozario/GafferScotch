#include "GafferScotch/RigidAttachCurves.h"
#include "GafferScotch/AttachCurvesDataStructures.h"

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

    GafferScene::ScenePlug::ScenePath makeScenePath(const std::string &p)
    {
        GafferScene::ScenePlug::ScenePath output;
        IECore::StringAlgo::tokenize<IECore::InternedString>(p, '/', std::back_inserter(output));
        return output;
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
    : AttributeProcessor(name)
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
    AttributeProcessor::affects(input, outputs);

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
    if (!AttributeProcessor::acceptsInput(plug, inputPlug))
    {
        return false;
    }

    return true;
}

bool RigidAttachCurves::affectsProcessedAttributes(const Gaffer::Plug *input) const
{
    // We don't actually modify attributes, so return false
    return false;
}

void RigidAttachCurves::hashProcessedAttributes(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    // We don't modify attributes, so just pass through the input hash
    h = inPlug()->attributesPlug()->hash();
}

IECore::ConstCompoundObjectPtr RigidAttachCurves::computeProcessedAttributes(const ScenePath &path, const Gaffer::Context *context, const IECore::CompoundObject *inputAttributes) const
{
    // We don't modify attributes, so just return the input
    return inputAttributes;
}

void RigidAttachCurves::hashObject(const ScenePath &path, const Gaffer::Context *context, const GafferScene::ScenePlug *parent, IECore::MurmurHash &h) const
{
    FilteredSceneProcessor::hashObject(path, context, parent, h);
    
    // Only hash if the filter matches
    const PathMatcher &filter = filterPlug()->getValue();
    if (filter.match(path) & Filter::ExactMatch)
    {
        // Hash all inputs that affect the object
        h.append(inPlug()->objectHash(path));
        
        // Get the target path based on mode
        ScenePath restPath;
        const bool useBindAttr = useBindAttrPlug()->getValue();
        if (!useBindAttr)
        {
            // Use path from plug
            restPath = makeScenePath(bindPathPlug()->getValue());
            h.append(restMeshPlug()->objectHash(restPath));
        }
        else
        {
            // We can't know the path without looking at the object, so hash all inputs
            bindAttrPlug()->hash(h);
        }
        
        rootAttrPlug()->hash(h);
        useBindAttrPlug()->hash(h);
        bindPathPlug()->hash(h);
    }
}

IECore::ConstObjectPtr RigidAttachCurves::computeObject(const ScenePath &path, const Gaffer::Context *context, const GafferScene::ScenePlug *parent) const
{
    // Only process if the filter matches
    const PathMatcher &filter = filterPlug()->getValue();
    if (!(filter.match(path) & Filter::ExactMatch))
    {
        return inPlug()->objectPlug()->getValue();
    }
    
    // Get input object
    ConstObjectPtr inputObject = inPlug()->object(path);
    const CurvesPrimitive *curves = runTimeCast<const CurvesPrimitive>(inputObject.get());
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
                    restPath = makeScenePath(pathData->readable());
                }
                else if (const StringVectorData *pathVectorData = runTimeCast<const StringVectorData>(it->second.data.get()))
                {
                    const std::vector<std::string> &paths = pathVectorData->readable();
                    if (!paths.empty())
                    {
                        restPath = makeScenePath(paths[0]); // Use first path for now
                    }
                }
            }
        }
    }
    else
    {
        // Use path from plug
        restPath = makeScenePath(bindPathPlug()->getValue());
    }
    
    // Get rest mesh using resolved path
    ConstObjectPtr restMeshObj = restMeshPlug()->object(restPath);
    const MeshPrimitive *restMesh = runTimeCast<const MeshPrimitive>(restMeshObj.get());
    
    if (!restMesh)
    {
        return inputObject;
    }
    
    // Get hashes for cache validation
    MurmurHash restMeshHash = restMeshPlug()->objectHash(restPath);
    MurmurHash curvesHash = inPlug()->objectHash(path);
    
    // Update rest cache if needed
    updateRestCache(restMesh, curves, restMeshHash, curvesHash);
    
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

void RigidAttachCurves::updateRestCache(const MeshPrimitive *restMesh,
                                        const CurvesPrimitive *curves,
                                        const MurmurHash &restMeshHash,
                                        const MurmurHash &curvesHash) const
{
    // Check if cache is valid
    if (m_restCache.valid &&
        m_restCache.restMeshHash == restMeshHash &&
        m_restCache.curvesHash == curvesHash)
    {
        return;
    }

    // Initialize curve data
    m_restCache.curveData.initFromCurves(curves);

    // Get rest mesh data with proper normal/tangent handling
    auto resampleNormals = [](const MeshPrimitive *mesh) -> PrimitiveVariable
    {
        auto normalIt = mesh->variables.find("N");
        if (normalIt == mesh->variables.end())
        {
            throw InvalidArgumentException("MeshPrimitive has no 'N' primitive variable.");
        }

        if (normalIt->second.interpolation == PrimitiveVariable::FaceVarying)
        {
            throw InvalidArgumentException("FaceVarying normal interpolation not yet supported.");
        }

        return normalIt->second;
    };

    // Get normals with proper interpolation
    PrimitiveVariable restNormals = resampleNormals(restMesh);

    // Calculate tangents for rest mesh
    auto calculateMeshTangents = [](const MeshPrimitive *mesh)
    {
        // Check for UV coordinates
        auto uvIt = mesh->variables.find("uv");
        if (uvIt == mesh->variables.end())
        {
            uvIt = mesh->variables.find("st");
        }
        if (uvIt == mesh->variables.end())
        {
            uvIt = mesh->variables.find("UV");
        }

        if (uvIt != mesh->variables.end())
        {
            return MeshAlgo::calculateTangents(mesh, uvIt->first, true, "P");
        }

        auto normalIt = mesh->variables.find("N");
        if (normalIt == mesh->variables.end())
        {
            throw InvalidArgumentException("MeshPrimitive has no 'N' primitive variable.");
        }

        return MeshAlgo::calculateTangentsFromTwoEdges(mesh, "P", normalIt->first, true, false);
    };

    std::pair<PrimitiveVariable, PrimitiveVariable> restTangents = calculateMeshTangents(restMesh);

    // Initialize rest mesh data and store tangents
    m_restCache.restMeshData.initFromMesh(restMesh, restNormals, restTangents.first);
    m_restCache.restTangents = restTangents.first;

    // Initialize binding cache
    m_restCache.bindingCache.initializeBindings(m_restCache.curveData.vertsPerCurve.size());

    // Update hashes
    m_restCache.restMeshHash = restMeshHash;
    m_restCache.curvesHash = curvesHash;
    m_restCache.valid = true;
}

void RigidAttachCurves::buildSpatialIndex(const MeshPrimitive *mesh, MeshPrimitiveEvaluator *evaluator) const
{
    // The evaluator already builds its internal BVH, so we don't need additional work here
    // This is a hook for future optimizations if needed
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

    // Build spatial index for acceleration
    buildSpatialIndex(triangulatedRestMesh.get(), restEvaluator.get());

    // Get curve data - copy instead of reference to handle different allocators
    std::vector<int> vertsPerCurve(m_restCache.curveData.vertsPerCurve.begin(), m_restCache.curveData.vertsPerCurve.end());
    std::vector<size_t> vertexOffsets(m_restCache.curveData.vertexOffsets.begin(), m_restCache.curveData.vertexOffsets.end());
    std::vector<V3f> points(m_restCache.curveData.points.begin(), m_restCache.curveData.points.end());

    // Process curves in parallel
    const size_t numCurves = vertsPerCurve.size();
    const size_t numThreads = std::thread::hardware_concurrency();
    const size_t batchSize = calculateBatchSize(numCurves, numThreads);

    parallel_for(blocked_range<size_t>(0, numCurves, batchSize),
                 [&](const blocked_range<size_t> &range)
                 {
                     // Thread-local evaluator result
                     PrimitiveEvaluator::ResultPtr result = restEvaluator->createResult();
                     MeshPrimitiveEvaluator::Result *meshResult = static_cast<MeshPrimitiveEvaluator::Result *>(result.get());

                     for (size_t i = range.begin(); i != range.end(); ++i)
                     {
                         Detail::CurveBinding &binding = m_restCache.bindingCache.bindings[i];

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
                         binding.restFrame.tangent = Detail::primVar<V3f>(m_restCache.restTangents,
                                                                          &binding.baryCoords[0],
                                                                          binding.triangleIndex,
                                                                          meshResult->vertexIds());
                         binding.restFrame.orthonormalize();

                         // Pre-compute rest space offsets
                         m_restCache.curveData.computeRestSpaceOffsets(binding.restFrame, i);

                         binding.valid = true;
                     }
                 });

    // Store binding data as primitive variables
    V3fVectorDataPtr restPositionsData = new V3fVectorData;
    V3fVectorDataPtr restNormalsData = new V3fVectorData;
    V3fVectorDataPtr restTangentsData = new V3fVectorData;
    V3fVectorDataPtr restBitangentsData = new V3fVectorData;
    V3fVectorDataPtr localOffsetsData = new V3fVectorData;
    FloatVectorDataPtr falloffValuesData = new FloatVectorData;
    V3fVectorDataPtr rootPointsData = new V3fVectorData;

    std::vector<V3f> &restPositions = restPositionsData->writable();
    std::vector<V3f> &restNormals = restNormalsData->writable();
    std::vector<V3f> &restTangents = restTangentsData->writable();
    std::vector<V3f> &restBitangents = restBitangentsData->writable();
    std::vector<V3f> &localOffsets = localOffsetsData->writable();
    std::vector<float> &falloffValues = falloffValuesData->writable();
    std::vector<V3f> &rootPoints = rootPointsData->writable();

    // Pre-allocate arrays
    restPositions.resize(numCurves);
    restNormals.resize(numCurves);
    restTangents.resize(numCurves);
    restBitangents.resize(numCurves);

    // Copy data instead of direct assignment to handle different allocators
    localOffsets.assign(m_restCache.curveData.restSpaceOffsets.begin(), m_restCache.curveData.restSpaceOffsets.end());
    falloffValues.assign(m_restCache.curveData.falloffValues.begin(), m_restCache.curveData.falloffValues.end());

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
        const Detail::CurveBinding &binding = m_restCache.bindingCache.bindings[i];
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
    outputCurves->variables["localOffset"] = PrimitiveVariable(PrimitiveVariable::Vertex, localOffsetsData);
    outputCurves->variables["falloffValue"] = PrimitiveVariable(PrimitiveVariable::Vertex, falloffValuesData);
    outputCurves->variables["rootPoint"] = PrimitiveVariable(PrimitiveVariable::Uniform, rootPointsData);

    // Store optimization data
    outputCurves->variables["triangleIndex"] = PrimitiveVariable(PrimitiveVariable::Uniform, triangleIndicesData);
    outputCurves->variables["barycentricCoords"] = PrimitiveVariable(PrimitiveVariable::Uniform, barycentricCoordsData);
    outputCurves->variables["uvCoords"] = PrimitiveVariable(PrimitiveVariable::Uniform, uvCoordsData);

    m_restCache.bindingCache.valid = true;
}