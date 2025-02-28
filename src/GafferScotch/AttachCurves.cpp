#include <math.h>
#include "GafferScotch/AttachCurves.h"
#include "GafferScotch/AttachCurvesDataStructures.h"
#include "GafferScotch/ScenePathUtil.h"

#include "IECore/NullObject.h"
#include "IECoreScene/CurvesPrimitive.h"
#include "IECoreScene/MeshPrimitive.h"
#include "IECoreScene/MeshPrimitiveEvaluator.h"
#include "IECoreScene/PrimitiveVariable.h"
#include "IECore/VectorTypedData.h"
#include "IECore/StringAlgo.h"
#include "Imath/ImathVec.h"
#include "Imath/ImathMatrix.h"
#include "IECoreScene/MeshAlgo.h"
#include <thread>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/cache_aligned_allocator.h>
#include <iostream>
#include <mutex>
#include <atomic>

using namespace Gaffer;
using namespace GafferScene;
using namespace GafferScotch;
using namespace IECore;
using namespace IECoreScene;
using namespace Imath;
using namespace tbb;
using namespace GafferScotch::Detail;

namespace
{
    std::mutex g_logMutex;
    std::atomic<int> g_concurrentOffsetWrites{0};
    std::atomic<int> g_cacheAccesses{0};

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

    // Helper to hash UV data
    void hashUVs(const Primitive *primitive, MurmurHash &h)
    {
        if (!primitive)
            return;

        // Try different UV attribute names
        const char *uvNames[] = {"uv", "st", "UV"};
        for (const char *uvName : uvNames)
        {
            auto it = primitive->variables.find(uvName);
            if (it != primitive->variables.end())
            {
                it->second.data->hash(h);
                return;
            }
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

IE_CORE_DEFINERUNTIMETYPED(GafferScotch::AttachCurves);

size_t GafferScotch::AttachCurves::g_firstPlugIndex = 0;

AttachCurves::AttachCurves(const std::string &name)
    : Deformer(name)
{
    storeIndexOfNextChild(g_firstPlugIndex);

    // Add source mesh inputs
    addChild(new ScenePlug("restMesh", Plug::In));
    addChild(new ScenePlug("animatedMesh", Plug::In));

    // Root point finding
    addChild(new StringPlug("rootAttributeName", Plug::In, ""));

    // Binding mode
    addChild(new BoolPlug("useBindRootAttribute", Plug::In, false));
    addChild(new StringPlug("rootPath", Plug::In, ""));                  // Used when useBindRootAttribute=false
    addChild(new StringPlug("bindRootAttribute", Plug::In, "bindPath")); // Used when useBindRootAttribute=true

    // Fast pass-throughs for things we don't modify
    outPlug()->attributesPlug()->setInput(inPlug()->attributesPlug());
    outPlug()->transformPlug()->setInput(inPlug()->transformPlug());
    outPlug()->boundPlug()->setInput(inPlug()->boundPlug());
}

AttachCurves::~AttachCurves()
{
}

ScenePlug *AttachCurves::restMeshPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

const ScenePlug *AttachCurves::restMeshPlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

ScenePlug *AttachCurves::animatedMeshPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex + 1);
}

const ScenePlug *AttachCurves::animatedMeshPlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex + 1);
}

StringPlug *AttachCurves::rootAttributeNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 2);
}

const StringPlug *AttachCurves::rootAttributeNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 2);
}

BoolPlug *AttachCurves::useBindRootAttributePlug()
{
    return getChild<BoolPlug>(g_firstPlugIndex + 3);
}

const BoolPlug *AttachCurves::useBindRootAttributePlug() const
{
    return getChild<BoolPlug>(g_firstPlugIndex + 3);
}

StringPlug *AttachCurves::rootPathPlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 4);
}

const StringPlug *AttachCurves::rootPathPlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 4);
}

StringPlug *AttachCurves::bindRootAttributePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 5);
}

const StringPlug *AttachCurves::bindRootAttributePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 5);
}

void AttachCurves::affects(const Gaffer::Plug *input, AffectedPlugsContainer &outputs) const
{
    Deformer::affects(input, outputs);

    if (
        input == restMeshPlug()->objectPlug() ||
        input == animatedMeshPlug()->objectPlug() ||
        input == rootAttributeNamePlug() ||
        input == rootPathPlug() ||
        input == bindRootAttributePlug() ||
        input == useBindRootAttributePlug())
    {
        outputs.push_back(outPlug()->objectPlug());
    }
}

bool AttachCurves::acceptsInput(const Gaffer::Plug *plug, const Gaffer::Plug *inputPlug) const
{
    if (!Deformer::acceptsInput(plug, inputPlug))
    {
        return false;
    }

    return true;
}

bool AttachCurves::affectsProcessedObject(const Gaffer::Plug *input) const
{
    return input == restMeshPlug()->objectPlug() ||
           input == animatedMeshPlug()->objectPlug() ||
           input == rootAttributeNamePlug() ||
           input == rootPathPlug() ||
           input == bindRootAttributePlug() ||
           input == useBindRootAttributePlug();
}

void AttachCurves::hashProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    // Get the target path for mesh inputs
    const std::string rootPathStr = rootPathPlug()->getValue();
    const ScenePath restPath = GafferScotch::makeScenePath(rootPathStr);

    // Get objects
    ConstObjectPtr restMeshObj = restMeshPlug()->object(restPath);
    ConstObjectPtr animatedMeshObj = animatedMeshPlug()->object(restPath);
    ConstObjectPtr inputObject = inPlug()->object(path);

    // Cast to primitives
    const MeshPrimitive *restMesh = runTimeCast<const MeshPrimitive>(restMeshObj.get());
    const MeshPrimitive *animatedMesh = runTimeCast<const MeshPrimitive>(animatedMeshObj.get());
    const CurvesPrimitive *curves = runTimeCast<const CurvesPrimitive>(inputObject.get());

    if (restMesh && animatedMesh && curves)
    {
        // Hash only the positions from all primitives
        hashPositions(restMesh, h);
        hashPositions(animatedMesh, h);
        hashPositions(curves, h);

        // Hash mesh topology
        hashTopology(restMesh, h);
        hashTopology(animatedMesh, h);

        // Hash curve topology
        const std::vector<int> &vertsPerCurve = curves->verticesPerCurve()->readable();
        h.append(vertsPerCurve.size());
        if (!vertsPerCurve.empty())
        {
            h.append(&vertsPerCurve[0], vertsPerCurve.size());
        }

        // Hash normals and UVs for both meshes
        hashNormals(restMesh, h);
        hashNormals(animatedMesh, h);
        hashUVs(restMesh, h);
        hashUVs(animatedMesh, h);
    }
    else
    {
        // If we don't have valid primitives, hash the entire objects
        h.append(restMeshPlug()->objectHash(restPath));
        h.append(animatedMeshPlug()->objectHash(restPath));
        h.append(inPlug()->objectHash(path));
    }

    // Hash the control plugs
    rootAttributeNamePlug()->hash(h);
    rootPathPlug()->hash(h);
    bindRootAttributePlug()->hash(h);
    useBindRootAttributePlug()->hash(h);
}

bool AttachCurves::affectsProcessedObjectBound(const Gaffer::Plug *input) const
{
    return input == restMeshPlug()->objectPlug() ||
           input == animatedMeshPlug()->objectPlug() ||
           input == rootPathPlug() ||
           input == bindRootAttributePlug() ||
           input == useBindRootAttributePlug();
}

void AttachCurves::hashProcessedObjectBound(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    // We use the same hash as the processed object since the bound depends on the deformed positions
    const std::string rootPathStr = rootPathPlug()->getValue();
    const ScenePath restPath = GafferScotch::makeScenePath(rootPathStr);

    h.append(inPlug()->objectHash(path));
    h.append(restMeshPlug()->objectHash(restPath));
    h.append(animatedMeshPlug()->objectHash(restPath));
    rootPathPlug()->hash(h);
    bindRootAttributePlug()->hash(h);
    useBindRootAttributePlug()->hash(h);
}

Imath::Box3f AttachCurves::computeProcessedObjectBound(const ScenePath &path, const Gaffer::Context *context) const
{
    // We'll let the base class compute the bound from the processed object
    // This is less efficient but ensures correctness
    return Deformer::computeProcessedObjectBound(path, context);
}

void AttachCurves::updateRestCache(const MeshPrimitive *restMesh,
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

    // Compute bindings
    computeBindings(restMesh, curves);

    // Update hashes
    m_restCache.restMeshHash = restMeshHash;
    m_restCache.curvesHash = curvesHash;
    m_restCache.valid = true;
}

void AttachCurves::computeBindings(const MeshPrimitive *restMesh, const CurvesPrimitive *curves) const
{
    {
        std::lock_guard<std::mutex> lock(g_logMutex);
        std::cerr << "Starting computeBindings with " << m_restCache.curveData.vertsPerCurve.size() << " curves" << std::endl;
    }

    // Create evaluator for rest mesh
    MeshPrimitivePtr triangulatedRestMesh = MeshAlgo::triangulate(restMesh);
    MeshPrimitiveEvaluatorPtr restEvaluator = new MeshPrimitiveEvaluator(triangulatedRestMesh);

    // Build spatial index for acceleration
    buildSpatialIndex(triangulatedRestMesh.get(), restEvaluator.get());

    // Process curves in parallel
    const size_t numCurves = m_restCache.curveData.vertsPerCurve.size();
    const size_t numThreads = std::thread::hardware_concurrency();
    const size_t batchSize = calculateBatchSize(numCurves, numThreads);

    parallel_for(blocked_range<size_t>(0, numCurves, batchSize),
                 [&](const blocked_range<size_t> &range)
                 {
                     // Thread-local evaluator result
                     PrimitiveEvaluator::ResultPtr result = restEvaluator->createResult();
                     MeshPrimitiveEvaluator::Result *meshResult = static_cast<MeshPrimitiveEvaluator::Result *>(result.get());

                     // Process curves in this range
                     for (size_t i = range.begin(); i != range.end(); ++i)
                     {
                         const CurveBinding &binding = m_restCache.bindingCache.bindings.get(i);
                         if (!binding.valid)
                         {
                             continue;
                         }

                         const size_t vertOffset = m_restCache.curveData.vertexOffsets[i];
                         const V3f &rootP = m_restCache.curveData.points[vertOffset];

                         // Find closest point
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
                                                                          &binding.baryCoords[0], binding.triangleIndex, meshResult->vertexIds());
                         binding.restFrame.orthonormalize();

                         // Pre-compute rest space offsets
                         m_restCache.curveData.computeRestSpaceOffsets(binding.restFrame, i);

                         binding.valid = true;
                     }
                 });

    m_restCache.bindingCache.valid = true;
}

void AttachCurves::buildSpatialIndex(const MeshPrimitive *mesh, MeshPrimitiveEvaluator *evaluator) const
{
    // The evaluator already builds its internal BVH, so we don't need additional work here
    // This is a hook for future optimizations if needed
}

void AttachCurves::applyDeformation(const MeshPrimitive *animatedMesh,
                                    const CurvesPrimitive *curves,
                                    std::vector<V3f> &outputPoints) const
{
    int currentCacheAccesses = ++g_cacheAccesses;
    {
        std::lock_guard<std::mutex> lock(g_logMutex);
        std::cerr << "Starting applyDeformation (cache access #" << currentCacheAccesses << ")" << std::endl;
    }

    if (!m_restCache.bindingCache.valid)
    {
        return;
    }

    // Create evaluator for animated mesh
    MeshPrimitivePtr triangulatedAnimatedMesh = MeshAlgo::triangulate(animatedMesh);
    MeshPrimitiveEvaluatorPtr animatedEvaluator = new MeshPrimitiveEvaluator(triangulatedAnimatedMesh);

    // Calculate tangents for animated mesh
    auto calculateMeshTangents = [](const MeshPrimitive *mesh)
    {
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

    std::pair<PrimitiveVariable, PrimitiveVariable> animatedTangents = calculateMeshTangents(triangulatedAnimatedMesh.get());

    // Process curves in parallel
    const size_t numCurves = m_restCache.curveData.vertsPerCurve.size();
    const size_t numThreads = std::thread::hardware_concurrency();
    const size_t batchSize = calculateBatchSize(numCurves, numThreads);

    parallel_for(blocked_range<size_t>(0, numCurves, batchSize),
                 [&](const blocked_range<size_t> &range)
                 {
                     // Thread-local evaluator result
                     PrimitiveEvaluator::ResultPtr result = animatedEvaluator->createResult();
                     MeshPrimitiveEvaluator::Result *animResult = static_cast<MeshPrimitiveEvaluator::Result *>(result.get());

                     // Process curves in this range
                     for (size_t i = range.begin(); i != range.end(); ++i)
                     {
                         const CurveBinding &binding = m_restCache.bindingCache.bindings[i];
                         if (!binding.valid)
                         {
                             continue;
                         }

                         const size_t vertOffset = m_restCache.curveData.vertexOffsets[i];
                         const int numVerts = m_restCache.curveData.vertsPerCurve[i];

                         // Try to find corresponding point using UV coordinates first
                         bool foundAnimPoint = animatedEvaluator->pointAtUV(binding.uvCoords, animResult);

                         // Fallback to closest point if UV lookup fails
                         if (!foundAnimPoint)
                         {
                             V3f expectedPos = binding.restFrame.position +
                                               (animResult->point() - binding.restFrame.position);
                             foundAnimPoint = animatedEvaluator->closestPoint(expectedPos, animResult);
                         }

                         if (!foundAnimPoint)
                         {
                             // If all methods fail, just translate the curve
                             V3f translation = animResult->point() - binding.restFrame.position;
                             for (int j = 0; j < numVerts; ++j)
                             {
                                 outputPoints[vertOffset + j] = m_restCache.curveData.points[vertOffset + j] + translation;
                             }
                             continue;
                         }

                         // Build animated frame
                         AlignedFrame animFrame;
                         animFrame.position = animResult->point();
                         animFrame.normal = animResult->normal();
                         animFrame.tangent = Detail::primVar<V3f>(animatedTangents.first,
                                                                  &binding.baryCoords[0], binding.triangleIndex, animResult->vertexIds());
                         animFrame.orthonormalize();

                         // Check if we need to flip the frame
                         if (animFrame.normal.dot(binding.restFrame.normal) < 0)
                         {
                             animFrame.normal = -animFrame.normal;
                             animFrame.tangent = -animFrame.tangent;
                             animFrame.bitangent = -animFrame.bitangent;
                         }

                         // Ensure tangent alignment between frames
                         float tangentDot = animFrame.tangent.dot(binding.restFrame.tangent);
                         if (tangentDot < 0.99999f)
                         {
                             V3f projectedRestTangent = binding.restFrame.tangent -
                                                        animFrame.normal * binding.restFrame.tangent.dot(animFrame.normal);
                             projectedRestTangent.normalize();
                             animFrame.tangent = projectedRestTangent;
                             animFrame.bitangent = (animFrame.normal % animFrame.tangent).normalized();
                         }

                         // Transform each point in the curve
                         for (int j = 0; j < numVerts; ++j)
                         {
                             const size_t pointIndex = vertOffset + j;
                             // Use thread-safe getter
                             const V3f &restSpaceOffset = m_restCache.curveData.getRestSpaceOffset(pointIndex);

                             // Transform to animated space
                             V3f animatedOffset =
                                 restSpaceOffset.x * animFrame.tangent +
                                 restSpaceOffset.y * animFrame.bitangent +
                                 restSpaceOffset.z * animFrame.normal;

                             float falloff = m_restCache.curveData.falloffValues[pointIndex];

                             // Simple translation for far points
                             V3f translation = animFrame.position - binding.restFrame.position;
                             V3f translatedPoint = m_restCache.curveData.points[pointIndex] + translation;

                             // Blend between deformed and translated positions
                             outputPoints[pointIndex] = animFrame.position + animatedOffset * falloff +
                                                        (translatedPoint - (animFrame.position + animatedOffset)) * (1.0f - falloff);
                         }
                     }
                 });
}

IECore::ConstObjectPtr AttachCurves::computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject) const
{
    // Early out if we don't have a valid input object
    const CurvesPrimitive *curves = runTimeCast<const CurvesPrimitive>(inputObject);
    if (!curves)
    {
        return inputObject;
    }

    // Get the target path for mesh inputs
    const std::string rootPathStr = rootPathPlug()->getValue();
    const ScenePath restPath = GafferScotch::makeScenePath(rootPathStr);

    // Get rest and animated meshes using root path
    ConstObjectPtr restMeshObj = restMeshPlug()->object(restPath);
    ConstObjectPtr animatedMeshObj = animatedMeshPlug()->object(restPath);

    const MeshPrimitive *restMesh = runTimeCast<const MeshPrimitive>(restMeshObj.get());
    const MeshPrimitive *animatedMesh = runTimeCast<const MeshPrimitive>(animatedMeshObj.get());

    if (!restMesh || !animatedMesh)
    {
        return inputObject;
    }

    if (restMesh->numFaces() != animatedMesh->numFaces() ||
        restMesh->verticesPerFace()->readable() != animatedMesh->verticesPerFace()->readable())
    {
        throw InvalidArgumentException("Rest and animated meshes must have matching topology");
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

    // Get output points for modification
    V3fVectorDataPtr outPoints = new V3fVectorData;
    outPoints->writable().resize(m_restCache.curveData.totalVerts);
    outputCurves->variables["P"] = PrimitiveVariable(PrimitiveVariable::Vertex, outPoints);

    // Apply deformation using cached bindings
    applyDeformation(animatedMesh, curves, outPoints->writable());

    return outputCurves;
}
