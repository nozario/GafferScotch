#include "GafferScotch/RigidDeformCurves.h"
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

    // Helper to hash binding data
    void hashBindingData(const Primitive *primitive, MurmurHash &h)
    {
        if (!primitive)
            return;

        // Hash all binding attributes
        const char *bindingAttrs[] = {
            "restPosition", "restNormal", "restTangent", "restBitangent",
            "localOffset", "falloffValue", "rootPoint"};

        for (const char *attrName : bindingAttrs)
        {
            auto it = primitive->variables.find(attrName);
            if (it != primitive->variables.end())
            {
                it->second.data->hash(h);
            }
        }
    }

    // Helper to create a transformation matrix from position and frame vectors
    M44f createFrameMatrix(const V3f &position, const V3f &tangent, const V3f &bitangent, const V3f &normal)
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
                    meshPath = makeScenePath(pathData->readable());
                }
                else if (const StringVectorData *pathVectorData = runTimeCast<const StringVectorData>(it->second.data.get()))
                {
                    const std::vector<std::string> &paths = pathVectorData->readable();
                    if (!paths.empty())
                    {
                        meshPath = makeScenePath(paths[0]); // Use first path for now
                    }
                }
            }
        }
    }
    else
    {
        // Use path from plug
        meshPath = makeScenePath(bindPathPlug()->getValue());
    }

    // Get mesh objects
    ConstObjectPtr restMeshObj = restMeshPlug()->object(meshPath);
    ConstObjectPtr animatedMeshObj = animatedMeshPlug()->object(meshPath);

    const MeshPrimitive *restMesh = runTimeCast<const MeshPrimitive>(restMeshObj.get());
    const MeshPrimitive *animatedMesh = runTimeCast<const MeshPrimitive>(animatedMeshObj.get());

    if (restMesh && animatedMesh)
    {
        // Hash only the positions from meshes
        hashPositions(restMesh, h);
        hashPositions(animatedMesh, h);

        // Hash binding data from curves
        hashBindingData(curves, h);
    }
    else
    {
        // If we don't have valid primitives, hash the entire objects
        h.append(restMeshPlug()->objectHash(meshPath));
        h.append(animatedMeshPlug()->objectHash(meshPath));
        h.append(inPlug()->objectHash(path));
    }

    // Hash the control plugs
    useBindAttrPlug()->hash(h);
    bindPathPlug()->hash(h);
    bindAttrPlug()->hash(h);
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
    // We use the same hash as the processed object since the bound depends on the deformed positions
    ConstObjectPtr inputObject = inPlug()->object(path);
    const CurvesPrimitive *curves = runTimeCast<const CurvesPrimitive>(inputObject.get());
    
    if (!curves)
    {
        h = inputObject->hash();
        return;
    }
    
    // Get the target path based on mode
    ScenePath meshPath;
    if (useBindAttrPlug()->getValue())
    {
        // Use first path for hashing
        meshPath = makeScenePath(bindPathPlug()->getValue());
    }
    else
    {
        meshPath = makeScenePath(bindPathPlug()->getValue());
    }
    
    h.append(inPlug()->objectHash(path));
    h.append(restMeshPlug()->objectHash(meshPath));
    h.append(animatedMeshPlug()->objectHash(meshPath));
    useBindAttrPlug()->hash(h);
    bindPathPlug()->hash(h);
    bindAttrPlug()->hash(h);
}

Imath::Box3f RigidDeformCurves::computeProcessedObjectBound(const ScenePath &path, const Gaffer::Context *context) const
{
    // We'll let the base class compute the bound from the processed object
    // This is less efficient but ensures correctness
    return Deformer::computeProcessedObjectBound(path, context);
}

void RigidDeformCurves::computeDeformedFrame(
    const V3f &restPosition,
    const V3f &restNormal,
    const V3f &restTangent,
    const V3f &restBitangent,
    const MeshPrimitive *restMesh,
    const MeshPrimitive *animatedMesh,
    V3f &outPosition,
    V3f &outNormal,
    V3f &outTangent,
    V3f &outBitangent) const
{
    // Create evaluators for both meshes
    MeshPrimitivePtr triangulatedRestMesh = MeshAlgo::triangulate(restMesh);
    MeshPrimitivePtr triangulatedAnimatedMesh = MeshAlgo::triangulate(animatedMesh);

    MeshPrimitiveEvaluatorPtr restEvaluator = new MeshPrimitiveEvaluator(triangulatedRestMesh);
    MeshPrimitiveEvaluatorPtr animatedEvaluator = new MeshPrimitiveEvaluator(triangulatedAnimatedMesh);

    // Create results for both evaluators
    PrimitiveEvaluator::ResultPtr restResult = restEvaluator->createResult();
    PrimitiveEvaluator::ResultPtr animResult = animatedEvaluator->createResult();

    MeshPrimitiveEvaluator::Result *restMeshResult = static_cast<MeshPrimitiveEvaluator::Result *>(restResult.get());
    MeshPrimitiveEvaluator::Result *animMeshResult = static_cast<MeshPrimitiveEvaluator::Result *>(animResult.get());

    // Find closest point on rest mesh to validate stored position
    if (!restEvaluator->closestPoint(restPosition, restMeshResult))
    {
        // If point not found, keep original position and frame
        outPosition = restPosition;
        outNormal = restNormal;
        outTangent = restTangent;
        outBitangent = restBitangent;
        return;
    }

    // Try multiple methods to find corresponding point on animated mesh
    bool foundAnimPoint = false;

    // Method 1: Try UV sampling first
    if (animatedEvaluator->pointAtUV(restMeshResult->uv(), animMeshResult))
    {
        foundAnimPoint = true;
    }

    // Method 2: If UV sampling fails, try closest point to expected position
    if (!foundAnimPoint)
    {
        V3f expectedPos = restPosition + (animMeshResult->point() - restMeshResult->point());
        if (animatedEvaluator->closestPoint(expectedPos, animMeshResult))
        {
            foundAnimPoint = true;
        }
    }

    if (!foundAnimPoint)
    {
        // If all methods fail, keep original position and frame
        outPosition = restPosition;
        outNormal = restNormal;
        outTangent = restTangent;
        outBitangent = restBitangent;
        return;
    }

    // Get animated frame data
    outPosition = animMeshResult->point();
    outNormal = animMeshResult->normal();

    // Calculate tangents for animated mesh
    std::pair<PrimitiveVariable, PrimitiveVariable> animatedTangents;
    auto uvIt = triangulatedAnimatedMesh->variables.find("uv");
    if (uvIt == triangulatedAnimatedMesh->variables.end())
    {
        uvIt = triangulatedAnimatedMesh->variables.find("st");
    }
    if (uvIt == triangulatedAnimatedMesh->variables.end())
    {
        uvIt = triangulatedAnimatedMesh->variables.find("UV");
    }

    if (uvIt != triangulatedAnimatedMesh->variables.end())
    {
        animatedTangents = MeshAlgo::calculateTangents(triangulatedAnimatedMesh.get(), uvIt->first, true, "P");
    }
    else
    {
        animatedTangents = MeshAlgo::calculateTangentsFromTwoEdges(triangulatedAnimatedMesh.get(), "P", "N", true, false);
    }

    // Get tangent at the point using barycentric coordinates
    outTangent = Detail::primVar<V3f>(animatedTangents.first,
                                      &animMeshResult->barycentricCoordinates()[0],
                                      animMeshResult->triangleIndex(),
                                      animMeshResult->vertexIds());

    // Check if we need to flip the frame
    if (outNormal.dot(restNormal) < 0)
    {
        outNormal = -outNormal;
        outTangent = -outTangent;
    }

    // Ensure tangent alignment between frames
    float tangentDot = outTangent.dot(restTangent);
    if (tangentDot < 0.99999f)
    {
        // Project rest tangent onto the plane perpendicular to the animated normal
        V3f projectedRestTangent = restTangent - outNormal * restTangent.dot(outNormal);
        projectedRestTangent.normalize();

        // Use projected rest tangent
        outTangent = projectedRestTangent;
    }

    // Compute bitangent from normal and tangent
    outBitangent = (outNormal % outTangent).normalized();
}

void RigidDeformCurves::updateRestCache(
    const IECoreScene::CurvesPrimitive *curves,
    const IECore::MurmurHash &curvesHash) const
{
    // Check if cache is valid
    if (m_restCache.valid && m_restCache.curvesHash == curvesHash)
    {
        return;
    }

    // Initialize curve data
    m_restCache.curveData.initFromCurves(curves);

    // Update hashes
    m_restCache.curvesHash = curvesHash;
    m_restCache.valid = true;
}

void RigidDeformCurves::deformCurves(
    const CurvesPrimitive *curves,
    const MeshPrimitive *restMesh,
    const MeshPrimitive *animatedMesh,
    CurvesPrimitive *outputCurves) const
{
    // Get binding data
    const V3fVectorData *restPositions = curves->variableData<V3fVectorData>("restPosition", PrimitiveVariable::Uniform);
    const V3fVectorData *restNormals = curves->variableData<V3fVectorData>("restNormal", PrimitiveVariable::Uniform);
    const V3fVectorData *restTangents = curves->variableData<V3fVectorData>("restTangent", PrimitiveVariable::Uniform);
    const V3fVectorData *restBitangents = curves->variableData<V3fVectorData>("restBitangent", PrimitiveVariable::Uniform);
    const V3fVectorData *localOffsets = curves->variableData<V3fVectorData>("localOffset", PrimitiveVariable::Vertex);
    const FloatVectorData *falloffValues = curves->variableData<FloatVectorData>("falloffValue", PrimitiveVariable::Vertex);
    const V3fVectorData *rootPoints = curves->variableData<V3fVectorData>("rootPoint", PrimitiveVariable::Uniform);

    // Get optimization data
    const IntVectorData *triangleIndices = curves->variableData<IntVectorData>("triangleIndex", PrimitiveVariable::Uniform);
    const V3fVectorData *barycentricCoords = curves->variableData<V3fVectorData>("barycentricCoords", PrimitiveVariable::Uniform);
    const V2fVectorData *uvCoords = curves->variableData<V2fVectorData>("uvCoords", PrimitiveVariable::Uniform);

    if (!restPositions || !restNormals || !restTangents || !restBitangents || !localOffsets || !falloffValues || !rootPoints ||
        !triangleIndices || !barycentricCoords || !uvCoords)
    {
        return;
    }

    // Initialize curve data if not already done
    updateRestCache(curves, inPlug()->objectHash(ScenePath()));

    // Create evaluators for both meshes (shared between threads)
    MeshPrimitivePtr triangulatedRestMesh = MeshAlgo::triangulate(restMesh);
    MeshPrimitivePtr triangulatedAnimatedMesh = MeshAlgo::triangulate(animatedMesh);

    MeshPrimitiveEvaluatorPtr restEvaluator = new MeshPrimitiveEvaluator(triangulatedRestMesh);
    MeshPrimitiveEvaluatorPtr animatedEvaluator = new MeshPrimitiveEvaluator(triangulatedAnimatedMesh);

    // Calculate tangents for animated mesh (shared between threads)
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

        return MeshAlgo::calculateTangentsFromTwoEdges(mesh, "P", "N", true, false);
    };

    std::pair<PrimitiveVariable, PrimitiveVariable> animatedTangents = calculateMeshTangents(triangulatedAnimatedMesh.get());

    // Create output points array
    V3fVectorDataPtr outputPointsData = new V3fVectorData;
    std::vector<V3f> &outputPoints = outputPointsData->writable();
    outputPoints.resize(m_restCache.curveData.totalVerts);

    // Process curves in parallel
    const size_t numCurves = m_restCache.curveData.vertsPerCurve.size();
    const size_t numThreads = std::thread::hardware_concurrency();
    const size_t batchSize = calculateBatchSize(numCurves, numThreads);

    parallel_for(blocked_range<size_t>(0, numCurves, batchSize),
                 [&](const blocked_range<size_t> &range)
                 {
                     // Thread-local evaluator results
                     PrimitiveEvaluator::ResultPtr restResult = restEvaluator->createResult();
                     PrimitiveEvaluator::ResultPtr animResult = animatedEvaluator->createResult();

                     MeshPrimitiveEvaluator::Result *restMeshResult = static_cast<MeshPrimitiveEvaluator::Result *>(restResult.get());
                     MeshPrimitiveEvaluator::Result *animMeshResult = static_cast<MeshPrimitiveEvaluator::Result *>(animResult.get());

                     for (size_t i = range.begin(); i != range.end(); ++i)
                     {
                         const size_t vertOffset = m_restCache.curveData.vertexOffsets[i];
                         const int numVerts = m_restCache.curveData.vertsPerCurve[i];

                         // Get rest frame data
                         Detail::AlignedFrame restFrame;
                         restFrame.position = restPositions->readable()[i];
                         restFrame.normal = restNormals->readable()[i];
                         restFrame.tangent = restTangents->readable()[i];
                         restFrame.bitangent = restBitangents->readable()[i];

                         // Get triangle index and barycentric coordinates
                         const int triIndex = triangleIndices->readable()[i];
                         const V3f &baryCoords = barycentricCoords->readable()[i];
                         const V2f &uvs = uvCoords->readable()[i];

                         // Try UV-based lookup first
                         bool foundAnimPoint = animatedEvaluator->pointAtUV(uvs, animMeshResult);

                         // Fallback to closest point if UV lookup fails
                         if (!foundAnimPoint)
                         {
                             V3f expectedPos = restFrame.position + (animMeshResult->point() - restFrame.position);
                             foundAnimPoint = animatedEvaluator->closestPoint(expectedPos, animMeshResult);
                         }

                         if (!foundAnimPoint)
                         {
                             // If all methods fail, just translate the curve
                             V3f translation = animMeshResult->point() - restFrame.position;
                             for (int j = 0; j < numVerts; ++j)
                             {
                                 outputPoints[vertOffset + j] = m_restCache.curveData.points[vertOffset + j] + translation;
                             }
                             continue;
                         }

                         // Build animated frame
                         Detail::AlignedFrame animFrame;
                         animFrame.position = animMeshResult->point();
                         animFrame.normal = animMeshResult->normal();
                         animFrame.tangent = Detail::primVar<V3f>(animatedTangents.first,
                                                                  &baryCoords[0],
                                                                  triIndex,
                                                                  animMeshResult->vertexIds());
                         animFrame.orthonormalize();

                         // Check if we need to flip the frame
                         if (animFrame.normal.dot(restFrame.normal) < 0)
                         {
                             animFrame.normal = -animFrame.normal;
                             animFrame.tangent = -animFrame.tangent;
                             animFrame.bitangent = -animFrame.bitangent;
                         }

                         // Ensure tangent alignment between frames
                         float tangentDot = animFrame.tangent.dot(restFrame.tangent);
                         if (tangentDot < 0.99999f)
                         {
                             // Project rest tangent onto the plane perpendicular to the animated normal
                             V3f projectedRestTangent = restFrame.tangent - animFrame.normal * restFrame.tangent.dot(animFrame.normal);
                             projectedRestTangent.normalize();
                             animFrame.tangent = projectedRestTangent;
                             animFrame.bitangent = (animFrame.normal % animFrame.tangent).normalized();
                         }

                         // Transform each point in the curve
                         for (int j = 0; j < numVerts; ++j)
                         {
                             const size_t idx = vertOffset + j;
                             const V3f &localOffset = localOffsets->readable()[idx];
                             const float falloff = falloffValues->readable()[idx];

                             // Transform point using frame vectors directly
                             V3f deformedPoint = animFrame.position +
                                                 animFrame.tangent * localOffset.x +
                                                 animFrame.bitangent * localOffset.y +
                                                 animFrame.normal * localOffset.z;

                             // Simple translation for far points
                             V3f translation = animFrame.position - restFrame.position;
                             V3f translatedPoint = m_restCache.curveData.points[idx] + translation;

                             // Blend between deformed and translated positions
                             outputPoints[idx] = deformedPoint * falloff + translatedPoint * (1.0f - falloff);
                         }
                     }
                 });

    // Update output points
    outputCurves->variables["P"] = PrimitiveVariable(PrimitiveVariable::Vertex, outputPointsData);
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
                    meshPath = makeScenePath(pathData->readable());
                }
                else if (const StringVectorData *pathVectorData = runTimeCast<const StringVectorData>(it->second.data.get()))
                {
                    const std::vector<std::string> &paths = pathVectorData->readable();
                    if (!paths.empty())
                    {
                        meshPath = makeScenePath(paths[0]); // Use first path for now
                    }
                }
            }
        }
    }
    else
    {
        // Use path from plug
        meshPath = makeScenePath(bindPathPlug()->getValue());
    }

    // Get mesh objects
    ConstObjectPtr restMeshObj = restMeshPlug()->object(meshPath);
    ConstObjectPtr animatedMeshObj = animatedMeshPlug()->object(meshPath);

    const MeshPrimitive *restMesh = runTimeCast<const MeshPrimitive>(restMeshObj.get());
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

    // Deform curves using binding data
    deformCurves(curves, restMesh, animatedMesh, outputCurves.get());

    return outputCurves;
}