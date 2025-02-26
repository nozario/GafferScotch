#include <math.h>
#include "GafferScotch/PointDeform.h"

#include "IECore/NullObject.h"
#include "IECoreScene/PointsPrimitive.h"
#include "IECoreScene/MeshPrimitive.h"
#include "IECoreScene/CurvesPrimitive.h"
#include "IECoreScene/Primitive.h"
#include "IECore/VectorTypedData.h"
#include "IECore/MatrixAlgo.h"
#include "IECore/TransformationMatrixData.h"
#include "Imath/ImathVec.h"
#include "Imath/ImathMatrix.h"

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
    // Aligned storage for better SIMD performance
    template <typename T>
    using AlignedVector = std::vector<T, tbb::cache_aligned_allocator<T>>;

    // Structure to hold cached influence data optimized for SIMD
    struct InfluenceData
    {
        struct Entry
        {
            const int *indices;
            const float *weights;
            size_t count;
        };
        std::vector<Entry> influences;

        void reserve(size_t size)
        {
            influences.reserve(size);
        }

        void add(const int *idx, const float *wgt, size_t cnt)
        {
            influences.push_back({idx, wgt, cnt});
        }
    };

    GafferScene::ScenePlug::ScenePath makeScenePath(const std::string &p)
    {
        GafferScene::ScenePlug::ScenePath output;
        IECore::StringAlgo::tokenize<IECore::InternedString>(p, '/', std::back_inserter(output));
        return output;
    }

    // Helper to get influence data with optimized memory layout
    bool getInfluenceData(const Primitive *primitive, int maxInfluences, InfluenceData &data)
    {
        data.influences.clear();
        data.influences.reserve(maxInfluences);

        for (int i = 0; i < maxInfluences; ++i)
        {
            std::string indexName = "captureIndex" + std::to_string(i + 1);
            std::string weightName = "captureWeight" + std::to_string(i + 1);

            auto indexIt = primitive->variables.find(indexName);
            auto weightIt = primitive->variables.find(weightName);

            if (indexIt == primitive->variables.end() || weightIt == primitive->variables.end())
                continue;

            const IntVectorData *indices = runTimeCast<const IntVectorData>(indexIt->second.data.get());
            const FloatVectorData *weights = runTimeCast<const FloatVectorData>(weightIt->second.data.get());

            if (!indices || !weights)
                continue;

            data.add(&indices->readable()[0], &weights->readable()[0], indices->readable().size());
        }

        return !data.influences.empty();
    }

    // Optimized deformation calculation using SIMD-friendly data layout
    inline void computeDeformation(
        const V3f &currentPos,
        const V3f *staticPos,
        const V3f *animatedPos,
        const InfluenceData::Entry &influence,
        size_t vertexIndex,
        V3f &outDelta,
        float &outTotalWeight)
    {
        const float weight = influence.weights[vertexIndex];
        if (weight <= 0.0f)
            return;

        const int sourceIndex = influence.indices[vertexIndex];
        if (sourceIndex < 0)
            return;

        const V3f &staticPoint = staticPos[sourceIndex];
        const V3f &animatedPoint = animatedPos[sourceIndex];

        // Compute deformation vector
        outDelta.x += (animatedPoint.x - staticPoint.x) * weight;
        outDelta.y += (animatedPoint.y - staticPoint.y) * weight;
        outDelta.z += (animatedPoint.z - staticPoint.z) * weight;
        outTotalWeight += weight;
    }

    // Simplified hashing for primitives - only hash what matters
    void hashPrimitiveForDeformation(const Primitive *primitive, MurmurHash &h)
    {
        if (!primitive)
            return;

        // For deformation, we only care about positions
        auto pIt = primitive->variables.find("P");
        if (pIt == primitive->variables.end())
            return;

        const V3fVectorData *positions = runTimeCast<const V3fVectorData>(pIt->second.data.get());
        if (!positions)
            return;

        // Hash the position data efficiently
        const std::vector<V3f> &pos = positions->readable();
        h.append(pos.size());
        if (!pos.empty())
        {
            h.append(&pos[0], pos.size());
        }
    }

    // Helper function to hash only the positions from a primitive
    void hashPositions(const IECoreScene::Primitive *primitive, IECore::MurmurHash &h)
    {
        if (!primitive)
            return;

        auto pIt = primitive->variables.find("P");
        if (pIt == primitive->variables.end())
            return;

        const V3fVectorData *positions = runTimeCast<const V3fVectorData>(pIt->second.data.get());
        if (!positions)
            return;

        // Hash the position data efficiently
        const std::vector<V3f> &pos = positions->readable();
        h.append(pos.size());
        if (!pos.empty())
        {
            h.append(&pos[0], pos.size());
        }
    }

    // Helper function to hash influence data
    void hashInfluences(const IECoreScene::Primitive *primitive, IECore::MurmurHash &h)
    {
        if (!primitive)
            return;

        // Hash capture influences count
        auto influencesIt = primitive->variables.find("captureInfluences");
        if (influencesIt == primitive->variables.end())
            return;

        const IntVectorData *influences = runTimeCast<const IntVectorData>(influencesIt->second.data.get());
        if (!influences)
            return;

        // Find max influences to know how many to hash
        int maxInfluences = 0;
        for (int numInf : influences->readable())
            maxInfluences = std::max(maxInfluences, numInf);

        h.append(maxInfluences);
        h.append(influences->readable().size());
        if (!influences->readable().empty())
            h.append(&influences->readable()[0], influences->readable().size());

        // Hash influence data
        for (int i = 1; i <= maxInfluences; ++i)
        {
            std::string indexName = "captureIndex" + std::to_string(i);
            std::string weightName = "captureWeight" + std::to_string(i);

            auto indexIt = primitive->variables.find(indexName);
            auto weightIt = primitive->variables.find(weightName);

            if (indexIt != primitive->variables.end() && weightIt != primitive->variables.end())
            {
                if (const IntVectorData *indices = runTimeCast<const IntVectorData>(indexIt->second.data.get()))
                {
                    h.append(indices->readable().size());
                    if (!indices->readable().empty())
                        h.append(&indices->readable()[0], indices->readable().size());
                }
                if (const FloatVectorData *weights = runTimeCast<const FloatVectorData>(weightIt->second.data.get()))
                {
                    h.append(weights->readable().size());
                    if (!weights->readable().empty())
                        h.append(&weights->readable()[0], weights->readable().size());
                }
            }
        }
    }
}

IE_CORE_DEFINERUNTIMETYPED(PointDeform);

size_t PointDeform::g_firstPlugIndex = 0;

PointDeform::PointDeform(const std::string &name)
    : Deformer(name)
{
    storeIndexOfNextChild(g_firstPlugIndex);

    // Add static deformer input
    addChild(new ScenePlug("staticDeformer"));

    // Add animated deformer input
    addChild(new ScenePlug("animatedDeformer"));

    // Add deformer path
    addChild(new StringPlug("deformerPath", Plug::In, ""));

    // Add cleanup attributes parameter
    addChild(new BoolPlug("cleanupAttributes", Plug::In, true));
}

ScenePlug *PointDeform::staticDeformerPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

const ScenePlug *PointDeform::staticDeformerPlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

ScenePlug *PointDeform::animatedDeformerPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex + 1);
}

const ScenePlug *PointDeform::animatedDeformerPlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex + 1);
}

StringPlug *PointDeform::deformerPathPlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 2);
}

const StringPlug *PointDeform::deformerPathPlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 2);
}

BoolPlug *PointDeform::cleanupAttributesPlug()
{
    return getChild<BoolPlug>(g_firstPlugIndex + 3);
}

const BoolPlug *PointDeform::cleanupAttributesPlug() const
{
    return getChild<BoolPlug>(g_firstPlugIndex + 3);
}

void PointDeform::affects(const Plug *input, AffectedPlugsContainer &outputs) const
{
    Deformer::affects(input, outputs);

    if (input == staticDeformerPlug()->objectPlug() ||
        input == animatedDeformerPlug()->objectPlug() ||
        input == deformerPathPlug() ||
        input == cleanupAttributesPlug())
    {
        outputs.push_back(outPlug()->objectPlug());
    }
}

bool PointDeform::affectsProcessedObjectBound(const Plug *input) const
{
    return input == staticDeformerPlug()->objectPlug() ||
           input == animatedDeformerPlug()->objectPlug() ||
           input == deformerPathPlug();
}

bool PointDeform::affectsProcessedObject(const Plug *input) const
{
    return input == staticDeformerPlug()->objectPlug() ||
           input == animatedDeformerPlug()->objectPlug() ||
           input == deformerPathPlug() ||
           input == cleanupAttributesPlug();
}

void PointDeform::hashProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    Deformer::hashProcessedObject(path, context, h);

    // Get source path and objects
    const ScenePath deformerPath = makeScenePath(deformerPathPlug()->getValue());
    ConstObjectPtr inputObject = inPlug()->object(path);
    ConstObjectPtr staticDeformerObject = staticDeformerPlug()->object(deformerPath);
    ConstObjectPtr animatedDeformerObject = animatedDeformerPlug()->object(deformerPath);

    // Cast to primitives
    const Primitive *inputPrimitive = runTimeCast<const Primitive>(inputObject.get());
    const Primitive *staticDeformerPrimitive = runTimeCast<const Primitive>(staticDeformerObject.get());
    const Primitive *animatedDeformerPrimitive = runTimeCast<const Primitive>(animatedDeformerObject.get());

    if (inputPrimitive && staticDeformerPrimitive && animatedDeformerPrimitive)
    {
        // Hash only the positions from deformers
        hashPositions(staticDeformerPrimitive, h);
        hashPositions(animatedDeformerPrimitive, h);

        // Hash influences and weights from input
        h.append(inPlug()->objectHash(path));

        // Hash the cleanup parameter as it affects the output
        cleanupAttributesPlug()->hash(h);
    }
    else
    {
        // If we don't have valid primitives, hash the entire objects
        h.append(inPlug()->objectHash(path));
        h.append(staticDeformerPlug()->objectHash(deformerPath));
        h.append(animatedDeformerPlug()->objectHash(deformerPath));
        deformerPathPlug()->hash(h);
        cleanupAttributesPlug()->hash(h);
    }
}

void PointDeform::hashProcessedObjectBound(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    // For bounds, we need the same dependencies as the object itself
    hashProcessedObject(path, context, h);
}

Imath::Box3f PointDeform::computeProcessedObjectBound(const ScenePath &path, const Gaffer::Context *context) const
{
    // Get the input object bound
    const Box3f inputBound = inPlug()->boundPlug()->getValue();
    
    // Early out if the bound is empty or if we're not deforming
    if (inputBound.isEmpty())
    {
        return inputBound;
    }
    
    // Try to be more efficient - compute bound directly from inputs
    // rather than computing the full deformed object
    ConstObjectPtr inputObject = inPlug()->objectPlug()->getValue();
    const Primitive *inputPrimitive = runTimeCast<const Primitive>(inputObject.get());
    if (!inputPrimitive)
    {
        return inputBound;
    }
    
    // Get the deformer path
    const ScenePath deformerPath = makeScenePath(deformerPathPlug()->getValue());
    
    // Check if we have valid deformer objects
    ConstObjectPtr staticObj = staticDeformerPlug()->object(deformerPath);
    ConstObjectPtr animatedObj = animatedDeformerPlug()->object(deformerPath);
    
    if (!runTimeCast<const Primitive>(staticObj.get()) || 
        !runTimeCast<const Primitive>(animatedObj.get()))
    {
        return inputBound;
    }
    
    // We have valid deformer objects - return a slightly expanded bound to account
    // for deformation. This is more efficient than recomputing the entire object.
    // The 5% expansion is a conservative estimate - adjust as needed.
    Box3f result = inputBound;
    result.min -= (result.size() * 0.05);
    result.max += (result.size() * 0.05);
    return result;
}

IECore::ConstObjectPtr PointDeform::computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject) const
{
    // Early out if we don't have a valid input object
    const Primitive *inputPrimitive = runTimeCast<const Primitive>(inputObject);
    if (!inputPrimitive)
        return inputObject;

    // Get the deformer path
    const ScenePath deformerPath = makeScenePath(deformerPathPlug()->getValue());

    // Get static deformer
    ConstObjectPtr staticDeformerObject = staticDeformerPlug()->object(deformerPath);
    const Primitive *staticDeformerPrimitive = runTimeCast<const Primitive>(staticDeformerObject.get());
    if (!staticDeformerPrimitive)
        return inputObject;

    // Get animated deformer
    ConstObjectPtr animatedDeformerObject = animatedDeformerPlug()->object(deformerPath);
    const Primitive *animatedDeformerPrimitive = runTimeCast<const Primitive>(animatedDeformerObject.get());
    if (!animatedDeformerPrimitive)
        return inputObject;

    // Create output primitive
    PrimitivePtr result = inputPrimitive->copy();

    // Get position data with aligned storage
    auto pIt = result->variables.find("P");
    if (pIt == result->variables.end())
        return result;

    V3fVectorDataPtr positionData = runTimeCast<V3fVectorData>(pIt->second.data);
    if (!positionData)
        return result;

    AlignedVector<V3f> positions(positionData->readable().begin(), positionData->readable().end());

    // Get static deformer positions
    auto staticPIt = staticDeformerPrimitive->variables.find("P");
    if (staticPIt == staticDeformerPrimitive->variables.end())
        return result;

    const V3fVectorData *staticPositions = runTimeCast<const V3fVectorData>(staticPIt->second.data.get());
    if (!staticPositions)
        return result;

    // Get animated deformer positions
    auto animatedPIt = animatedDeformerPrimitive->variables.find("P");
    if (animatedPIt == animatedDeformerPrimitive->variables.end())
        return result;

    const V3fVectorData *animatedPositions = runTimeCast<const V3fVectorData>(animatedPIt->second.data.get());
    if (!animatedPositions)
        return result;

    // Get capture influences
    auto captureInfluencesIt = inputPrimitive->variables.find("captureInfluences");
    if (captureInfluencesIt == inputPrimitive->variables.end())
        return result;

    const IntVectorData *captureInfluences = runTimeCast<const IntVectorData>(captureInfluencesIt->second.data.get());
    if (!captureInfluences)
        return result;

    const std::vector<int> &influences = captureInfluences->readable();
    const std::vector<V3f> &staticPos = staticPositions->readable();
    const std::vector<V3f> &animatedPos = animatedPositions->readable();

    // Cache influence data with optimized layout
    int maxInfluences = 0;
    for (int numInf : influences)
        maxInfluences = std::max(maxInfluences, numInf);

    InfluenceData influenceData;
    if (!getInfluenceData(inputPrimitive, maxInfluences, influenceData))
        return result;

    // Process points in parallel with cache-aligned data
    const size_t numPoints = positions.size();
    parallel_for(blocked_range<size_t>(0, numPoints, 1024),
                 [&](const blocked_range<size_t> &range)
                 {
                     // Thread-local storage aligned for SIMD
                     V3f delta;
                     float totalWeight;

                     for (size_t i = range.begin(); i != range.end(); ++i)
                     {
                         delta.setValue(0, 0, 0);
                         totalWeight = 0;

                         // Process each influence using SIMD-friendly computations
                         for (const auto &influence : influenceData.influences)
                         {
                             computeDeformation(
                                 positions[i],
                                 &staticPos[0],
                                 &animatedPos[0],
                                 influence,
                                 i,
                                 delta,
                                 totalWeight);
                         }

                         // Apply weighted deformation
                         if (totalWeight > 0)
                         {
                             const float invWeight = 1.0f / totalWeight;
                             positions[i].x += delta.x * invWeight;
                             positions[i].y += delta.y * invWeight;
                             positions[i].z += delta.z * invWeight;
                         }
                     }
                 });

    // Update result positions
    positionData->writable().assign(positions.begin(), positions.end());

    // Cleanup capture attributes if requested
    if (cleanupAttributesPlug()->getValue())
    {
        result->variables.erase("captureInfluences");
        for (int i = 1; i <= maxInfluences; ++i)
        {
            result->variables.erase("captureIndex" + std::to_string(i));
            result->variables.erase("captureWeight" + std::to_string(i));
        }
    }

    return result;
}