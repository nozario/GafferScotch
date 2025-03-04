#include <math.h>
#include "GafferScotch/PointDeform.h"
#include "GafferScotch/ScenePathUtil.h"

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
    template <typename T>
    using AlignedVector = std::vector<T, tbb::cache_aligned_allocator<T>>;

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

        outDelta.x += (animatedPoint.x - staticPoint.x) * weight;
        outDelta.y += (animatedPoint.y - staticPoint.y) * weight;
        outDelta.z += (animatedPoint.z - staticPoint.z) * weight;
        outTotalWeight += weight;
    }

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

        const std::vector<V3f> &pos = positions->readable();
        h.append(pos.size());
        if (!pos.empty())
        {
            h.append(&pos[0], pos.size());
        }
    }
}

IE_CORE_DEFINERUNTIMETYPED(PointDeform);

size_t PointDeform::g_firstPlugIndex = 0;

PointDeform::PointDeform(const std::string &name)
    : Deformer(name)
{
    storeIndexOfNextChild(g_firstPlugIndex);

    addChild(new ScenePlug("staticDeformer"));
    addChild(new ScenePlug("animatedDeformer"));
    addChild(new StringPlug("deformerPath", Plug::In, ""));
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

    const ScenePath deformerPath = GafferScotch::makeScenePath(deformerPathPlug()->getValue());
    ConstObjectPtr inputObject = inPlug()->object(path);
    
    const Primitive *inputPrimitive = runTimeCast<const Primitive>(inputObject.get());
    if (!inputPrimitive)
    {
        h.append(inPlug()->objectHash(path));
        return;
    }
    
    ConstObjectPtr staticDeformerObject = staticDeformerPlug()->object(deformerPath);
    ConstObjectPtr animatedDeformerObject = animatedDeformerPlug()->object(deformerPath);

    const Primitive *staticDeformerPrimitive = runTimeCast<const Primitive>(staticDeformerObject.get());
    const Primitive *animatedDeformerPrimitive = runTimeCast<const Primitive>(animatedDeformerObject.get());

    if (inputPrimitive && staticDeformerPrimitive && animatedDeformerPrimitive)
    {
        hashPositions(staticDeformerPrimitive, h);
        hashPositions(animatedDeformerPrimitive, h);
        h.append(inPlug()->objectHash(path));
        cleanupAttributesPlug()->hash(h);
    }
    else
    {
        h.append(inPlug()->objectHash(path));
        h.append(staticDeformerPlug()->objectHash(deformerPath));
        h.append(animatedDeformerPlug()->objectHash(deformerPath));
        deformerPathPlug()->hash(h);
        cleanupAttributesPlug()->hash(h);
    }
}

void PointDeform::hashProcessedObjectBound(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    hashProcessedObject(path, context, h);
}

Imath::Box3f PointDeform::computeProcessedObjectBound(const ScenePath &path, const Gaffer::Context *context) const
{
    const Box3f inputBound = inPlug()->boundPlug()->getValue();
    if (inputBound.isEmpty())
    {
        return inputBound;
    }
    
    ConstObjectPtr inputObject = inPlug()->objectPlug()->getValue();
    const Primitive *inputPrimitive = runTimeCast<const Primitive>(inputObject.get());
    if (!inputPrimitive)
    {
        return inputBound;
    }
    
    const ScenePath deformerPath = GafferScotch::makeScenePath(deformerPathPlug()->getValue());
    
    ConstObjectPtr staticObj = staticDeformerPlug()->object(deformerPath);
    ConstObjectPtr animatedObj = animatedDeformerPlug()->object(deformerPath);
    
    const Primitive *staticDeformerPrimitive = runTimeCast<const Primitive>(staticObj.get());
    const Primitive *animatedDeformerPrimitive = runTimeCast<const Primitive>(animatedObj.get());
    
    if (!staticDeformerPrimitive || !animatedDeformerPrimitive)
    {
        return inputBound;
    }
    
    // Get the positions from both deformers
    auto staticPIt = staticDeformerPrimitive->variables.find("P");
    auto animatedPIt = animatedDeformerPrimitive->variables.find("P");
    
    if (staticPIt == staticDeformerPrimitive->variables.end() || 
        animatedPIt == animatedDeformerPrimitive->variables.end())
    {
        return inputBound;
    }
    
    const V3fVectorData *staticPositions = runTimeCast<const V3fVectorData>(staticPIt->second.data.get());
    const V3fVectorData *animatedPositions = runTimeCast<const V3fVectorData>(animatedPIt->second.data.get());
    
    if (!staticPositions || !animatedPositions)
    {
        return inputBound;
    }
    
    const std::vector<V3f> &staticPos = staticPositions->readable();
    const std::vector<V3f> &animatedPos = animatedPositions->readable();
    
    if (staticPos.size() != animatedPos.size() || staticPos.empty())
    {
        return inputBound;
    }
    
    V3f maxDisplacement(0, 0, 0);
    
    for (size_t i = 0; i < staticPos.size(); ++i)
    {
        V3f displacement = animatedPos[i] - staticPos[i];
        maxDisplacement.x = std::max(maxDisplacement.x, std::abs(displacement.x));
        maxDisplacement.y = std::max(maxDisplacement.y, std::abs(displacement.y));
        maxDisplacement.z = std::max(maxDisplacement.z, std::abs(displacement.z));
    }
    
    Box3f result = inputBound;
    result.min -= maxDisplacement;
    result.max += maxDisplacement;
    
    return result;
}

IECore::ConstObjectPtr PointDeform::computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject) const
{
    const Primitive *inputPrimitive = runTimeCast<const Primitive>(inputObject);
    if (!inputPrimitive)
        return inputObject;

    const ScenePath deformerPath = GafferScotch::makeScenePath(deformerPathPlug()->getValue());

    ConstObjectPtr staticDeformerObject = staticDeformerPlug()->object(deformerPath);
    const Primitive *staticDeformerPrimitive = runTimeCast<const Primitive>(staticDeformerObject.get());
    if (!staticDeformerPrimitive)
        return inputObject;

    ConstObjectPtr animatedDeformerObject = animatedDeformerPlug()->object(deformerPath);
    const Primitive *animatedDeformerPrimitive = runTimeCast<const Primitive>(animatedDeformerObject.get());
    if (!animatedDeformerPrimitive)
        return inputObject;

    PrimitivePtr result = inputPrimitive->copy();

    auto pIt = result->variables.find("P");
    if (pIt == result->variables.end())
        return result;

    V3fVectorDataPtr positionData = runTimeCast<V3fVectorData>(pIt->second.data);
    if (!positionData)
        return result;

    AlignedVector<V3f> positions(positionData->readable().begin(), positionData->readable().end());

    auto staticPIt = staticDeformerPrimitive->variables.find("P");
    if (staticPIt == staticDeformerPrimitive->variables.end())
        return result;

    const V3fVectorData *staticPositions = runTimeCast<const V3fVectorData>(staticPIt->second.data.get());
    if (!staticPositions)
        return result;

    auto animatedPIt = animatedDeformerPrimitive->variables.find("P");
    if (animatedPIt == animatedDeformerPrimitive->variables.end())
        return result;

    const V3fVectorData *animatedPositions = runTimeCast<const V3fVectorData>(animatedPIt->second.data.get());
    if (!animatedPositions)
        return result;

    auto captureInfluencesIt = inputPrimitive->variables.find("captureInfluences");
    if (captureInfluencesIt == inputPrimitive->variables.end())
        return result;

    const IntVectorData *captureInfluences = runTimeCast<const IntVectorData>(captureInfluencesIt->second.data.get());
    if (!captureInfluences)
        return result;

    const std::vector<int> &influences = captureInfluences->readable();
    const std::vector<V3f> &staticPos = staticPositions->readable();
    const std::vector<V3f> &animatedPos = animatedPositions->readable();

    int maxInfluences = 0;
    for (int numInf : influences)
        maxInfluences = std::max(maxInfluences, numInf);

    InfluenceData influenceData;
    if (!getInfluenceData(inputPrimitive, maxInfluences, influenceData))
        return result;

    const size_t numPoints = positions.size();
    parallel_for(blocked_range<size_t>(0, numPoints, 1024),
                 [&](const blocked_range<size_t> &range)
                 {
                     V3f delta;
                     float totalWeight;

                     for (size_t i = range.begin(); i != range.end(); ++i)
                     {
                         delta.setValue(0, 0, 0);
                         totalWeight = 0;

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

                         if (totalWeight > 0)
                         {
                             const float invWeight = 1.0f / totalWeight;
                             positions[i].x += delta.x * invWeight;
                             positions[i].y += delta.y * invWeight;
                             positions[i].z += delta.z * invWeight;
                         }
                     }
                 });

    positionData->writable().assign(positions.begin(), positions.end());

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