#include <math.h>
#include "GafferScotch/CaptureWeights.h"
#include "GafferScotch/ScenePathUtil.h"

#include "IECore/NullObject.h"
#include "IECore/KDTree.h"
#include "IECoreScene/PointsPrimitive.h"
#include "IECoreScene/MeshPrimitive.h"
#include "IECoreScene/CurvesPrimitive.h"
#include "IECoreScene/Primitive.h"
#include "IECore/VectorTypedData.h"
#include "Imath/ImathVec.h"

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/enumerable_thread_specific.h>
#include <limits>

using namespace Gaffer;
using namespace GafferScene;
using namespace GafferScotch;
using namespace IECore;
using namespace IECoreScene;
using namespace Imath;
using namespace tbb;

namespace
{
    using V3fTree = IECore::V3fTree;

    struct BatchResults
    {
        std::vector<int> allIndices;      // size = numVertices * maxPoints
        std::vector<float> allWeights;    // size = numVertices * maxPoints
        std::vector<int> influenceCounts; // size = numVertices

        BatchResults(size_t numVertices, int maxPoints)
            : allIndices(numVertices * maxPoints, 0), allWeights(numVertices * maxPoints, 0.0f), influenceCounts(numVertices, 0)
        {
        }

        int *getIndices(size_t vertexIndex, int maxPoints)
        {
            return &allIndices[vertexIndex * maxPoints];
        }

        float *getWeights(size_t vertexIndex, int maxPoints)
        {
            return &allWeights[vertexIndex * maxPoints];
        }
    };

    struct ThreadLocalStorage
    {
        std::vector<V3fTree::Neighbour> neighbours;
        std::vector<std::pair<float, int>> validNeighbours;
        std::vector<std::pair<float, int>> fallbackNeighbours;
        std::vector<float> vertexWeights;

        ThreadLocalStorage(int maxPoints)
        {
            neighbours.reserve(maxPoints * 4);
            validNeighbours.reserve(maxPoints * 4);
            fallbackNeighbours.reserve(maxPoints * 4);
            vertexWeights.reserve(maxPoints);
        }
    };

    inline float calculateWeight(float distSquared, float invMaxDistSquared)
    {
        float normalizedDist = sqrt(distSquared * invMaxDistSquared);
        // Smoother falloff using cubic function
        float t = 1.0f - normalizedDist;
        return t * t * (3.0f - 2.0f * t);
    }

    bool getPieceValueInt(const PrimitiveVariable &var, size_t index, int &value)
    {
        if (const IntData *constantData = runTimeCast<const IntData>(var.data.get()))
        {
            if (var.interpolation == PrimitiveVariable::Constant)
            {
                value = constantData->readable();
                return true;
            }
        }

        if (const IntVectorData *vectorData = runTimeCast<const IntVectorData>(var.data.get()))
        {
            const std::vector<int> &data = vectorData->readable();

            if (var.interpolation == PrimitiveVariable::Constant && !data.empty())
            {
                value = data[0];
                return true;
            }
            else if ((var.interpolation == PrimitiveVariable::Uniform ||
                      var.interpolation == PrimitiveVariable::Vertex ||
                      var.interpolation == PrimitiveVariable::Varying ||
                      var.interpolation == PrimitiveVariable::FaceVarying) &&
                     index < data.size())
            {
                value = data[index];
                return true;
            }
        }
        return false;
    }

    bool getPieceValueFloat(const PrimitiveVariable &var, size_t index, float &value)
    {
        if (const FloatData *constantData = runTimeCast<const FloatData>(var.data.get()))
        {
            if (var.interpolation == PrimitiveVariable::Constant)
            {
                value = constantData->readable();
                return true;
            }
        }

        if (const FloatVectorData *vectorData = runTimeCast<const FloatVectorData>(var.data.get()))
        {
            const std::vector<float> &data = vectorData->readable();

            if (var.interpolation == PrimitiveVariable::Constant && !data.empty())
            {
                value = data[0];
                return true;
            }
            else if ((var.interpolation == PrimitiveVariable::Uniform ||
                      var.interpolation == PrimitiveVariable::Vertex ||
                      var.interpolation == PrimitiveVariable::Varying ||
                      var.interpolation == PrimitiveVariable::FaceVarying) &&
                     index < data.size())
            {
                value = data[index];
                return true;
            }
        }
        return false;
    }

    bool getPieceValueString(const PrimitiveVariable &var, size_t index, std::string &value)
    {
        if (const StringData *constantData = runTimeCast<const StringData>(var.data.get()))
        {
            if (var.interpolation == PrimitiveVariable::Constant)
            {
                value = constantData->readable();
                return true;
            }
        }

        if (const StringVectorData *vectorData = runTimeCast<const StringVectorData>(var.data.get()))
        {
            const std::vector<std::string> &data = vectorData->readable();

            if (var.interpolation == PrimitiveVariable::Constant && !data.empty())
            {
                value = data[0];
                return true;
            }
            else if ((var.interpolation == PrimitiveVariable::Uniform ||
                      var.interpolation == PrimitiveVariable::Vertex ||
                      var.interpolation == PrimitiveVariable::Varying ||
                      var.interpolation == PrimitiveVariable::FaceVarying) &&
                     index < data.size())
            {
                value = data[index];
                return true;
            }
        }
        return false;
    }

    bool getPieceValueV3f(const PrimitiveVariable &var, size_t index, V3f &value)
    {
        if (const V3fData *constantData = runTimeCast<const V3fData>(var.data.get()))
        {
            if (var.interpolation == PrimitiveVariable::Constant)
            {
                value = constantData->readable();
                return true;
            }
        }

        if (const V3fVectorData *vectorData = runTimeCast<const V3fVectorData>(var.data.get()))
        {
            const std::vector<V3f> &data = vectorData->readable();

            if (var.interpolation == PrimitiveVariable::Constant && !data.empty())
            {
                value = data[0];
                return true;
            }
            else if ((var.interpolation == PrimitiveVariable::Uniform ||
                      var.interpolation == PrimitiveVariable::Vertex ||
                      var.interpolation == PrimitiveVariable::Varying ||
                      var.interpolation == PrimitiveVariable::FaceVarying) &&
                     index < data.size())
            {
                value = data[index];
                return true;
            }
        }
        return false;
    }

    bool piecesMatch(const PrimitiveVariable *sourcePiece, size_t sourceIndex,
                     const PrimitiveVariable *targetPiece, size_t targetIndex)
    {
        if (!sourcePiece || !targetPiece)
        {
            return true;
        }

        int sourceInt = 0, targetInt = 0;
        if (getPieceValueInt(*sourcePiece, sourceIndex, sourceInt) &&
            getPieceValueInt(*targetPiece, targetIndex, targetInt))
        {
            return sourceInt == targetInt;
        }

        float sourceFloat = 0.0f, targetFloat = 0.0f;
        if (getPieceValueFloat(*sourcePiece, sourceIndex, sourceFloat) &&
            getPieceValueFloat(*targetPiece, targetIndex, targetFloat))
        {
            return std::abs(sourceFloat - targetFloat) < 1e-6f;
        }

        std::string sourceStr, targetStr;
        if (getPieceValueString(*sourcePiece, sourceIndex, sourceStr) &&
            getPieceValueString(*targetPiece, targetIndex, targetStr))
        {
            return sourceStr == targetStr;
        }

        V3f sourceVec(0), targetVec(0);
        if (getPieceValueV3f(*sourcePiece, sourceIndex, sourceVec) &&
            getPieceValueV3f(*targetPiece, targetIndex, targetVec))
        {
            return (sourceVec - targetVec).length() < 1e-6f;
        }

        return false;
    }

    const V3fVectorData *getPositions(const Primitive *primitive)
    {
        if (!primitive)
        {
            return nullptr;
        }

        auto it = primitive->variables.find("P");
        if (it == primitive->variables.end())
        {
            return nullptr;
        }

        return runTimeCast<const V3fVectorData>(it->second.data.get());
    }

    void hashPrimitiveForCapture(const Primitive *primitive, const std::string &pieceAttr, MurmurHash &h)
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

        if (!pieceAttr.empty())
        {
            auto pieceIt = primitive->variables.find(pieceAttr);
            if (pieceIt != primitive->variables.end())
            {
                pieceIt->second.data->hash(h);
            }
        }
    }

    void computeCaptureWeightss(
        const std::vector<V3f> &sourcePoints,
        const std::vector<V3f> &targetPoints,
        float radius,
        int maxPoints,
        int minPoints,
        const PrimitiveVariable *sourcePiece,
        const PrimitiveVariable *targetPiece,
        BatchResults &results)
    {
        V3fTree tree(sourcePoints.begin(), sourcePoints.end(), 16);
        const float radiusSquared = radius * radius;
        const float invRadiusSquared = 1.0f / radiusSquared;

        // Get many more neighbors than needed to maximize chance of finding matching pieces
        const int candidateMultiplier = 10; // Get 10x more candidates than maxPoints
        const int maxCandidates = maxPoints * candidateMultiplier;

        enumerable_thread_specific<ThreadLocalStorage> threadStorage(maxPoints);

        parallel_for(blocked_range<size_t>(0, targetPoints.size(), 1024),
                     [&](const blocked_range<size_t> &range)
                     {
                         ThreadLocalStorage &tls = threadStorage.local();

                         for (size_t i = range.begin(); i != range.end(); ++i)
                         {
                             const V3f &targetPoint = targetPoints[i];

                             // Step 1: Find many nearest neighbors without initial radius constraint
                             tls.neighbours.clear();
                             tls.validNeighbours.clear();
                             tls.fallbackNeighbours.clear();
                             tls.vertexWeights.clear();

                             unsigned int found = tree.nearestNNeighbours(targetPoint, maxCandidates, tls.neighbours);

                             // Step 2: Filter neighbors
                             if (sourcePiece && targetPiece)
                             {
                                 // We have piece attributes - try to match them
                                 for (size_t j = 0; j < found; ++j)
                                 {
                                     const V3fTree::Neighbour &neighbour = tls.neighbours[j];
                                     const int sourceIndex = neighbour.point - sourcePoints.begin();

                                     // Consider only points within maximum radius for fallbacks
                                     if (neighbour.distSquared <= radiusSquared)
                                     {
                                         tls.fallbackNeighbours.emplace_back(neighbour.distSquared, sourceIndex);
                                     }

                                     // For valid neighbors, check piece matching
                                     if (piecesMatch(sourcePiece, sourceIndex, targetPiece, i))
                                     {
                                         tls.validNeighbours.emplace_back(neighbour.distSquared, sourceIndex);
                                     }
                                 }

                                 // If we don't have enough valid neighbors with matching pieces,
                                 // fall back to closest points regardless of piece but within radius
                                 if (tls.validNeighbours.size() < static_cast<size_t>(minPoints))
                                 {

                                     // Cap to maxPoints
                                     if (tls.fallbackNeighbours.size() > static_cast<size_t>(maxPoints))
                                     {
                                         tls.fallbackNeighbours.resize(maxPoints);
                                     }

                                     // Use fallbacks instead
                                     std::swap(tls.validNeighbours, tls.fallbackNeighbours);
                                 }
                                 else
                                 {
                                     if (tls.validNeighbours.size() > static_cast<size_t>(maxPoints))
                                     {
                                         tls.validNeighbours.resize(maxPoints);
                                     }
                                 }
                             }
                             else
                             {
                                 // No piece attributes - just use the nearest points within radius
                                 for (size_t j = 0; j < found; ++j)
                                 {
                                     const V3fTree::Neighbour &neighbour = tls.neighbours[j];
                                     if (neighbour.distSquared <= radiusSquared)
                                     {
                                         const int sourceIndex = neighbour.point - sourcePoints.begin();
                                         tls.validNeighbours.emplace_back(neighbour.distSquared, sourceIndex);
                                     }
                                 }
                                 if (tls.validNeighbours.size() > static_cast<size_t>(maxPoints))
                                 {
                                     tls.validNeighbours.resize(maxPoints);
                                 }
                             }

                             // Calculate weights based on distance
                             float totalWeight = 0.0f;

                             for (const auto &neighbour : tls.validNeighbours)
                             {
                                 float weight = calculateWeight(neighbour.first, invRadiusSquared);
                                 tls.vertexWeights.push_back(weight);
                                 totalWeight += weight;
                             }

                             // Store results
                             results.influenceCounts[i] = tls.validNeighbours.size();

                             // Normalize weights
                             const float invTotalWeight = totalWeight > 0.0f ? 1.0f / totalWeight : 0.0f;

                             int *indices = results.getIndices(i, maxPoints);
                             float *weights = results.getWeights(i, maxPoints);

                             for (int j = 0; j < maxPoints; ++j)
                             {
                                 if (j < tls.validNeighbours.size())
                                 {
                                     indices[j] = tls.validNeighbours[j].second;
                                     weights[j] = tls.vertexWeights[j] * invTotalWeight;
                                 }
                                 else
                                 {
                                     indices[j] = 0;
                                     weights[j] = 0.0f;
                                 }
                             }
                         }
                     });
    }
}

IE_CORE_DEFINERUNTIMETYPED(CaptureWeights);

size_t CaptureWeights::g_firstPlugIndex = 0;

CaptureWeights::CaptureWeights(const std::string &name)
    : ObjectProcessor(name)
{
    storeIndexOfNextChild(g_firstPlugIndex);

    addChild(new ScenePlug("staticDeformer", Plug::In));

    addChild(new StringPlug("deformerPath", Plug::In, ""));

    addChild(new FloatPlug("radius", Plug::In, 1.0f, 0.0f));
    addChild(new IntPlug("maxPoints", Plug::In, 4, 1));
    addChild(new IntPlug("minPoints", Plug::In, 1, 1));
    addChild(new StringPlug("pieceAttribute", Plug::In, ""));
}

ScenePlug *CaptureWeights::staticDeformerPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

const ScenePlug *CaptureWeights::staticDeformerPlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

StringPlug *CaptureWeights::deformerPathPlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 1);
}

const StringPlug *CaptureWeights::deformerPathPlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 1);
}

FloatPlug *CaptureWeights::radiusPlug()
{
    return getChild<FloatPlug>(g_firstPlugIndex + 2);
}

const FloatPlug *CaptureWeights::radiusPlug() const
{
    return getChild<FloatPlug>(g_firstPlugIndex + 2);
}

IntPlug *CaptureWeights::maxPointsPlug()
{
    return getChild<IntPlug>(g_firstPlugIndex + 3);
}

const IntPlug *CaptureWeights::maxPointsPlug() const
{
    return getChild<IntPlug>(g_firstPlugIndex + 3);
}

IntPlug *CaptureWeights::minPointsPlug()
{
    return getChild<IntPlug>(g_firstPlugIndex + 4);
}

const IntPlug *CaptureWeights::minPointsPlug() const
{
    return getChild<IntPlug>(g_firstPlugIndex + 4);
}

StringPlug *CaptureWeights::pieceAttributePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 5);
}

const StringPlug *CaptureWeights::pieceAttributePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 5);
}

void CaptureWeights::affects(const Gaffer::Plug *input, AffectedPlugsContainer &outputs) const
{
    ObjectProcessor::affects(input, outputs);

    if (input == staticDeformerPlug()->objectPlug() ||
        input == deformerPathPlug() ||
        input == radiusPlug() ||
        input == maxPointsPlug() ||
        input == minPointsPlug() ||
        input == pieceAttributePlug())
    {
        outputs.push_back(outPlug()->objectPlug());
    }
}

bool CaptureWeights::affectsProcessedObject(const Gaffer::Plug *input) const
{
    return input == staticDeformerPlug()->objectPlug() ||
           input == deformerPathPlug() ||
           input == radiusPlug() ||
           input == maxPointsPlug() ||
           input == minPointsPlug() ||
           input == pieceAttributePlug();
}

void CaptureWeights::hashPositions(const IECoreScene::Primitive *primitive, IECore::MurmurHash &h) const
{
    if (!primitive)
        return;

    auto it = primitive->variables.find("P");
    if (it == primitive->variables.end())
        return;

    const IECore::V3fVectorData *positions = IECore::runTimeCast<const IECore::V3fVectorData>(it->second.data.get());
    if (!positions)
        return;

    const std::vector<Imath::V3f> &pos = positions->readable();
    h.append(pos.size());
    if (!pos.empty())
    {
        h.append(&pos[0], pos.size());
    }
}

void CaptureWeights::hashPieceAttribute(const IECoreScene::Primitive *primitive, const std::string &attrName, IECore::MurmurHash &h) const
{
    if (!primitive || attrName.empty())
        return;

    auto it = primitive->variables.find(attrName);
    if (it == primitive->variables.end())
        return;

    it->second.data->hash(h);
}

void CaptureWeights::hashProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    const std::string deformerPathStr = deformerPathPlug()->getValue();
    ScenePath deformerPath = GafferScotch::makeScenePath(deformerPathStr);

    ConstObjectPtr sourceObj = staticDeformerPlug()->object(deformerPath);
    ConstObjectPtr inputObj = inPlug()->object(path);

    const IECoreScene::Primitive *sourcePrimitive = IECore::runTimeCast<const IECoreScene::Primitive>(sourceObj.get());
    const IECoreScene::Primitive *inputPrimitive = IECore::runTimeCast<const IECoreScene::Primitive>(inputObj.get());

    if (!sourcePrimitive || !inputPrimitive)
    {
        h = inputObj->hash();
        return;
    }

    const std::string pieceAttr = pieceAttributePlug()->getValue();

    hashPrimitiveForCapture(sourcePrimitive, pieceAttr, h);
    hashPrimitiveForCapture(inputPrimitive, pieceAttr, h);

    radiusPlug()->hash(h);
    maxPointsPlug()->hash(h);
    minPointsPlug()->hash(h);
    pieceAttributePlug()->hash(h);
}

IECore::ConstObjectPtr CaptureWeights::computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject) const
{
    const Primitive *inputPrimitive = runTimeCast<const Primitive>(inputObject);
    if (!inputPrimitive)
    {
        return inputObject;
    }

    const ScenePath deformerPath = GafferScotch::makeScenePath(deformerPathPlug()->getValue());

    ConstObjectPtr sourceObject = staticDeformerPlug()->object(deformerPath);
    const Primitive *sourcePrimitive = runTimeCast<const Primitive>(sourceObject.get());
    if (!sourcePrimitive)
    {
        return inputObject;
    }

    const float radius = radiusPlug()->getValue();
    const int maxPoints = maxPointsPlug()->getValue();
    const int minPoints = minPointsPlug()->getValue();
    const std::string pieceAttr = pieceAttributePlug()->getValue();

    const V3fVectorData *sourcePositions = getPositions(sourcePrimitive);
    if (!sourcePositions)
    {
        return inputObject;
    }

    const V3fVectorData *targetPositions = getPositions(inputPrimitive);
    if (!targetPositions)
    {
        return inputObject;
    }

    const PrimitiveVariable *sourcePiece = nullptr;
    if (!pieceAttr.empty())
    {
        auto it = sourcePrimitive->variables.find(pieceAttr);
        if (it != sourcePrimitive->variables.end())
        {
            sourcePiece = &it->second;
        }
    }

    const PrimitiveVariable *targetPiece = nullptr;
    if (!pieceAttr.empty())
    {
        auto it = inputPrimitive->variables.find(pieceAttr);
        if (it != inputPrimitive->variables.end())
        {
            targetPiece = &it->second;
        }
    }

    const std::vector<V3f> &sourcePoints = sourcePositions->readable();
    const std::vector<V3f> &targetPoints = targetPositions->readable();

    BatchResults results(targetPoints.size(), maxPoints);

    computeCaptureWeightss(sourcePoints, targetPoints, radius, maxPoints, minPoints,
                           sourcePiece, targetPiece, results);

    PrimitivePtr resultPrimitive = inputPrimitive->copy();

    IntVectorDataPtr captureInfluences = new IntVectorData(results.influenceCounts);
    std::vector<IntVectorDataPtr> captureIndices;
    std::vector<FloatVectorDataPtr> captureWeights;

    for (int i = 0; i < maxPoints; ++i)
    {
        captureIndices.push_back(new IntVectorData);
        captureWeights.push_back(new FloatVectorData);

        captureIndices[i]->writable().resize(targetPoints.size());
        captureWeights[i]->writable().resize(targetPoints.size());
    }

    for (size_t i = 0; i < targetPoints.size(); ++i)
    {
        for (int j = 0; j < maxPoints; ++j)
        {
            captureIndices[j]->writable()[i] = results.allIndices[i * maxPoints + j];
            captureWeights[j]->writable()[i] = results.allWeights[i * maxPoints + j];
        }
    }

    resultPrimitive->variables["captureInfluences"] = PrimitiveVariable(PrimitiveVariable::Vertex, captureInfluences);

    for (int i = 0; i < maxPoints; ++i)
    {
        std::string indexName = "captureIndex" + std::to_string(i + 1);
        std::string weightName = "captureWeight" + std::to_string(i + 1);

        resultPrimitive->variables[indexName] = PrimitiveVariable(PrimitiveVariable::Vertex, captureIndices[i]);
        resultPrimitive->variables[weightName] = PrimitiveVariable(PrimitiveVariable::Vertex, captureWeights[i]);
    }

    return resultPrimitive;
}
