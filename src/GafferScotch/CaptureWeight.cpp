#include <math.h>
#include "GafferScotch/CaptureWeight.h"

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

using namespace Gaffer;
using namespace GafferScene;
using namespace GafferScotch;
using namespace IECore;
using namespace IECoreScene;
using namespace Imath;
using namespace tbb;

namespace
{
    GafferScene::ScenePlug::ScenePath makeScenePath(const std::string &p)
    {
        GafferScene::ScenePlug::ScenePath output;
        IECore::StringAlgo::tokenize<IECore::InternedString>(p, '/', std::back_inserter(output));
        return output;
    }

    // Structure to hold per-vertex results to avoid thread contention
    struct VertexResult
    {
        std::vector<int> indices;
        std::vector<float> weights;
        int numInfluences;

        VertexResult(int maxPoints) : indices(maxPoints, 0), weights(maxPoints, 0.0f), numInfluences(0) {}
    };

    // Helper function for weight calculation optimization
    inline float calculateWeight(float distSquared, float maxDistSquared)
    {
        float normalizedDist2 = distSquared / maxDistSquared;
        float weight = 1.0f - normalizedDist2;
        return weight * weight; // Square for smoother falloff
    }

    // Helper functions to get piece attribute values for specific types
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

    // Helper function to check if two points match based on piece attribute
    bool piecesMatch(const PrimitiveVariable *sourcePiece, size_t sourceIndex,
                     const PrimitiveVariable *targetPiece, size_t targetIndex)
    {
        if (!sourcePiece || !targetPiece)
        {
            return true; // No piece filtering
        }

        // Try integer attributes
        int sourceInt = 0, targetInt = 0;
        if (getPieceValueInt(*sourcePiece, sourceIndex, sourceInt) &&
            getPieceValueInt(*targetPiece, targetIndex, targetInt))
        {
            return sourceInt == targetInt;
        }

        // Try float attributes (with epsilon comparison)
        float sourceFloat = 0.0f, targetFloat = 0.0f;
        if (getPieceValueFloat(*sourcePiece, sourceIndex, sourceFloat) &&
            getPieceValueFloat(*targetPiece, targetIndex, targetFloat))
        {
            return std::abs(sourceFloat - targetFloat) < 1e-6f;
        }

        // Try string attributes
        std::string sourceStr, targetStr;
        if (getPieceValueString(*sourcePiece, sourceIndex, sourceStr) &&
            getPieceValueString(*targetPiece, targetIndex, targetStr))
        {
            return sourceStr == targetStr;
        }

        // Try vector attributes
        V3f sourceVec(0), targetVec(0);
        if (getPieceValueV3f(*sourcePiece, sourceIndex, sourceVec) &&
            getPieceValueV3f(*targetPiece, targetIndex, targetVec))
        {
            return (sourceVec - targetVec).length() < 1e-6f;
        }

        return false; // No matching types found
    }

    // Helper to get position data from a primitive
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

    // Helper to get hash for piece attribute
    void hashPieceAttribute(const Primitive *primitive, const std::string &attrName, MurmurHash &h)
    {
        if (!primitive || attrName.empty())
            return;

        auto it = primitive->variables.find(attrName);
        if (it == primitive->variables.end())
            return;

        // Hash the attribute data by appending to the provided hash
        it->second.data->hash(h);
    }
} // namespace

IE_CORE_DEFINERUNTIMETYPED(CaptureWeight);

size_t CaptureWeight::g_firstPlugIndex = 0;

CaptureWeight::CaptureWeight(const std::string &name)
    : AttributeProcessor(name)
{
    storeIndexOfNextChild(g_firstPlugIndex);

    // Add source points input
    addChild(new ScenePlug("source", Plug::In));

    // Add source path
    addChild(new StringPlug("sourcePath", Plug::In, ""));

    // Add parameters
    addChild(new FloatPlug("radius", Plug::In, 1.0f, 0.0f));
    addChild(new IntPlug("maxPoints", Plug::In, 4, 1));
    addChild(new IntPlug("minPoints", Plug::In, 1, 1));
    addChild(new StringPlug("pieceAttribute", Plug::In, ""));
}

ScenePlug *CaptureWeight::sourcePlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

const ScenePlug *CaptureWeight::sourcePlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

StringPlug *CaptureWeight::sourcePathPlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 1);
}

const StringPlug *CaptureWeight::sourcePathPlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 1);
}

FloatPlug *CaptureWeight::radiusPlug()
{
    return getChild<FloatPlug>(g_firstPlugIndex + 2);
}

const FloatPlug *CaptureWeight::radiusPlug() const
{
    return getChild<FloatPlug>(g_firstPlugIndex + 2);
}

IntPlug *CaptureWeight::maxPointsPlug()
{
    return getChild<IntPlug>(g_firstPlugIndex + 3);
}

const IntPlug *CaptureWeight::maxPointsPlug() const
{
    return getChild<IntPlug>(g_firstPlugIndex + 3);
}

IntPlug *CaptureWeight::minPointsPlug()
{
    return getChild<IntPlug>(g_firstPlugIndex + 4);
}

const IntPlug *CaptureWeight::minPointsPlug() const
{
    return getChild<IntPlug>(g_firstPlugIndex + 4);
}

StringPlug *CaptureWeight::pieceAttributePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 5);
}

const StringPlug *CaptureWeight::pieceAttributePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 5);
}

void CaptureWeight::affects(const Gaffer::Plug *input, AffectedPlugsContainer &outputs) const
{
    AttributeProcessor::affects(input, outputs);

    if (input == sourcePlug()->objectPlug() ||
        input == sourcePathPlug() ||
        input == radiusPlug() ||
        input == maxPointsPlug() ||
        input == minPointsPlug() ||
        input == pieceAttributePlug())
    {
        outputs.push_back(outPlug()->objectPlug());
    }
}

bool CaptureWeight::affectsProcessedAttributes(const Gaffer::Plug *input) const
{
    // We don't actually modify attributes, so return false
    return false;
}

void CaptureWeight::hashProcessedAttributes(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    // We don't modify attributes, so just pass through the input hash
    h = inPlug()->attributesPlug()->hash();
}

IECore::ConstCompoundObjectPtr CaptureWeight::computeProcessedAttributes(const ScenePath &path, const Gaffer::Context *context, const IECore::CompoundObject *inputAttributes) const
{
    // We don't modify attributes, so just return the input
    return inputAttributes;
}

void CaptureWeight::hashObject(const ScenePath &path, const Gaffer::Context *context, const GafferScene::ScenePlug *parent, IECore::MurmurHash &h) const
{
    FilteredSceneProcessor::hashObject(path, context, parent, h);
    
    // Only hash if the filter matches
    const PathMatcher &filter = filterPlug()->getValue();
    if (filter.match(path) & Filter::ExactMatch)
    {
        // Hash all inputs that affect the object
        h.append(inPlug()->objectHash(path));
        
        const std::string sourcePath = sourcePathPlug()->getValue();
        if (!sourcePath.empty())
        {
            ScenePlug::ScenePath sourceScenePath;
            ScenePlug::stringToPath(sourcePath, sourceScenePath);
            h.append(sourcePlug()->objectHash(sourceScenePath));
        }
        
        radiusPlug()->hash(h);
        maxPointsPlug()->hash(h);
        minPointsPlug()->hash(h);
        pieceAttributePlug()->hash(h);
    }
}

IECore::ConstObjectPtr CaptureWeight::computeObject(const ScenePath &path, const Gaffer::Context *context, const GafferScene::ScenePlug *parent) const
{
    // Only process if the filter matches
    const PathMatcher &filter = filterPlug()->getValue();
    if (!(filter.match(path) & Filter::ExactMatch))
    {
        return inPlug()->objectPlug()->getValue();
    }
    
    // Get input object
    ConstObjectPtr inputObject = inPlug()->object(path);
    const Primitive *inputPrimitive = runTimeCast<const Primitive>(inputObject.get());
    if (!inputPrimitive)
    {
        return inputObject;
    }
    
    // Get source object
    const std::string sourcePath = sourcePathPlug()->getValue();
    if (sourcePath.empty())
    {
        return inputObject;
    }
    
    ScenePlug::ScenePath sourceScenePath;
    ScenePlug::stringToPath(sourcePath, sourceScenePath);
    ConstObjectPtr sourceObject = sourcePlug()->object(sourceScenePath);
    const Primitive *sourcePrimitive = runTimeCast<const Primitive>(sourceObject.get());
    if (!sourcePrimitive)
    {
        return inputObject;
    }
    
    // Compute capture weights
    return computeCaptureWeights(inputPrimitive, sourcePrimitive);
}

IECore::PrimitivePtr CaptureWeight::computeCaptureWeights(const IECoreScene::Primitive *inputPrimitive, const IECoreScene::Primitive *sourcePrimitive) const
{
    // Early out if we don't have a valid input object
    if (!inputPrimitive)
    {
        return inputPrimitive->copy();
    }

    // Get the source path
    const ScenePath sourcePath = makeScenePath(sourcePathPlug()->getValue());

    // Get position data from source primitive
    const V3fVectorData *sourcePositions = getPositions(sourcePrimitive);
    if (!sourcePositions)
    {
        return inputPrimitive->copy();
    }

    // Get position data from target primitive
    const V3fVectorData *targetPositions = getPositions(inputPrimitive);
    if (!targetPositions)
    {
        return inputPrimitive->copy();
    }

    // Get piece attributes if specified
    const PrimitiveVariable *sourcePiece = nullptr;
    const std::string pieceAttr = pieceAttributePlug()->getValue();
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

    // Build KD tree for source points
    const std::vector<V3f> &sourcePoints = sourcePositions->readable();
    const std::vector<V3f> &targetPoints = targetPositions->readable();
    typedef V3fTree::Iterator VectorIterator;
    V3fTree tree(sourcePoints.begin(), sourcePoints.end());

    // Pre-allocate results vector for all vertices
    const size_t numVertices = targetPoints.size();
    std::vector<VertexResult> results;
    results.reserve(numVertices);
    for (size_t i = 0; i < numVertices; ++i)
    {
        results.emplace_back(maxPointsPlug()->getValue());
    }

    // Process vertices in parallel
    parallel_for(blocked_range<size_t>(0, numVertices),
                 [&](const blocked_range<size_t> &range)
                 {
                     // Thread-local storage for temporary data
                     std::vector<V3fTree::Neighbour> neighbours;
                     std::vector<std::pair<float, int>> validNeighbours;
                     std::vector<float> vertexWeights;
                     neighbours.reserve(maxPointsPlug()->getValue() * 4);
                     validNeighbours.reserve(maxPointsPlug()->getValue() * 4);
                     vertexWeights.reserve(maxPointsPlug()->getValue());

                     for (size_t i = range.begin(); i != range.end(); ++i)
                     {
                         const V3f &targetPos = targetPoints[i];

                         // Find nearest points using KDTree
                         neighbours.clear();
                         unsigned int found = tree.nearestNNeighbours(targetPos, maxPointsPlug()->getValue(), neighbours);

                         // Filter by piece attribute and radius
                         validNeighbours.clear();
                         validNeighbours.reserve(found);
                         std::vector<std::pair<float, int>> fallbackNeighbours;
                         fallbackNeighbours.reserve(found);

                         float maxDistSquared = radiusPlug()->getValue() * radiusPlug()->getValue();
                         for (size_t j = 0; j < found; ++j)
                         {
                             const int sourceIndex = neighbours[j].point - sourcePoints.begin();

                             // Store all points within radius as fallback
                             if (neighbours[j].distSquared <= maxDistSquared)
                             {
                                 fallbackNeighbours.emplace_back(neighbours[j].distSquared, sourceIndex);
                             }

                             // Check piece attribute match
                             if (!piecesMatch(sourcePiece, sourceIndex, targetPiece, i))
                             {
                                 continue;
                             }

                             // Check radius
                             if (neighbours[j].distSquared <= maxDistSquared)
                             {
                                 validNeighbours.emplace_back(neighbours[j].distSquared, sourceIndex);
                             }
                         }

                         // If we found too few points with matching pieces, try expanding search radius
                         if (validNeighbours.size() < static_cast<size_t>(minPointsPlug()->getValue()))
                         {
                             validNeighbours.clear();
                             neighbours.clear();

                             // Use a larger number of neighbors to ensure we find enough valid ones
                             unsigned int extraFound = tree.nearestNNeighbours(targetPos, maxPointsPlug()->getValue() * 4, neighbours);

                             for (size_t j = 0; j < extraFound; ++j)
                             {
                                 const int sourceIndex = neighbours[j].point - sourcePoints.begin();

                                 // Store all points as potential fallback
                                 fallbackNeighbours.emplace_back(neighbours[j].distSquared, sourceIndex);

                                 if (!piecesMatch(sourcePiece, sourceIndex, targetPiece, i))
                                 {
                                     continue;
                                 }

                                 validNeighbours.emplace_back(neighbours[j].distSquared, sourceIndex);
                                 maxDistSquared = std::max(maxDistSquared, neighbours[j].distSquared);

                                 if (validNeighbours.size() >= static_cast<size_t>(minPointsPlug()->getValue()))
                                 {
                                     break;
                                 }
                             }
                         }

                         // If we still don't have enough points, use fallback points (ignoring piece matching)
                         if (validNeighbours.size() < static_cast<size_t>(minPointsPlug()->getValue()))
                         {
                             // Sort fallback points by distance
                             std::sort(fallbackNeighbours.begin(), fallbackNeighbours.end());

                             // Take the closest points up to minPoints
                             size_t numNeeded = static_cast<size_t>(minPointsPlug()->getValue()) - validNeighbours.size();
                             size_t numAvailable = std::min(numNeeded, fallbackNeighbours.size());

                             for (size_t j = 0; j < numAvailable; ++j)
                             {
                                 validNeighbours.push_back(fallbackNeighbours[j]);
                                 maxDistSquared = std::max(maxDistSquared, fallbackNeighbours[j].first);
                             }
                         }

                         // Limit to maxPoints
                         if (validNeighbours.size() > static_cast<size_t>(maxPointsPlug()->getValue()))
                         {
                             validNeighbours.resize(maxPointsPlug()->getValue());
                         }

                         // Calculate weights
                         float totalWeight = 0.0f;
                         vertexWeights.clear();
                         vertexWeights.reserve(validNeighbours.size());

                         // Optimize weight calculations by avoiding unnecessary sqrt operations
                         for (const auto &neighbour : validNeighbours)
                         {
                             float weight = calculateWeight(neighbour.first, maxDistSquared);
                             totalWeight += weight;
                             vertexWeights.push_back(weight);
                         }

                         // Store results
                         VertexResult &result = results[i];
                         result.numInfluences = validNeighbours.size();

                         for (int j = 0; j < maxPointsPlug()->getValue(); ++j)
                         {
                             if (j < validNeighbours.size())
                             {
                                 result.indices[j] = validNeighbours[j].second;
                                 result.weights[j] = totalWeight > 0.0f ? vertexWeights[j] / totalWeight : 0.0f;
                             }
                             else
                             {
                                 result.indices[j] = 0;
                                 result.weights[j] = 0.0f;
                             }
                         }
                     }
                 });

    // Create output arrays
    IntVectorDataPtr captureInfluences = new IntVectorData;
    std::vector<IntVectorDataPtr> captureIndices;
    std::vector<FloatVectorDataPtr> captureWeights;

    captureInfluences->writable().reserve(numVertices);

    for (int i = 0; i < maxPointsPlug()->getValue(); ++i)
    {
        captureIndices.push_back(new IntVectorData);
        captureWeights.push_back(new FloatVectorData);

        captureIndices[i]->writable().reserve(numVertices);
        captureWeights[i]->writable().reserve(numVertices);
    }

    // Copy results to output arrays
    for (size_t i = 0; i < numVertices; ++i)
    {
        const VertexResult &result = results[i];
        captureInfluences->writable().push_back(result.numInfluences);

        for (int j = 0; j < maxPointsPlug()->getValue(); ++j)
        {
            captureIndices[j]->writable().push_back(result.indices[j]);
            captureWeights[j]->writable().push_back(result.weights[j]);
        }
    }

    // Create output primitive
    PrimitivePtr resultPrimitive = inputPrimitive->copy();

    // Add influence count
    resultPrimitive->variables["captureInfluences"] = PrimitiveVariable(PrimitiveVariable::Vertex, captureInfluences);

    // Add per-influence primitive variables
    for (int i = 0; i < maxPointsPlug()->getValue(); ++i)
    {
        std::string indexName = "captureIndex" + std::to_string(i + 1);
        std::string weightName = "captureWeight" + std::to_string(i + 1);

        resultPrimitive->variables[indexName] = PrimitiveVariable(PrimitiveVariable::Vertex, captureIndices[i]);
        resultPrimitive->variables[weightName] = PrimitiveVariable(PrimitiveVariable::Vertex, captureWeights[i]);
    }

    return resultPrimitive;
}
