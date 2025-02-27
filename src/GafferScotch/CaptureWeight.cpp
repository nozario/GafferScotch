#include <math.h>
#include "GafferScotch/CaptureWeight.h"
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

using namespace Gaffer;
using namespace GafferScene;
using namespace GafferScotch;
using namespace IECore;
using namespace IECoreScene;
using namespace Imath;
using namespace tbb;

namespace
{
    // Define our own KDTree type for V3f points
    typedef KDTree<std::vector<V3f>::const_iterator> V3fKDTree;

    // Structure to hold batch results for all vertices
    struct BatchResults
    {
        std::vector<int> allIndices;       // size = numVertices * maxPoints
        std::vector<float> allWeights;     // size = numVertices * maxPoints
        std::vector<int> influenceCounts;  // size = numVertices

        BatchResults(size_t numVertices, int maxPoints)
            : allIndices(numVertices * maxPoints, 0)
            , allWeights(numVertices * maxPoints, 0.0f)
            , influenceCounts(numVertices, 0)
        {
        }

        int* getIndices(size_t vertexIndex, int maxPoints)
        {
            return &allIndices[vertexIndex * maxPoints];
        }

        float* getWeights(size_t vertexIndex, int maxPoints)
        {
            return &allWeights[vertexIndex * maxPoints];
        }
    };

    // Thread-local storage for temporary data
    struct ThreadLocalStorage
    {
        std::vector<V3fKDTree::Neighbour> neighbours;
        std::vector<std::pair<float, int>> validNeighbours;
        std::vector<std::pair<float, int>> fallbackNeighbours;
        std::vector<float> vertexWeights;

        ThreadLocalStorage(int maxPoints)
        {
            // Pre-allocate with sufficient capacity to avoid reallocations
            neighbours.reserve(maxPoints * 4);
            validNeighbours.reserve(maxPoints * 4);
            fallbackNeighbours.reserve(maxPoints * 4);
            vertexWeights.reserve(maxPoints);
        }
    };

    // Optimized weight calculation function
    inline float calculateWeight(float distSquared, float invMaxDistSquared)
    {
        float normalizedDist2 = distSquared * invMaxDistSquared; // Multiply instead of divide
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

    // Simplified hashing for primitives - only hash what matters for capture weights
    void hashPrimitiveForCapture(const Primitive *primitive, const std::string &pieceAttr, MurmurHash &h)
    {
        if (!primitive)
            return;

        // For capture weights, we only care about positions
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

        // Hash piece attribute if specified
        if (!pieceAttr.empty())
        {
            auto pieceIt = primitive->variables.find(pieceAttr);
            if (pieceIt != primitive->variables.end())
            {
                pieceIt->second.data->hash(h);
            }
        }
    }

    // Main function to compute capture weights using IECore's KDTree
    void computeCaptureWeights(
        const std::vector<V3f>& sourcePoints,
        const std::vector<V3f>& targetPoints,
        float radius,
        int maxPoints,
        int minPoints,
        const PrimitiveVariable *sourcePiece,
        const PrimitiveVariable *targetPiece,
        BatchResults& results)
    {
        // Build KDTree for source points
        // Use our own V3fKDTree typedef for KDTree<std::vector<V3f>::const_iterator>
        V3fKDTree tree(sourcePoints.begin(), sourcePoints.end(), 8); // Use 8 as maxLeafSize (tune through benchmarking)

        // Pre-compute squared radius and its inverse for optimization
        const float radius2 = radius * radius;
        const float invRadius2 = 1.0f / radius2;

        // Create thread-local storage
        enumerable_thread_specific<ThreadLocalStorage> threadStorage(maxPoints);

        // Process vertices in parallel
        const size_t numVertices = targetPoints.size();
        parallel_for(blocked_range<size_t>(0, numVertices, 1024), // Use 1024 as grain size
            [&](const blocked_range<size_t>& range)
            {
                // Get thread-local storage
                ThreadLocalStorage& tls = threadStorage.local();
                
                for (size_t i = range.begin(); i != range.end(); ++i)
                {
                    const V3f& targetPos = targetPoints[i];
                    
                    // Clear temporary vectors but maintain capacity
                    tls.neighbours.clear();
                    tls.validNeighbours.clear();
                    tls.fallbackNeighbours.clear();
                    tls.vertexWeights.clear();
                    
                    // Find nearest points using KDTree
                    // This returns a vector of Neighbour objects with point (iterator) and distSquared
                    unsigned int found = tree.nearestNNeighbours(targetPos, maxPoints * 2, tls.neighbours);
                    
                    // Filter by radius and piece attribute, collect valid neighbors
                    float maxDistSquared = radius2;
                    for (size_t j = 0; j < found; ++j)
                    {
                        const V3fKDTree::Neighbour& neighbour = tls.neighbours[j];
                        
                        // Skip if outside radius
                        if (neighbour.distSquared > radius2)
                            continue;
                        
                        // Calculate source index
                        const int sourceIndex = neighbour.point - sourcePoints.begin();
                        
                        // Store all points within radius as fallback
                        tls.fallbackNeighbours.emplace_back(neighbour.distSquared, sourceIndex);
                        
                        // Check piece attribute match
                        if (!piecesMatch(sourcePiece, sourceIndex, targetPiece, i))
                            continue;
                        
                        // Store valid neighbor
                        tls.validNeighbours.emplace_back(neighbour.distSquared, sourceIndex);
                    }
                    
                    // If we found too few points with matching pieces, try expanding search radius
                    if (tls.validNeighbours.size() < static_cast<size_t>(minPoints))
                    {
                        tls.neighbours.clear();
                        
                        // Use a larger number of neighbors
                        unsigned int extraFound = tree.nearestNNeighbours(targetPos, maxPoints * 4, tls.neighbours);
                        
                        for (size_t j = 0; j < extraFound; ++j)
                        {
                            const V3fKDTree::Neighbour& neighbour = tls.neighbours[j];
                            const int sourceIndex = neighbour.point - sourcePoints.begin();
                            
                            // Store all points as potential fallback
                            tls.fallbackNeighbours.emplace_back(neighbour.distSquared, sourceIndex);
                            
                            if (!piecesMatch(sourcePiece, sourceIndex, targetPiece, i))
                                continue;
                            
                            tls.validNeighbours.emplace_back(neighbour.distSquared, sourceIndex);
                            maxDistSquared = std::max(maxDistSquared, neighbour.distSquared);
                            
                            if (tls.validNeighbours.size() >= static_cast<size_t>(minPoints))
                                break;
                        }
                    }
                    
                    // If we still don't have enough points, use fallback points (ignoring piece matching)
                    if (tls.validNeighbours.size() < static_cast<size_t>(minPoints))
                    {
                        // Sort fallback points by distance
                        std::sort(tls.fallbackNeighbours.begin(), tls.fallbackNeighbours.end());
                        
                        // Take the closest points up to minPoints
                        size_t numNeeded = static_cast<size_t>(minPoints) - tls.validNeighbours.size();
                        size_t numAvailable = std::min(numNeeded, tls.fallbackNeighbours.size());
                        
                        for (size_t j = 0; j < numAvailable; ++j)
                        {
                            tls.validNeighbours.push_back(tls.fallbackNeighbours[j]);
                            maxDistSquared = std::max(maxDistSquared, tls.fallbackNeighbours[j].first);
                        }
                    }
                    
                    // Sort by distance
                    std::sort(tls.validNeighbours.begin(), tls.validNeighbours.end());
                    
                    // Limit to maxPoints
                    if (tls.validNeighbours.size() > static_cast<size_t>(maxPoints))
                    {
                        tls.validNeighbours.resize(maxPoints);
                    }
                    
                    // Calculate weights
                    float totalWeight = 0.0f;
                    const float invMaxDistSquared = 1.0f / maxDistSquared;
                    
                    for (const auto& neighbour : tls.validNeighbours)
                    {
                        float weight = calculateWeight(neighbour.first, invMaxDistSquared);
                        totalWeight += weight;
                        tls.vertexWeights.push_back(weight);
                    }
                    
                    // Store results
                    results.influenceCounts[i] = tls.validNeighbours.size();
                    
                    // Calculate inverse total weight once
                    const float invTotalWeight = totalWeight > 0.0f ? 1.0f / totalWeight : 0.0f;
                    
                    // Get pointers to result arrays for this vertex
                    int* indices = results.getIndices(i, maxPoints);
                    float* weights = results.getWeights(i, maxPoints);
                    
                    // Store normalized weights and indices
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
} // namespace

IE_CORE_DEFINERUNTIMETYPED(CaptureWeight);

size_t CaptureWeight::g_firstPlugIndex = 0;

CaptureWeight::CaptureWeight(const std::string &name)
    : ObjectProcessor(name)
{
    storeIndexOfNextChild(g_firstPlugIndex);

    // Add static deformer input
    addChild(new ScenePlug("staticDeformer", Plug::In));

    // Add deformer path
    addChild(new StringPlug("deformerPath", Plug::In, ""));

    // Add parameters
    addChild(new FloatPlug("radius", Plug::In, 1.0f, 0.0f));
    addChild(new IntPlug("maxPoints", Plug::In, 4, 1));
    addChild(new IntPlug("minPoints", Plug::In, 1, 1));
    addChild(new StringPlug("pieceAttribute", Plug::In, ""));
}

ScenePlug *CaptureWeight::staticDeformerPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

const ScenePlug *CaptureWeight::staticDeformerPlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

StringPlug *CaptureWeight::deformerPathPlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 1);
}

const StringPlug *CaptureWeight::deformerPathPlug() const
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

bool CaptureWeight::affectsProcessedObject(const Gaffer::Plug *input) const
{
    return input == staticDeformerPlug()->objectPlug() ||
           input == deformerPathPlug() ||
           input == radiusPlug() ||
           input == maxPointsPlug() ||
           input == minPointsPlug() ||
           input == pieceAttributePlug();
}

void CaptureWeight::hashPositions(const IECoreScene::Primitive *primitive, IECore::MurmurHash &h) const
{
    if (!primitive)
        return;

    auto it = primitive->variables.find("P");
    if (it == primitive->variables.end())
        return;

    const IECore::V3fVectorData *positions = IECore::runTimeCast<const IECore::V3fVectorData>(it->second.data.get());
    if (!positions)
        return;

    // Hash only the number of vertices and position data
    const std::vector<Imath::V3f> &pos = positions->readable();
    h.append(pos.size());
    if (!pos.empty())
    {
        h.append(&pos[0], pos.size());
    }
}

void CaptureWeight::hashPieceAttribute(const IECoreScene::Primitive *primitive, const std::string &attrName, IECore::MurmurHash &h) const
{
    if (!primitive || attrName.empty())
        return;

    auto it = primitive->variables.find(attrName);
    if (it == primitive->variables.end())
        return;

    // Hash the attribute data by appending to the provided hash
    it->second.data->hash(h);
}

void CaptureWeight::hashProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    // Get deformer path
    const std::string deformerPathStr = deformerPathPlug()->getValue();
    ScenePath deformerPath = GafferScotch::makeScenePath(deformerPathStr);

    // Get objects
    ConstObjectPtr sourceObj = staticDeformerPlug()->object(deformerPath);
    ConstObjectPtr inputObj = inPlug()->object(path);
    
    // Cast to primitives for more efficient hashing
    const IECoreScene::Primitive *sourcePrimitive = IECore::runTimeCast<const IECoreScene::Primitive>(sourceObj.get());
    const IECoreScene::Primitive *inputPrimitive = IECore::runTimeCast<const IECoreScene::Primitive>(inputObj.get());
    
    if (!sourcePrimitive || !inputPrimitive) {
        // If not valid primitives, just pass through
        h = inputObj->hash();
        return;
    }
    
    // Get piece attribute
    const std::string pieceAttr = pieceAttributePlug()->getValue();
    
    // Hash only the positions and piece attributes from both primitives
    hashPrimitiveForCapture(sourcePrimitive, pieceAttr, h);
    hashPrimitiveForCapture(inputPrimitive, pieceAttr, h);
    
    // Hash parameters that affect the output
    radiusPlug()->hash(h);
    maxPointsPlug()->hash(h);
    minPointsPlug()->hash(h);
    pieceAttributePlug()->hash(h);
}

IECore::ConstObjectPtr CaptureWeight::computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject) const
{
    // Early out if we don't have a valid input object
    const Primitive *inputPrimitive = runTimeCast<const Primitive>(inputObject);
    if (!inputPrimitive)
    {
        return inputObject;
    }

    // Get the deformer path
    const ScenePath deformerPath = GafferScotch::makeScenePath(deformerPathPlug()->getValue());

    // Get source points
    ConstObjectPtr sourceObject = staticDeformerPlug()->object(deformerPath);
    const Primitive *sourcePrimitive = runTimeCast<const Primitive>(sourceObject.get());
    if (!sourcePrimitive)
    {
        return inputObject;
    }

    // Cache parameter values to avoid repeated plug evaluations
    const float radius = radiusPlug()->getValue();
    const int maxPoints = maxPointsPlug()->getValue();
    const int minPoints = minPointsPlug()->getValue();
    const std::string pieceAttr = pieceAttributePlug()->getValue();

    // Get position data from source primitive
    const V3fVectorData *sourcePositions = getPositions(sourcePrimitive);
    if (!sourcePositions)
    {
        return inputObject;
    }

    // Get position data from target primitive
    const V3fVectorData *targetPositions = getPositions(inputPrimitive);
    if (!targetPositions)
    {
        return inputObject;
    }

    // Get piece attributes if specified
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

    // Get the source and target points
    const std::vector<V3f> &sourcePoints = sourcePositions->readable();
    const std::vector<V3f> &targetPoints = targetPositions->readable();

    // Create batch results structure
    BatchResults results(targetPoints.size(), maxPoints);

    // Compute capture weights using optimized implementation
    computeCaptureWeights(sourcePoints, targetPoints, radius, maxPoints, minPoints, 
                          sourcePiece, targetPiece, results);

    // Create output primitive
    PrimitivePtr resultPrimitive = inputPrimitive->copy();

    // Create output arrays
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

    // Copy results to output arrays
    for (size_t i = 0; i < targetPoints.size(); ++i)
    {
        for (int j = 0; j < maxPoints; ++j)
        {
            captureIndices[j]->writable()[i] = results.allIndices[i * maxPoints + j];
            captureWeights[j]->writable()[i] = results.allWeights[i * maxPoints + j];
        }
    }

    // Add influence count
    resultPrimitive->variables["captureInfluences"] = PrimitiveVariable(PrimitiveVariable::Vertex, captureInfluences);

    // Add per-influence primitive variables
    for (int i = 0; i < maxPoints; ++i)
    {
        std::string indexName = "captureIndex" + std::to_string(i + 1);
        std::string weightName = "captureWeight" + std::to_string(i + 1);

        resultPrimitive->variables[indexName] = PrimitiveVariable(PrimitiveVariable::Vertex, captureIndices[i]);
        resultPrimitive->variables[weightName] = PrimitiveVariable(PrimitiveVariable::Vertex, captureWeights[i]);
    }

    return resultPrimitive;
}
