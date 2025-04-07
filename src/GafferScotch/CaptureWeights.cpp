#include <math.h>
#include "GafferScotch/CaptureWeights.h"
#include "GafferScotch/ScenePathUtil.h"
#include "GafferScotch/nanoflann.hpp"

#include "IECoreScene/PointsPrimitive.h"
#include "IECoreScene/MeshPrimitive.h"
#include "IECoreScene/CurvesPrimitive.h"
#include "IECoreScene/Primitive.h"

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
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
    // Forward declare helper functions
    bool getPieceValueInt(const PrimitiveVariable &var, size_t index, int &value);
    bool getPieceValueFloat(const PrimitiveVariable &var, size_t index, float &value);
    bool getPieceValueString(const PrimitiveVariable &var, size_t index, std::string &value);
    bool getPieceValueV3f(const PrimitiveVariable &var, size_t index, V3f &value);
    bool piecesMatch(const PrimitiveVariable *sourcePiece, size_t sourceIndex,
                     const PrimitiveVariable *targetPiece, size_t targetIndex);

    // Point cloud adaptor for nanoflann
    struct PointCloudAdaptor
    {
        const std::vector<V3f> &points;

        PointCloudAdaptor(const std::vector<V3f> &pts) : points(pts) {}

        inline size_t kdtree_get_point_count() const { return points.size(); }

        inline float kdtree_get_pt(const size_t idx, const size_t dim) const
        {
            return dim == 0 ? points[idx].x : (dim == 1 ? points[idx].y : points[idx].z);
        }

        template <class BBOX>
        bool kdtree_get_bbox(BBOX &) const { return false; }
    };

    using KDTreeType = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>,
        PointCloudAdaptor,
        3 /* dimensionality */
        >;

    struct SearchResult
    {
        std::vector<std::pair<size_t, float>> matches; // index and squared distance
        float effectiveRadius;
    };

    class PieceAttributeHandler
    {
    public:
        // Represents a piece value that can be int, float, string, or V3f
        struct PieceValue
        {
            enum class Type
            {
                None,
                Int,
                Float,
                String,
                Vector
            };

            Type type;
            union Values
            {
                int intValue;
                float floatValue;
                V3f vectorValue;

                Values() : intValue(0) {} // Default constructor initializes first member
                ~Values() {}              // Destructor needed for proper cleanup
            } values;
            std::string stringValue; // Separate since it can't be in union

            // Default constructor
            PieceValue() : type(Type::None) {}

            // Copy constructor
            PieceValue(const PieceValue &other) : type(other.type), stringValue(other.stringValue)
            {
                switch (type)
                {
                case Type::Int:
                    values.intValue = other.values.intValue;
                    break;
                case Type::Float:
                    values.floatValue = other.values.floatValue;
                    break;
                case Type::Vector:
                    new (&values.vectorValue) V3f(other.values.vectorValue);
                    break;
                default:
                    values.intValue = 0;
                    break;
                }
            }

            // Move constructor
            PieceValue(PieceValue &&other) noexcept
                : type(other.type), stringValue(std::move(other.stringValue))
            {
                switch (type)
                {
                case Type::Int:
                    values.intValue = other.values.intValue;
                    break;
                case Type::Float:
                    values.floatValue = other.values.floatValue;
                    break;
                case Type::Vector:
                    new (&values.vectorValue) V3f(std::move(other.values.vectorValue));
                    break;
                default:
                    values.intValue = 0;
                    break;
                }
                other.type = Type::None;
            }

            // Copy assignment operator
            PieceValue &operator=(const PieceValue &other)
            {
                if (this != &other)
                {
                    // Clean up old value if needed
                    if (type == Type::Vector)
                    {
                        values.vectorValue.~V3f();
                    }

                    type = other.type;
                    stringValue = other.stringValue;

                    switch (type)
                    {
                    case Type::Int:
                        values.intValue = other.values.intValue;
                        break;
                    case Type::Float:
                        values.floatValue = other.values.floatValue;
                        break;
                    case Type::Vector:
                        new (&values.vectorValue) V3f(other.values.vectorValue);
                        break;
                    default:
                        values.intValue = 0;
                        break;
                    }
                }
                return *this;
            }

            // Move assignment operator
            PieceValue &operator=(PieceValue &&other) noexcept
            {
                if (this != &other)
                {
                    // Clean up old value if needed
                    if (type == Type::Vector)
                    {
                        values.vectorValue.~V3f();
                    }

                    type = other.type;
                    stringValue = std::move(other.stringValue);

                    switch (type)
                    {
                    case Type::Int:
                        values.intValue = other.values.intValue;
                        break;
                    case Type::Float:
                        values.floatValue = other.values.floatValue;
                        break;
                    case Type::Vector:
                        new (&values.vectorValue) V3f(std::move(other.values.vectorValue));
                        break;
                    default:
                        values.intValue = 0;
                        break;
                    }

                    other.type = Type::None;
                }
                return *this;
            }

            // Destructor
            ~PieceValue()
            {
                if (type == Type::Vector)
                {
                    values.vectorValue.~V3f();
                }
            }

            bool operator==(const PieceValue &other) const
            {
                if (type != other.type)
                    return false;

                switch (type)
                {
                case Type::Int:
                    return values.intValue == other.values.intValue;
                case Type::Float:
                    return std::abs(values.floatValue - other.values.floatValue) < 1e-6f;
                case Type::String:
                    return stringValue == other.stringValue;
                case Type::Vector:
                    return (values.vectorValue - other.values.vectorValue).length() < 1e-6f;
                default:
                    return true; // None type always matches
                }
            }
        };

        static PieceValue getPieceValue(const PrimitiveVariable *var, size_t index)
        {
            if (!var)
                return PieceValue();

            PieceValue result;

            // Try each type in order
            int intValue;
            if (getPieceValueInt(*var, index, intValue))
            {
                result.type = PieceValue::Type::Int;
                result.values.intValue = intValue;
                return result;
            }

            float floatValue;
            if (getPieceValueFloat(*var, index, floatValue))
            {
                result.type = PieceValue::Type::Float;
                result.values.floatValue = floatValue;
                return result;
            }

            std::string stringValue;
            if (getPieceValueString(*var, index, stringValue))
            {
                result.type = PieceValue::Type::String;
                result.stringValue = stringValue;
                return result;
            }

            V3f vectorValue;
            if (getPieceValueV3f(*var, index, vectorValue))
            {
                result.type = PieceValue::Type::Vector;
                result.values.vectorValue = vectorValue;
                return result;
            }

            return result;
        }

        struct SearchContext
        {
            const PrimitiveVariable *sourcePiece;
            const PrimitiveVariable *targetPiece;
            size_t targetIndex;
            PieceValue targetValue;
            bool hasValidPieces;

            SearchContext(
                const PrimitiveVariable *srcPiece,
                const PrimitiveVariable *tgtPiece,
                size_t tgtIndex) : sourcePiece(srcPiece), targetPiece(tgtPiece), targetIndex(tgtIndex)
            {
                hasValidPieces = (sourcePiece && targetPiece);
                if (hasValidPieces)
                {
                    targetValue = getPieceValue(targetPiece, targetIndex);
                }
            }

            bool piecesMatch(size_t sourceIndex) const
            {
                if (!hasValidPieces)
                    return true;

                PieceValue sourceValue = getPieceValue(sourcePiece, sourceIndex);
                return sourceValue == targetValue;
            }
        };

        static std::vector<std::pair<size_t, float>> filterByPiece(
            const std::vector<std::pair<size_t, float>> &candidates,
            const SearchContext &context,
            int minPoints)
        {
            if (!context.hasValidPieces)
                return candidates;

            std::vector<std::pair<size_t, float>> matching;
            std::vector<std::pair<size_t, float>> nonMatching;
            matching.reserve(candidates.size());
            nonMatching.reserve(candidates.size());

            // First pass: separate matching and non-matching pieces
            for (const auto &candidate : candidates)
            {
                if (context.piecesMatch(candidate.first))
                {
                    matching.push_back(candidate);
                }
                else
                {
                    nonMatching.push_back(candidate);
                }
            }

            // If we have enough matching pieces, use only those
            if (matching.size() >= minPoints)
            {
                return matching;
            }

            // Otherwise, add closest non-matching pieces as fallback
            std::sort(nonMatching.begin(), nonMatching.end(),
                      [](const auto &a, const auto &b)
                      { return a.second < b.second; });

            size_t needed = minPoints - matching.size();
            if (needed > nonMatching.size())
                needed = nonMatching.size();

            matching.insert(
                matching.end(),
                nonMatching.begin(),
                nonMatching.begin() + needed);

            // Sort final results by distance
            std::sort(matching.begin(), matching.end(),
                      [](const auto &a, const auto &b)
                      { return a.second < b.second; });

            return matching;
        }
    };

    class CapturePointFinder
    {
    public:
        CapturePointFinder(const std::vector<V3f> &sourcePoints)
            : m_adaptor(sourcePoints), m_kdtree(3, m_adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10))
        {
            m_kdtree.buildIndex();
        }

        SearchResult findPoints(
            const V3f &queryPoint,
            float radius,
            int maxPoints,
            int minPoints,
            const PrimitiveVariable *sourcePiece,
            const PrimitiveVariable *targetPiece,
            size_t targetIndex)
        {
            // Create search context for piece matching
            PieceAttributeHandler::SearchContext context(sourcePiece, targetPiece, targetIndex);

            // Phase 1: Try with given radius
            SearchResult result = findPointsWithPiece(
                queryPoint, radius, maxPoints, context);

            // Phase 2: If we don't have enough points, do unlimited radius search
            if (result.matches.size() < minPoints)
            {
                // Use unlimited radius to find at least minPoints
                const float unlimitedRadius = std::numeric_limits<float>::max();
                result = findPointsWithPiece(
                    queryPoint, unlimitedRadius, minPoints, context);

                // Adjust radius to 1.1 * distance to furthest point (like Houdini)
                if (!result.matches.empty())
                {
                    float maxDist = std::sqrt(result.matches.back().second);
                    result.effectiveRadius = maxDist * 1.1f;

                    // Recompute weights using the adjusted radius
                    for (auto &match : result.matches)
                    {
                        match.second = std::sqrt(match.second) / result.effectiveRadius;
                    }
                }
            }
            else
            {
                // Using original radius, normalize distances
                result.effectiveRadius = radius;
                for (auto &match : result.matches)
                {
                    match.second = std::sqrt(match.second) / radius;
                }
            }

            return result;
        }

    private:
        PointCloudAdaptor m_adaptor;
        KDTreeType m_kdtree;

        SearchResult findNearestPoints(const V3f &queryPoint, float radius, int maxPoints)
        {
            SearchResult result;
            std::vector<std::pair<size_t, float>> matches;

            const float queryPt[3] = {queryPoint.x, queryPoint.y, queryPoint.z};

            // Use radius search if we have a finite radius
            if (radius < std::numeric_limits<float>::max())
            {
                matches.reserve(maxPoints * 2); // Reserve extra space
                const float radiusSquared = radius * radius;

                // Use nanoflann's ResultItem type for radius search
                std::vector<nanoflann::ResultItem<uint32_t, float>> radiusMatches;
                m_kdtree.radiusSearch(queryPt, radiusSquared, radiusMatches);

                // Convert to our format and sort
                matches.reserve(radiusMatches.size());
                for (const auto &match : radiusMatches)
                {
                    matches.emplace_back(match.first, match.second);
                }

                // Sort by distance and limit to maxPoints
                std::sort(matches.begin(), matches.end(),
                          [](const auto &a, const auto &b)
                          { return a.second < b.second; });

                if (matches.size() > maxPoints)
                {
                    matches.resize(maxPoints);
                }
            }
            else
            {
                // Use k-nearest neighbor search for unlimited radius
                std::vector<uint32_t> indices(maxPoints);
                std::vector<float> distances(maxPoints);

                size_t found = m_kdtree.knnSearch(queryPt, static_cast<size_t>(maxPoints), indices.data(), distances.data());

                // Convert to pair format
                matches.reserve(found);
                for (size_t i = 0; i < found; ++i)
                {
                    matches.emplace_back(indices[i], distances[i]);
                }
            }

            result.matches = std::move(matches);
            return result;
        }

        SearchResult findPointsWithPiece(
            const V3f &queryPoint,
            float radius,
            int numPoints,
            const PieceAttributeHandler::SearchContext &context)
        {
            SearchResult result;

            // Start with more candidates than needed
            int candidateMultiplier = 4;
            int maxCandidates = numPoints * candidateMultiplier;

            while (true)
            {
                // Get candidates
                auto candidates = findNearestPoints(queryPoint, radius, maxCandidates);

                // Filter by piece attribute
                auto matchingPieces = PieceAttributeHandler::filterByPiece(
                    candidates.matches, context, numPoints);

                // If we have enough matching pieces or can't get more candidates
                if (matchingPieces.size() >= numPoints ||
                    candidates.matches.size() < maxCandidates)
                {

                    // Limit to numPoints
                    if (matchingPieces.size() > numPoints)
                    {
                        matchingPieces.resize(numPoints);
                    }

                    result.matches = std::move(matchingPieces);
                    result.effectiveRadius = radius;
                    return result;
                }

                // Try with more candidates
                candidateMultiplier *= 2;
                maxCandidates = numPoints * candidateMultiplier;
            }
        }
    };

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

    inline float calculateWeight(float distSquared, float radius)
    {
        float r2 = distSquared / (radius * radius);
        float weight = 1.0f - r2;
        return weight * weight;
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

    void computeCaptureWeights(
        const std::vector<V3f> &sourcePoints,
        const std::vector<V3f> &targetPoints,
        float radius,
        int maxPoints,
        int minPoints,
        const PrimitiveVariable *sourcePiece,
        const PrimitiveVariable *targetPiece,
        BatchResults &results)
    {
        // Create point finder
        CapturePointFinder pointFinder(sourcePoints);

        // Process each target point in parallel
        parallel_for(blocked_range<size_t>(0, targetPoints.size(), 1024),
                     [&](const blocked_range<size_t> &range)
                     {
                         for (size_t i = range.begin(); i != range.end(); ++i)
                         {
                             const V3f &targetPoint = targetPoints[i];

                             // Find nearest points
                             auto searchResult = pointFinder.findPoints(
                                 targetPoint, radius, maxPoints, minPoints,
                                 sourcePiece, targetPiece, i);

                             // Store influence count
                             results.influenceCounts[i] = searchResult.matches.size();

                             // Calculate and normalize weights
                             float totalWeight = 0.0f;
                             std::vector<float> weights;
                             weights.reserve(searchResult.matches.size());

                             for (const auto &match : searchResult.matches)
                             {
                                 float weight = calculateWeight(match.second, searchResult.effectiveRadius);
                                 weights.push_back(weight);
                                 totalWeight += weight;
                             }

                             // Store results
                             int *indices = results.getIndices(i, maxPoints);
                             float *outWeights = results.getWeights(i, maxPoints);

                             const float invTotalWeight = totalWeight > 0.0f ? 1.0f / totalWeight : 0.0f;

                             for (size_t j = 0; j < maxPoints; ++j)
                             {
                                 if (j < searchResult.matches.size())
                                 {
                                     indices[j] = searchResult.matches[j].first;
                                     outWeights[j] = weights[j] * invTotalWeight;
                                 }
                                 else
                                 {
                                     indices[j] = 0;
                                     outWeights[j] = 0.0f;
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

    computeCaptureWeights(sourcePoints, targetPoints, radius, maxPoints, minPoints,
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
