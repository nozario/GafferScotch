#include "GafferScotch/CurvesPostProcess.h"

#include "IECore/NullObject.h"
#include "IECoreScene/CurvesPrimitive.h"
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
    // Helper function to find adjacent vertices in a curve
    struct CurveNeighborFinder
    {
        CurveNeighborFinder(const std::vector<int> &vertsPerCurve)
        {
            int offset = 0;
            curveStartIndices.reserve(vertsPerCurve.size());

            for (int vertCount : vertsPerCurve)
            {
                curveStartIndices.push_back(offset);
                offset += vertCount;
            }
            curveStartIndices.push_back(offset); // End marker
        }

        // Find neighbor vertices for a vertex index
        void getNeighbors(int vertIndex, std::vector<int> &neighbors) const
        {
            neighbors.clear();

            // Find which curve this vertex belongs to
            auto it = std::upper_bound(curveStartIndices.begin(), curveStartIndices.end(), vertIndex);
            if (it == curveStartIndices.begin() || it == curveStartIndices.end())
            {
                return;
            }

            int curveIndex = static_cast<int>(std::distance(curveStartIndices.begin(), it)) - 1;
            int startIndex = curveStartIndices[curveIndex];
            int endIndex = curveStartIndices[curveIndex + 1];
            int localIndex = vertIndex - startIndex;
            int curveLength = endIndex - startIndex;

            // Add adjacent vertices
            if (localIndex > 0)
            {
                neighbors.push_back(vertIndex - 1);
            }

            if (localIndex < curveLength - 1)
            {
                neighbors.push_back(vertIndex + 1);
            }
        }

    private:
        std::vector<int> curveStartIndices;
    };
}

IE_CORE_DEFINERUNTIMETYPED(CurvesPostProcess);

size_t CurvesPostProcess::g_firstPlugIndex = 0;

CurvesPostProcess::CurvesPostProcess(const std::string &name)
    : ObjectProcessor(name)
{
    storeIndexOfNextChild(g_firstPlugIndex);

    // Taubin smoothing
    addChild(new BoolPlug("enableTaubinSmoothing", Plug::In, false));
    addChild(new FloatPlug("lambda", Plug::In, 0.5f, 0.0f, 2.0f));
    addChild(new FloatPlug("mu", Plug::In, -0.53f, -2.0f, 0.0f));
    addChild(new IntPlug("iterations", Plug::In, 10, 1, 100));

    // End Points Fix
    addChild(new BoolPlug("enableEndPointsFix", Plug::In, false));
}

BoolPlug *CurvesPostProcess::enableTaubinSmoothingPlug()
{
    return getChild<BoolPlug>(g_firstPlugIndex);
}

const BoolPlug *CurvesPostProcess::enableTaubinSmoothingPlug() const
{
    return getChild<BoolPlug>(g_firstPlugIndex);
}

FloatPlug *CurvesPostProcess::lambdaPlug()
{
    return getChild<FloatPlug>(g_firstPlugIndex + 1);
}

const FloatPlug *CurvesPostProcess::lambdaPlug() const
{
    return getChild<FloatPlug>(g_firstPlugIndex + 1);
}

FloatPlug *CurvesPostProcess::muPlug()
{
    return getChild<FloatPlug>(g_firstPlugIndex + 2);
}

const FloatPlug *CurvesPostProcess::muPlug() const
{
    return getChild<FloatPlug>(g_firstPlugIndex + 2);
}

IntPlug *CurvesPostProcess::iterationsPlug()
{
    return getChild<IntPlug>(g_firstPlugIndex + 3);
}

const IntPlug *CurvesPostProcess::iterationsPlug() const
{
    return getChild<IntPlug>(g_firstPlugIndex + 3);
}

BoolPlug *CurvesPostProcess::enableEndPointsFixPlug()
{
    return getChild<BoolPlug>(g_firstPlugIndex + 4);
}

const BoolPlug *CurvesPostProcess::enableEndPointsFixPlug() const
{
    return getChild<BoolPlug>(g_firstPlugIndex + 4);
}

void CurvesPostProcess::affects(const Gaffer::Plug *input, AffectedPlugsContainer &outputs) const
{
    ObjectProcessor::affects(input, outputs);

    if (input == enableTaubinSmoothingPlug() ||
        input == lambdaPlug() ||
        input == muPlug() ||
        input == iterationsPlug() ||
        input == enableEndPointsFixPlug())
    {
        outputs.push_back(outPlug()->objectPlug());
    }
}

bool CurvesPostProcess::affectsProcessedObject(const Gaffer::Plug *input) const
{
    return input == enableTaubinSmoothingPlug() ||
           input == lambdaPlug() ||
           input == muPlug() ||
           input == iterationsPlug() ||
           input == enableEndPointsFixPlug();
}

void CurvesPostProcess::hashProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    ObjectProcessor::hashProcessedObject(path, context, h);

    enableTaubinSmoothingPlug()->hash(h);
    
    if (enableTaubinSmoothingPlug()->getValue())
    {
        lambdaPlug()->hash(h);
        muPlug()->hash(h);
        iterationsPlug()->hash(h);
    }
}

IECore::ConstObjectPtr CurvesPostProcess::computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject) const
{
    const CurvesPrimitive *inputCurves = runTimeCast<const CurvesPrimitive>(inputObject);
    if (!inputCurves)
    {
        return inputObject;
    }

    // Create a copy of the input curves
    CurvesPrimitivePtr result = inputCurves->copy();

    // Apply Taubin smoothing if enabled
    if (enableTaubinSmoothingPlug()->getValue())
    {
        const float lambda = lambdaPlug()->getValue();
        const float mu = muPlug()->getValue();
        const int iterations = iterationsPlug()->getValue();

        applyTaubinSmoothing(result, lambda, mu, iterations);
    }

    // Apply end points fix if enabled
    if (enableEndPointsFixPlug()->getValue())
    {
        applyEndPointsFix(result);
    }

    return result;
}

void CurvesPostProcess::applyTaubinSmoothing(
    CurvesPrimitivePtr curves, 
    float lambda, 
    float mu, 
    int iterations) const
{
    auto pIt = curves->variables.find("P");
    if (pIt == curves->variables.end())
    {
        return;
    }

    V3fVectorDataPtr positions = runTimeCast<V3fVectorData>(pIt->second.data);
    if (!positions)
    {
        return;
    }

    std::vector<V3f> &pos = positions->writable();
    const std::vector<int> &vertsPerCurve = curves->verticesPerCurve()->readable();

    // Skip if empty
    if (pos.empty() || vertsPerCurve.empty())
    {
        return;
    }

    // Precompute curve topology for neighbor finding
    CurveNeighborFinder neighborFinder(vertsPerCurve);

    // Temporary buffer for new positions during smoothing
    std::vector<V3f> newPos(pos.size());

    // Perform Taubin smoothing iterations
    for (int iter = 0; iter < iterations; ++iter)
    {
        // First pass with positive lambda weight
        parallel_for(blocked_range<size_t>(0, pos.size()),
            [&](const blocked_range<size_t> &range)
            {
                std::vector<int> neighbors;
                neighbors.reserve(2);  // Typically 2 neighbors for curves

                for (size_t i = range.begin(); i != range.end(); ++i)
                {
                    neighborFinder.getNeighbors(static_cast<int>(i), neighbors);
                    
                    // If no neighbors, keep original position
                    if (neighbors.empty())
                    {
                        newPos[i] = pos[i];
                        continue;
                    }

                    // Calculate centroid of neighbors
                    V3f centroid(0, 0, 0);
                    for (int neighbor : neighbors)
                    {
                        centroid += pos[neighbor];
                    }
                    centroid /= neighbors.size();

                    // Apply lambda weight (positive) smoothing
                    newPos[i] = pos[i] + lambda * (centroid - pos[i]);
                }
            });

        // Copy new positions to current positions
        pos.swap(newPos);

        // Second pass with negative mu weight (anti-shrinking)
        parallel_for(blocked_range<size_t>(0, pos.size()),
            [&](const blocked_range<size_t> &range)
            {
                std::vector<int> neighbors;
                neighbors.reserve(2);  // Typically 2 neighbors for curves

                for (size_t i = range.begin(); i != range.end(); ++i)
                {
                    neighborFinder.getNeighbors(static_cast<int>(i), neighbors);
                    
                    // If no neighbors, keep original position
                    if (neighbors.empty())
                    {
                        newPos[i] = pos[i];
                        continue;
                    }

                    // Calculate centroid of neighbors
                    V3f centroid(0, 0, 0);
                    for (int neighbor : neighbors)
                    {
                        centroid += pos[neighbor];
                    }
                    centroid /= neighbors.size();

                    // Apply mu weight (negative) anti-shrinking
                    newPos[i] = pos[i] + mu * (centroid - pos[i]);
                }
            });

        // Copy new positions to current positions
        pos.swap(newPos);
    }
}

void CurvesPostProcess::applyEndPointsFix(IECoreScene::CurvesPrimitivePtr curves) const
{
    auto pIt = curves->variables.find("P");
    if (pIt == curves->variables.end())
    {
        return;
    }

    V3fVectorDataPtr positions = runTimeCast<V3fVectorData>(pIt->second.data);
    if (!positions)
    {
        return;
    }

    std::vector<V3f> &pos = positions->writable();
    const std::vector<int> &vertsPerCurve = curves->verticesPerCurve()->readable();

    // Skip if empty
    if (pos.empty() || vertsPerCurve.empty())
    {
        return;
    }

    // Outer parallel loop over curves
    parallel_for(blocked_range<size_t>(0, vertsPerCurve.size()),
        [&](const blocked_range<size_t> &curveRange)
        {
            size_t offset = 0;
            for (size_t curveIndex = 0; curveIndex < curveRange.begin(); ++curveIndex)
            {
                offset += vertsPerCurve[curveIndex];
            }

            for (size_t curveIndex = curveRange.begin(); curveIndex != curveRange.end(); ++curveIndex)
            {
                int vertCount = vertsPerCurve[curveIndex];
                if (vertCount < 2)
                {
                    offset += vertCount;
                    continue;
                }

                // Calculate distances between consecutive points
                std::vector<float> distances(vertCount - 1);
                for (int i = 0; i < vertCount - 1; ++i)
                {
                    distances[i] = (pos[offset + i + 1] - pos[offset + i]).length();
                }

                // Calculate median distance
                std::nth_element(distances.begin(), distances.begin() + distances.size() / 2, distances.end());
                float medianDistance = distances[distances.size() / 2];

                // Focus on the last 2-5 points
                int startFixIndex = std::max(vertCount - 5, 1);
                for (int i = startFixIndex; i < vertCount; ++i)
                {
                    float distance = (pos[offset + i] - pos[offset + i - 1]).length();
                    if (distance > 1.2f * medianDistance) // Adjusted threshold factor
                    {
                        // Weighted realignment
                        float weight = 1.0f - (i - startFixIndex) / float(vertCount - startFixIndex);
                        V3f direction = (pos[offset + i] - pos[offset + i - 1]).normalized();
                        pos[offset + i] = pos[offset + i - 1] + direction * medianDistance * weight;
                    }
                }

                offset += vertCount;
            }
        });
} 