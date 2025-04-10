#include <math.h>
#include <Eigen/Dense>
#include "GafferScotch/MatrixDeform.h"
#include "GafferScotch/ScenePathUtil.h"

#include "IECoreScene/Primitive.h"

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/cache_aligned_allocator.h>

#include <iostream>
#include <iomanip>

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
            const int *indices;       // Points to the array of indices for this influence level
            const float *weights;     // Points to the array of weights for this influence level
            size_t count;
        };
        std::vector<Entry> influences;

        // Add a method to get all influences for a specific point
        void getPointInfluences(size_t pointIndex, std::vector<std::pair<int, float>> &pointInfluences) const
        {
            pointInfluences.clear();
            for (const auto &influence : influences)
            {
                if (pointIndex >= influence.count)
                    continue;

                const int idx = influence.indices[pointIndex];
                const float weight = influence.weights[pointIndex];

                if (idx >= 0 && weight > 0.0f)
                {
                    pointInfluences.emplace_back(idx, weight);
                }
            }
        }
    };

    bool getInfluenceData(const Primitive *primitive, int maxInfluences, InfluenceData &data)
    {
        data.influences.clear();
        data.influences.reserve(maxInfluences);

        for (int i = 1; i <= maxInfluences; ++i)
        {
            std::string indexName = "captureIndex" + std::to_string(i);
            std::string weightName = "captureWeight" + std::to_string(i);

            auto indexIt = primitive->variables.find(indexName);
            auto weightIt = primitive->variables.find(weightName);

            if (indexIt == primitive->variables.end() || weightIt == primitive->variables.end())
                continue;

            const IntVectorData *indices = runTimeCast<const IntVectorData>(indexIt->second.data.get());
            const FloatVectorData *weights = runTimeCast<const FloatVectorData>(weightIt->second.data.get());

            if (!indices || !weights)
                continue;

            InfluenceData::Entry entry;
            entry.indices = indices->readable().data();
            entry.weights = weights->readable().data();
            entry.count = indices->readable().size();

            data.influences.push_back(std::move(entry));
        }

        return !data.influences.empty();
    }

    M44f polarDecomposition(const M44f &m)
    {
        // Extract the 3x3 rotation+scale component
        Eigen::Matrix3f A;
        A << m[0][0], m[0][1], m[0][2],
             m[1][0], m[1][1], m[1][2],
             m[2][0], m[2][1], m[2][2];
        
        // Compute SVD
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        
        // Reconstruct rotation matrix
        Eigen::Matrix3f R = svd.matrixU() * svd.matrixV().transpose();
        
        // Ensure proper rotation (determinant should be 1)
        if (R.determinant() < 0) {
            Eigen::Matrix3f correction = Eigen::Matrix3f::Identity();
            correction(2, 2) = -1;
            R = svd.matrixU() * correction * svd.matrixV().transpose();
        }
        
        // Reconstruct as 4x4 matrix with original translation
        M44f result;
        result[0][0] = R(0, 0); result[0][1] = R(0, 1); result[0][2] = R(0, 2); result[0][3] = m[0][3];
        result[1][0] = R(1, 0); result[1][1] = R(1, 1); result[1][2] = R(1, 2); result[1][3] = m[1][3];
        result[2][0] = R(2, 0); result[2][1] = R(2, 1); result[2][2] = R(2, 2); result[2][3] = m[2][3];
        result[3][0] = m[3][0]; result[3][1] = m[3][1]; result[3][2] = m[3][2]; result[3][3] = m[3][3];
        
        return result;
    }

    inline void computeMatrixDeformation(
        const V3f &currentPos,
        const V3f *staticPos,
        const V3f *animatedPos,
        const M44f *sourceMatrices,
        const InfluenceData::Entry &influence,
        size_t vertexIndex,
        V3f &outDeformedPos,
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
        const M44f &sourceMatrix = sourceMatrices[sourceIndex];

        // Create a transform from static to animated using the source matrix
        M44f staticToWorld = sourceMatrix;
        
        // Get the difference between static and animated positions
        V3f translation = animatedPoint - staticPoint;
        
        // Add the translation to the matrix
        M44f animatedMatrix = staticToWorld;
        animatedMatrix[0][3] += translation.x;
        animatedMatrix[1][3] += translation.y;
        animatedMatrix[2][3] += translation.z;
        
        // Calculate full transform between rest and deformed state
        M44f transform = staticToWorld.inverse() * animatedMatrix;
        
        // Apply transformation to current point in local space
        V3f localPoint = currentPos - staticPoint;
        V3f transformedPoint;
        transform.multVecMatrix(localPoint, transformedPoint);
        
        // Add result back to world space
        V3f newPoint = transformedPoint + animatedPoint;
        
        // Weighted accumulation
        outDeformedPos += newPoint * weight;
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

IE_CORE_DEFINERUNTIMETYPED(MatrixDeform);

size_t MatrixDeform::g_firstPlugIndex = 0;

MatrixDeform::MatrixDeform(const std::string &name)
    : Deformer(name)
{
    storeIndexOfNextChild(g_firstPlugIndex);

    addChild(new ScenePlug("staticDeformer"));
    addChild(new ScenePlug("animatedDeformer"));
    addChild(new StringPlug("deformerPath", Plug::In, ""));
    addChild(new BoolPlug("rigidProjection", Plug::In, true));
    addChild(new BoolPlug("cleanupAttributes", Plug::In, true));
}

ScenePlug *MatrixDeform::staticDeformerPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

const ScenePlug *MatrixDeform::staticDeformerPlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

ScenePlug *MatrixDeform::animatedDeformerPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex + 1);
}

const ScenePlug *MatrixDeform::animatedDeformerPlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex + 1);
}

StringPlug *MatrixDeform::deformerPathPlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 2);
}

const StringPlug *MatrixDeform::deformerPathPlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 2);
}

BoolPlug *MatrixDeform::rigidProjectionPlug()
{
    return getChild<BoolPlug>(g_firstPlugIndex + 3);
}

const BoolPlug *MatrixDeform::rigidProjectionPlug() const
{
    return getChild<BoolPlug>(g_firstPlugIndex + 3);
}

BoolPlug *MatrixDeform::cleanupAttributesPlug()
{
    return getChild<BoolPlug>(g_firstPlugIndex + 4);
}

const BoolPlug *MatrixDeform::cleanupAttributesPlug() const
{
    return getChild<BoolPlug>(g_firstPlugIndex + 4);
}

void MatrixDeform::affects(const Plug *input, AffectedPlugsContainer &outputs) const
{
    Deformer::affects(input, outputs);

    if (input == staticDeformerPlug()->objectPlug() ||
        input == animatedDeformerPlug()->objectPlug() ||
        input == deformerPathPlug() ||
        input == rigidProjectionPlug() ||
        input == cleanupAttributesPlug())
    {
        outputs.push_back(outPlug()->objectPlug());
    }
}

bool MatrixDeform::affectsProcessedObjectBound(const Plug *input) const
{
    return input == staticDeformerPlug()->objectPlug() ||
           input == animatedDeformerPlug()->objectPlug() ||
           input == deformerPathPlug() ||
           input == rigidProjectionPlug();
}

bool MatrixDeform::affectsProcessedObject(const Plug *input) const
{
    return input == staticDeformerPlug()->objectPlug() ||
           input == animatedDeformerPlug()->objectPlug() ||
           input == deformerPathPlug() ||
           input == rigidProjectionPlug() ||
           input == cleanupAttributesPlug();
}

void MatrixDeform::hashProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
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
        rigidProjectionPlug()->hash(h);
        cleanupAttributesPlug()->hash(h);
    }
    else
    {
        h.append(inPlug()->objectHash(path));
        h.append(staticDeformerPlug()->objectHash(deformerPath));
        h.append(animatedDeformerPlug()->objectHash(deformerPath));
        deformerPathPlug()->hash(h);
        rigidProjectionPlug()->hash(h);
        cleanupAttributesPlug()->hash(h);
    }
}

void MatrixDeform::hashProcessedObjectBound(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    hashProcessedObject(path, context, h);
}

Imath::Box3f MatrixDeform::computeProcessedObjectBound(const ScenePath &path, const Gaffer::Context *context) const
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

    // Check for matrices - if none, just use positions for bound calculation
    bool hasMatrices = staticDeformerPrimitive->variables.find("captureMatrices") != staticDeformerPrimitive->variables.end();
    
    if (!hasMatrices)
    {
        // Fallback to simple displacement calculation
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
    else
    {
        // When we have matrix information, the bounds can grow more due to rotation
        // Use a more conservative estimate based on the max possible scale from matrices
        float maxDistance = 0;
        
        // Find maximum distance from any input point to bounds center
        V3f center = inputBound.center();
        for (auto p : staticPos)
        {
            maxDistance = std::max(maxDistance, (p - center).length());
        }
        
        // Add a generous padding factor to account for rotation
        maxDistance *= 1.5f;
        
        Box3f result = inputBound;
        V3f padding(maxDistance, maxDistance, maxDistance);
        result.min -= padding;
        result.max += padding;
        
        return result;
    }
}

IECore::ConstObjectPtr MatrixDeform::computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject) const
{
    const Primitive *inputPrimitive = runTimeCast<const Primitive>(inputObject);
    if (!inputPrimitive)
        return inputObject;

    const ScenePath deformerPath = GafferScotch::makeScenePath(deformerPathPlug()->getValue());

    // Get static and animated deformer objects
    ConstObjectPtr staticDeformerObject = staticDeformerPlug()->object(deformerPath);
    ConstObjectPtr animatedDeformerObject = animatedDeformerPlug()->object(deformerPath);

    const Primitive *staticDeformerPrimitive = runTimeCast<const Primitive>(staticDeformerObject.get());
    const Primitive *animatedDeformerPrimitive = runTimeCast<const Primitive>(animatedDeformerObject.get());

    if (!staticDeformerPrimitive || !animatedDeformerPrimitive)
        return inputObject;

    // Get all position data
    PrimitivePtr result = inputPrimitive->copy();
    auto pIt = result->variables.find("P");
    if (pIt == result->variables.end())
        return result;

    V3fVectorDataPtr positionData = runTimeCast<V3fVectorData>(pIt->second.data);
    if (!positionData)
        return result;

    // Get static and animated positions
    auto staticPIt = staticDeformerPrimitive->variables.find("P");
    auto animatedPIt = animatedDeformerPrimitive->variables.find("P");

    if (staticPIt == staticDeformerPrimitive->variables.end() ||
        animatedPIt == animatedDeformerPrimitive->variables.end())
        return result;

    const V3fVectorData *staticPositions = runTimeCast<const V3fVectorData>(staticPIt->second.data.get());
    const V3fVectorData *animatedPositions = runTimeCast<const V3fVectorData>(animatedPIt->second.data.get());

    if (!staticPositions || !animatedPositions)
        return result;

    const std::vector<V3f> &staticPos = staticPositions->readable();
    const std::vector<V3f> &animatedPos = animatedPositions->readable();
    std::vector<V3f> &positions = positionData->writable();

    // Try to get the transformation matrices, might be missing
    const M44fVectorData *captureMatrices = nullptr;
    auto matricesIt = staticDeformerPrimitive->variables.find("captureMatrices");
    if (matricesIt != staticDeformerPrimitive->variables.end())
    {
        captureMatrices = runTimeCast<const M44fVectorData>(matricesIt->second.data.get());
    }

    // Get the rigid projection flag
    bool applyRigidProjection = rigidProjectionPlug()->getValue();

    // Get influence data
    auto captureInfluencesIt = inputPrimitive->variables.find("captureInfluences");
    if (captureInfluencesIt == inputPrimitive->variables.end())
        return result;

    const std::vector<int> &numInfluences = runTimeCast<const IntVectorData>(captureInfluencesIt->second.data.get())->readable();

    // Count max influences
    int maxInfluences = 0;
    for (int count : numInfluences)
    {
        maxInfluences = std::max(maxInfluences, count);
    }

    // Get influence data
    InfluenceData influenceData;
    if (!getInfluenceData(inputPrimitive, maxInfluences, influenceData))
        return result;

    // Process each point in parallel
    parallel_for(blocked_range<size_t>(0, positions.size()),
                 [&](const blocked_range<size_t> &range)
                 {
                     for (size_t i = range.begin(); i != range.end(); ++i)
                     {
                         const int maxInfluences = (i < numInfluences.size()) ? numInfluences[i] : 0;
                         if (maxInfluences <= 0)
                             continue;

                         // If we have matrices, use matrix-based deformation
                         if (captureMatrices)
                         {
                             const std::vector<M44f> &matrices = captureMatrices->readable();
                             
                             // Process with matrix-based deformation
                             V3f deformedPos(0);
                             float totalWeight = 0;
                             
                             // Process each influence
                             for (int j = 0; j < influenceData.influences.size() && j < maxInfluences; ++j)
                             {
                                 computeMatrixDeformation(
                                     positions[i],
                                     staticPos.data(),
                                     animatedPos.data(),
                                     matrices.data(),
                                     influenceData.influences[j],
                                     i,
                                     deformedPos,
                                     totalWeight
                                 );
                             }
                             
                             // Apply accumulated weighted transform
                             if (totalWeight > 0)
                             {
                                 positions[i] = deformedPos / totalWeight;
                             }
                         }
                         else
                         {
                             // Fallback to simple translation-based deformation
                             V3f totalOffset(0);
                             float totalWeight = 0;

                             // Process each influence
                             for (int j = 0; j < influenceData.influences.size() && j < maxInfluences; ++j)
                             {
                                 const InfluenceData::Entry &influence = influenceData.influences[j];
                                 const float weight = influence.weights[i];
                                 
                                 if (weight <= 0.0f)
                                     continue;
                                     
                                 const int idx = influence.indices[i];
                                 if (idx < 0 || idx >= staticPos.size())
                                     continue;
                                     
                                 // Simple translation
                                 V3f offset = animatedPos[idx] - staticPos[idx];
                                 totalOffset += offset * weight;
                                 totalWeight += weight;
                             }
                             
                             // Apply accumulated weighted translations
                             if (totalWeight > 0)
                             {
                                 positions[i] += totalOffset;
                             }
                         }
                     }
                 });

    // If requested, clean up the capture attributes
    if (cleanupAttributesPlug()->getValue())
    {
        result->variables.erase("captureInfluences");
        for (int i = 1; i <= maxInfluences; ++i)
        {
            result->variables.erase("captureIndex" + std::to_string(i));
            result->variables.erase("captureWeight" + std::to_string(i));
        }
        
        // Also clean up matrix attributes
        result->variables.erase("captureMatrices");
        result->variables.erase("captureFrameNormals");
        result->variables.erase("captureFrameTangents");
        result->variables.erase("captureFrameBitangents");
    }

    return result;
} 