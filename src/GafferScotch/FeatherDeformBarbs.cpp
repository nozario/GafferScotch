#include "GafferScotch/FeatherDeformBarbs.h"

#include "GafferScene/SceneAlgo.h"

#include "IECoreScene/CurvesPrimitive.h"
// #include "IECoreScene/ObjectAlgo.h"
#include "IECore/NullObject.h"

// Anonymous namespace for helpers, similar to FeatherAttachBarbs
namespace
{
    // Helper to safely normalize a V3f
    inline Imath::V3f safeNormalize(const Imath::V3f &v)
    {
        float length = v.length();
        if (length > 1e-6f) // Use a small epsilon
        {
            return v / length;
        }
        return Imath::V3f(0.0f);
    }

    // Helper to get a tangent for a point on a curve
    Imath::V3f calculateTangent(const std::vector<Imath::V3f> &points, size_t curveStartIndex, size_t numCurvePoints, size_t pointLocalIndex)
    {
        if (numCurvePoints <= 1)
            return Imath::V3f(1, 0, 0);
        const Imath::V3f &pt = points[curveStartIndex + pointLocalIndex];
        if (pointLocalIndex < numCurvePoints - 1)
        {
            return safeNormalize(points[curveStartIndex + pointLocalIndex + 1] - pt);
        }
        else
        {
            return safeNormalize(pt - points[curveStartIndex + pointLocalIndex - 1]);
        }
    }

    struct Frame
    {
        Imath::V3f P = Imath::V3f(0.0f);
        Imath::V3f T = Imath::V3f(1.0f, 0.0f, 0.0f);
        Imath::V3f B = Imath::V3f(0.0f, 1.0f, 0.0f);
        Imath::V3f N = Imath::V3f(0.0f, 0.0f, 1.0f);
        bool valid = true;

        void fromQuaternion(const Imath::V4f &q)
        {
            Imath::M44f m = Imath::M44f().setQuat(Imath::Quatf(q.x, q.y, q.z, q.w));
            T = safeNormalize(Imath::V3f(m[0][0], m[1][0], m[2][0]));
            B = safeNormalize(Imath::V3f(m[0][1], m[1][1], m[2][1]));
            N = safeNormalize(Imath::V3f(m[0][2], m[1][2], m[2][2]));
            if (T.length() < 0.5f || B.length() < 0.5f || N.length() < 0.5f || std::abs(T.dot(B)) > 0.1f || std::abs(T.dot(N)) > 0.1f || std::abs(B.dot(N)) > 0.1f)
            {
                valid = false;
            }
        }

        void fromUpVector(const Imath::V3f &tangent, const Imath::V3f &up)
        {
            T = safeNormalize(tangent);
            B = safeNormalize(T.cross(up));
            if (B.length() < 0.5f)
            {
                valid = false;
                return;
            }
            N = safeNormalize(B.cross(T));
            if (N.length() < 0.5f)
                valid = false;
        }

        void fromDefault(const Imath::V3f &tangent)
        {
            T = safeNormalize(tangent);
            Imath::V3f refUp = Imath::V3f(0, 1, 0);
            if (std::abs(T.dot(refUp)) > 0.99f)
                refUp = Imath::V3f(1, 0, 0);
            B = safeNormalize(T.cross(refUp));
            if (B.length() < 0.5f)
            {
                valid = false;
                return;
            }
            N = safeNormalize(B.cross(T));
            if (N.length() < 0.5f)
                valid = false;
        }
    };

    // ShaftInfo struct, same as in FeatherAttachBarbs
    struct AnimatedShaftInfo
    {
        size_t primitiveIndex;
        const std::vector<Imath::V3f> *points; // Points for this specific animated shaft curve
        size_t numPoints;
        const Imath::V3f *upVector;                           // Optional, points to a single V3f if uniform and valid
        const Imath::V4f *orientQuaternion;                   // Optional, points to a single V4f if uniform and valid
        PrimitiveVariable::Interpolation orientInterpolation; // If orientQuaternion is per-vertex
    };
}

using namespace Gaffer;
using namespace GafferScene;
using namespace GafferScotch;
using namespace IECore;
using namespace IECoreScene;

IE_CORE_DEFINERUNTIMETYPED(FeatherDeformBarbs);

size_t FeatherDeformBarbs::g_firstPlugIndex = 0;

FeatherDeformBarbs::FeatherDeformBarbs(const std::string &name)
    : Deformer(name)
{
    storeIndexOfNextChild(g_firstPlugIndex);

    addChild(new ScenePlug("animatedShafts", Plug::In, Plug::Default & ~Plug::Serialisable));
    // inPlug() from Deformer will take the barbs geometry (output of FeatherAttachBarbs)

    addChild(new StringPlug("hairIdAttrName", Plug::In, "hair_id"));
    addChild(new StringPlug("shaftUpVectorPrimVarName", Plug::In, ""));
    addChild(new StringPlug("shaftPointOrientAttrName", Plug::In, ""));
    addChild(new BoolPlug("cleanupBindAttributes", Plug::In, true));

    // Pass through attributes and transform by default, as Deformer doesn't do this automatically.
    outPlug()->attributesPlug()->setInput(inPlug()->attributesPlug());
    outPlug()->transformPlug()->setInput(inPlug()->transformPlug());
    // Bound is computed by Deformer::computeBoundFromInput()
}

FeatherDeformBarbs::~FeatherDeformBarbs()
{
}

ScenePlug *FeatherDeformBarbs::animatedShaftsPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

const ScenePlug *FeatherDeformBarbs::animatedShaftsPlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

Gaffer::StringPlug *FeatherDeformBarbs::hairIdAttrNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 1);
}

const Gaffer::StringPlug *FeatherDeformBarbs::hairIdAttrNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 1);
}

Gaffer::StringPlug *FeatherDeformBarbs::shaftUpVectorPrimVarNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 2);
}

const Gaffer::StringPlug *FeatherDeformBarbs::shaftUpVectorPrimVarNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 2);
}

Gaffer::StringPlug *FeatherDeformBarbs::shaftPointOrientAttrNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 3);
}

const Gaffer::StringPlug *FeatherDeformBarbs::shaftPointOrientAttrNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 3);
}

Gaffer::BoolPlug *FeatherDeformBarbs::cleanupBindAttributesPlug()
{
    return getChild<BoolPlug>(g_firstPlugIndex + 4);
}

const Gaffer::BoolPlug *FeatherDeformBarbs::cleanupBindAttributesPlug() const
{
    return getChild<BoolPlug>(g_firstPlugIndex + 4);
}

bool FeatherDeformBarbs::affectsProcessedObject(const Gaffer::Plug *input) const
{
    return Deformer::affectsProcessedObject(input) ||
           input == animatedShaftsPlug()->objectPlug() ||
           input == hairIdAttrNamePlug() ||
           input == shaftUpVectorPrimVarNamePlug() ||
           input == shaftPointOrientAttrNamePlug() ||
           input == cleanupBindAttributesPlug();
}

void FeatherDeformBarbs::hashProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    Deformer::hashProcessedObject(path, context, h);
    h.append(animatedShaftsPlug()->objectHash(path));
    // inPlug()->objectHash(path) is handled by Deformer::hashProcessedObject

    hairIdAttrNamePlug()->hash(h);
    shaftUpVectorPrimVarNamePlug()->hash(h);
    shaftPointOrientAttrNamePlug()->hash(h);
    cleanupBindAttributesPlug()->hash(h);
}

IECore::ConstObjectPtr FeatherDeformBarbs::computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject) const
{
    const CurvesPrimitive *inBarbsCurves = runTimeCast<const CurvesPrimitive>(inputObject);
    if (!inBarbsCurves)
    {
        return inputObject;
    }

    ConstObjectPtr animatedShaftsObj = animatedShaftsPlug()->object(path);
    const CurvesPrimitive *animatedShaftsCurves = runTimeCast<const CurvesPrimitive>(animatedShaftsObj.get());
    if (!animatedShaftsCurves)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "Animated shaft input is not a CurvesPrimitive or not found at expected path.");
        return inputObject; // Cannot deform without shafts
    }

    // Retrieve attribute names from plugs
    const std::string hairIdAttr = hairIdAttrNamePlug()->getValue();
    const std::string shaftUpVectorAttr = shaftUpVectorPrimVarNamePlug()->getValue();
    const std::string shaftOrientAttr = shaftPointOrientAttrNamePlug()->getValue();
    const bool cleanupAttrs = cleanupBindAttributesPlug()->getValue();

    if (hairIdAttr.empty())
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "Hair ID attribute name plug is empty.");
        return inputObject;
    }

    // Get P from input barbs (needed for topology and original points)
    const V3fVectorData *barbsPData = inBarbsCurves->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);
    if (!barbsPData)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "Input barbs are missing 'P' primitive variable.");
        return inputObject;
    }
    const std::vector<Imath::V3f> &barbsPOriginal = barbsPData->readable();

    // Get P from animated shafts
    const V3fVectorData *shaftsPData = animatedShaftsCurves->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);
    if (!shaftsPData)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "Animated shafts are missing 'P' primitive variable.");
        return inputObject;
    }
    const std::vector<Imath::V3f> &animatedShaftsP = shaftsPData->readable();

    // Get verticesPerCurve from barbs (for output topology and iteration)
    const IntVectorData *barbsVertsPerCurveData = inBarbsCurves->verticesPerCurve();
    if (!barbsVertsPerCurveData)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "Input barbs are missing verticesPerCurve data.");
        return inputObject;
    }
    const std::vector<int> &barbsVertsPerCurve = barbsVertsPerCurveData->readable();
    if (barbsVertsPerCurve.empty())
        return inputObject; // No curves

    // Get verticesPerCurve from animated shafts
    const IntVectorData *shaftsVertsPerCurveData = animatedShaftsCurves->verticesPerCurve();
    if (!shaftsVertsPerCurveData || shaftsVertsPerCurveData->readable().empty())
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "Animated shafts are missing verticesPerCurve data or have no curves.");
        return inputObject;
    }
    const std::vector<int> &animatedShaftsVertsPerCurve = shaftsVertsPerCurveData->readable();

    // --- Prepare output curves ---
    CurvesPrimitivePtr resultCurves = new CurvesPrimitive();
    resultCurves->setTopologyUnchecked(
        barbsVertsPerCurveData->copy(),
        inBarbsCurves->basis(),
        inBarbsCurves->periodic());
    V3fVectorDataPtr resultPData = new V3fVectorData();
    resultPData->writable().resize(barbsPOriginal.size()); // Pre-allocate for deformed points
    std::vector<Imath::V3f> &resultP = resultPData->writable();

    // Copy other existing primvars from barbs to result (will handle cleanup later)
    for (const auto &[name, primVar] : inBarbsCurves->variables)
    {
        if (name != "P") // P will be the new deformed points
        {
            resultCurves->variables[name] = primVar;
        }
    }

    // --- Retrieve animated shaft attributes ---
    const PrimitiveVariable::DataAndInterpolation shaftHairId = SceneAlgo::primitiveVariable(animatedShaftsCurves, hairIdAttr);
    if (!shaftHairId.data)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Missing required primitive variable '%s' on animated shafts.") % hairIdAttr);
        return inputObject;
    }

    const V3fVectorData *shaftUpVectorData = nullptr;
    if (!shaftUpVectorAttr.empty())
    {
        shaftUpVectorData = animatedShaftsCurves->variableData<V3fVectorData>(shaftUpVectorAttr, PrimitiveVariable::Uniform);
        if (!shaftUpVectorData)
            IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Animated shaft up-vector attribute '%s' not found or not V3fVectorData (Uniform).") % shaftUpVectorAttr);
    }

    const V4fVectorData *shaftOrientData = nullptr;
    PrimitiveVariable::Interpolation shaftOrientInterpolation = PrimitiveVariable::Invalid;
    if (!shaftOrientAttr.empty())
    {
        shaftOrientData = animatedShaftsCurves->variableData<V4fVectorData>(shaftOrientAttr, PrimitiveVariable::Vertex);
        if (shaftOrientData)
            shaftOrientInterpolation = PrimitiveVariable::Vertex;
        else
        {
            shaftOrientData = animatedShaftsCurves->variableData<V4fVectorData>(shaftOrientAttr, PrimitiveVariable::Uniform);
            if (shaftOrientData)
                shaftOrientInterpolation = PrimitiveVariable::Uniform;
        }
        if (!shaftOrientData)
            IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Animated shaft orientation attribute '%s' not found or not V4fVectorData (Vertex or Uniform).") % shaftOrientAttr);
    }

    // --- Prepare Animated Shaft Lookup Map ---
    std::map<int, AnimatedShaftInfo> intShaftLookup;
    std::map<std::string, AnimatedShaftInfo> stringShaftLookup;
    bool shaftsHaveIntHairId = false;
    bool shaftsHaveStringHairId = false;

    size_t currentAnimatedShaftPointOffset = 0;
    for (size_t i = 0; i < animatedShaftsVertsPerCurve.size(); ++i)
    {
        const int numShaftCVs = animatedShaftsVertsPerCurve[i];
        if (numShaftCVs <= 0)
            continue;

        AnimatedShaftInfo info;
        info.primitiveIndex = i;
        info.points = &animatedShaftsP;
        info.numPoints = numShaftCVs;
        info.upVector = nullptr;
        info.orientQuaternion = nullptr;
        info.orientInterpolation = PrimitiveVariable::Invalid;

        if (shaftUpVectorData)
        {
            if (shaftUpVectorData->interpolation() == PrimitiveVariable::Uniform && shaftUpVectorData->readable().size() == 1)
            {
                info.upVector = &(shaftUpVectorData->readable()[0]);
            }
            else if (shaftUpVectorData->interpolation() == PrimitiveVariable::Uniform && shaftUpVectorData->readable().size() > i)
            {
                info.upVector = &(shaftUpVectorData->readable()[i]);
            }
        }

        if (shaftOrientData)
        {
            info.orientInterpolation = shaftOrientInterpolation;
            if (shaftOrientInterpolation == PrimitiveVariable::Uniform)
            {
                if (shaftOrientData->readable().size() == 1)
                {
                    info.orientQuaternion = &(shaftOrientData->readable()[0]);
                }
                else if (shaftOrientData->readable().size() > i)
                {
                    info.orientQuaternion = &(shaftOrientData->readable()[i]);
                }
            }
        }

        // Get hairId for this animated shaft primitive
        if (const IntData *idData = runTimeCast<const IntData>(shaftHairId.data.get()))
        {
            shaftsHaveIntHairId = true;
            int id = 0;
            if (shaftHairId.interpolation == PrimitiveVariable::Uniform && idData->readable().size() == 1)
                id = idData->readable()[0];
            else if ((shaftHairId.interpolation == PrimitiveVariable::Uniform || shaftHairId.interpolation == PrimitiveVariable::Primitive) && idData->readable().size() > i)
                id = idData->readable()[i];
            else
                continue;
            intShaftLookup[id] = info;
        }
        else if (const StringData *idData = runTimeCast<const StringData>(shaftHairId.data.get()))
        {
            shaftsHaveStringHairId = true;
            std::string id;
            if (shaftHairId.interpolation == PrimitiveVariable::Uniform && idData->readable().size() == 1)
                id = idData->readable()[0];
            else if ((shaftHairId.interpolation == PrimitiveVariable::Uniform || shaftHairId.interpolation == PrimitiveVariable::Primitive) && idData->readable().size() > i)
                id = idData->readable()[i];
            else
                continue;
            stringShaftLookup[id] = info;
        }
        else if (const IntVectorData *idVecData = runTimeCast<const IntVectorData>(shaftHairId.data.get()))
        {
            shaftsHaveIntHairId = true;
            int id = 0;
            if ((shaftHairId.interpolation == PrimitiveVariable::Uniform || shaftHairId.interpolation == PrimitiveVariable::Primitive) && idVecData->readable().size() > i)
                id = idVecData->readable()[i];
            else if (shaftHairId.interpolation == PrimitiveVariable::Constant && !idVecData->readable().empty())
                id = idVecData->readable()[0];
            else
                continue;
            intShaftLookup[id] = info;
        }
        else if (const StringVectorData *idVecData = runTimeCast<const StringVectorData>(shaftHairId.data.get()))
        {
            shaftsHaveStringHairId = true;
            std::string id;
            if ((shaftHairId.interpolation == PrimitiveVariable::Uniform || shaftHairId.interpolation == PrimitiveVariable::Primitive) && idVecData->readable().size() > i)
                id = idVecData->readable()[i];
            else if (shaftHairId.interpolation == PrimitiveVariable::Constant && !idVecData->readable().empty())
                id = idVecData->readable()[0];
            else
                continue;
            stringShaftLookup[id] = info;
        }
        currentAnimatedShaftPointOffset += numShaftCVs;
    }
    if (!shaftsHaveIntHairId && !shaftsHaveStringHairId)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Animated shaft hair ID attribute '%s' has unsupported data type or interpolation.") % hairIdAttr);
        return inputObject;
    }

    // Pre-calculate animated shaft point offsets
    std::vector<size_t> animatedShaftCurveStartPointIndices(animatedShaftsVertsPerCurve.size());
    size_t runningAnimatedShaftPointTotal = 0;
    for (size_t i = 0; i < animatedShaftsVertsPerCurve.size(); ++i)
    {
        animatedShaftCurveStartPointIndices[i] = runningAnimatedShaftPointTotal;
        runningAnimatedShaftPointTotal += animatedShaftsVertsPerCurve[i];
    }

    // --- Retrieve all necessary bind attributes from input barbs ---
    // These are all expected to be Uniform
    const Data *bind_shaftHairId_rawData = inBarbsCurves->variableData<Data>("bind_shaftHairId", PrimitiveVariable::Uniform);
    const IntVectorData *bind_shaftPointId_data = inBarbsCurves->variableData<IntVectorData>("bind_shaftPointId", PrimitiveVariable::Uniform);
    const V3fVectorData *bind_shaftRest_P_data = inBarbsCurves->variableData<V3fVectorData>("bind_shaftRest_P", PrimitiveVariable::Uniform);
    const V3fVectorData *bind_shaftRest_T_data = inBarbsCurves->variableData<V3fVectorData>("bind_shaftRest_T", PrimitiveVariable::Uniform);
    const V3fVectorData *bind_shaftRest_B_data = inBarbsCurves->variableData<V3fVectorData>("bind_shaftRest_B", PrimitiveVariable::Uniform);
    const V3fVectorData *bind_shaftRest_N_data = inBarbsCurves->variableData<V3fVectorData>("bind_shaftRest_N", PrimitiveVariable::Uniform);
    const V3fVectorData *bind_barbRootOffsetLocal_data = inBarbsCurves->variableData<V3fVectorData>("bind_barbRootOffsetLocal", PrimitiveVariable::Uniform);

    if (!bind_shaftHairId_rawData || !bind_shaftPointId_data || !bind_shaftRest_P_data || !bind_shaftRest_T_data ||
        !bind_shaftRest_B_data || !bind_shaftRest_N_data || !bind_barbRootOffsetLocal_data)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "Missing one or more required 'bind_*' attributes on input barbs. Ensure barbs are processed by FeatherAttachBarbs first.");
        return inputObject;
    }

    const IntVectorData *bind_shaftHairId_int_data = runTimeCast<const IntVectorData>(bind_shaftHairId_rawData);
    const StringVectorData *bind_shaftHairId_str_data = runTimeCast<const StringVectorData>(bind_shaftHairId_rawData);
    if (!bind_shaftHairId_int_data && !bind_shaftHairId_str_data)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "'bind_shaftHairId' attribute has unsupported data type.");
        return inputObject;
    }

    // We also need the original barb "curveu" to find the barb's rest root point, if not stored directly.
    const FloatVectorData *barbParamUData = inBarbsCurves->variableData<FloatVectorData>(
        inBarbsCurves->variables.count("barbParamAttrName") ? inBarbsCurves->variables.at("barbParamAttrName").data->cast<StringData>()->readable() : "curveu",
        PrimitiveVariable::Vertex);
    if (!barbParamUData && inBarbsCurves->variables.count("barbParamAttrName"))
    {
        // Try the default name if the plug-specified one wasn't found in the above specific check
        barbParamUData = inBarbsCurves->variableData<FloatVectorData>("curveu", PrimitiveVariable::Vertex);
    }
    if (!barbParamUData)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "Missing 'curveu' (or equivalent specified by barbParamAttrName in FeatherAttachBarbs) attribute on input barbs needed to find barb root.");
        return inputObject;
    }
    const std::vector<float> &barbU = barbParamUData->readable();

    // --- Iterate through Input Barb Primitives ---
    size_t currentBarbPointOffset = 0;
    const size_t numBarbs = barbsVertsPerCurve.size();

    for (size_t barbIdx = 0; barbIdx < numBarbs; ++barbIdx)
    {
        const int numBarbCVs = barbsVertsPerCurve[barbIdx];
        if (numBarbCVs <= 0)
        {
            currentBarbPointOffset += numBarbCVs; // Should be 0, but for safety
            continue;
        }

        // a. Read bind_* attributes for the current barb
        int shaft_hair_id_int = 0;
        std::string shaft_hair_id_str;
        bool barbBoundToIntShaft = false;

        if (bind_shaftHairId_int_data && bind_shaftHairId_int_data->readable().size() > barbIdx)
        {
            shaft_hair_id_int = bind_shaftHairId_int_data->readable()[barbIdx];
            barbBoundToIntShaft = true;
        }
        else if (bind_shaftHairId_str_data && bind_shaftHairId_str_data->readable().size() > barbIdx)
        {
            shaft_hair_id_str = bind_shaftHairId_str_data->readable()[barbIdx];
        }
        else
        {
            IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Insufficient data in bind_shaftHairId for barb %d") % barbIdx);
            // Copy original points for this barb and continue
            for (int k = 0; k < numBarbCVs; ++k)
                resultP[currentBarbPointOffset + k] = barbsPOriginal[currentBarbPointOffset + k];
            currentBarbPointOffset += numBarbCVs;
            continue;
        }

        const int shaft_pt_id = bind_shaftPointId_data->readable()[barbIdx];
        const Imath::V3f bind_P_rest = bind_shaftRest_P_data->readable()[barbIdx];
        const Imath::V3f bind_T_rest = bind_shaftRest_T_data->readable()[barbIdx];
        const Imath::V3f bind_B_rest = bind_shaftRest_B_data->readable()[barbIdx];
        const Imath::V3f bind_N_rest = bind_shaftRest_N_data->readable()[barbIdx];
        const Imath::V3f barb_root_offset_local = bind_barbRootOffsetLocal_data->readable()[barbIdx];

        // b. Find corresponding animated shaft
        const AnimatedShaftInfo *foundAnimatedShaftInfo = nullptr;
        if (barbBoundToIntShaft && shaftsHaveIntHairId)
        {
            auto it = intShaftLookup.find(shaft_hair_id_int);
            if (it != intShaftLookup.end())
                foundAnimatedShaftInfo = &it->second;
        }
        else if (!barbBoundToIntShaft && shaftsHaveStringHairId)
        {
            auto it = stringShaftLookup.find(shaft_hair_id_str);
            if (it != stringShaftLookup.end())
                foundAnimatedShaftInfo = &it->second;
        }
        else
        {
            IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Hair ID type mismatch or missing lookup for barb %d with shaft ID %s%d.") % barbIdx % shaft_hair_id_str % shaft_hair_id_int);
            for (int k = 0; k < numBarbCVs; ++k)
                resultP[currentBarbPointOffset + k] = barbsPOriginal[currentBarbPointOffset + k];
            currentBarbPointOffset += numBarbCVs;
            continue;
        }

        if (!foundAnimatedShaftInfo)
        {
            IECore::msg(IECore::Msg::Detail, staticTypeName(), boost::format("Animated shaft not found for barb %d.") % barbIdx);
            for (int k = 0; k < numBarbCVs; ++k)
                resultP[currentBarbPointOffset + k] = barbsPOriginal[currentBarbPointOffset + k];
            currentBarbPointOffset += numBarbCVs;
            continue;
        }

        if (shaft_pt_id < 0 || static_cast<size_t>(shaft_pt_id) >= foundAnimatedShaftInfo->numPoints)
        {
            IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Invalid bind_shaftPointId %d for barb %d (animated shaft has %d points).") % shaft_pt_id % barbIdx % foundAnimatedShaftInfo->numPoints);
            for (int k = 0; k < numBarbCVs; ++k)
                resultP[currentBarbPointOffset + k] = barbsPOriginal[currentBarbPointOffset + k];
            currentBarbPointOffset += numBarbCVs;
            continue;
        }

        // c. Calculate Animated Shaft Attachment Frame
        const size_t animShaftCurveGlobalPointStartIndex = animatedShaftCurveStartPointIndices[foundAnimatedShaftInfo->primitiveIndex];
        const Imath::V3f P_attach_shaft_anim = (*foundAnimatedShaftInfo->points)[animShaftCurveGlobalPointStartIndex + shaft_pt_id];

        Frame animShaftFrame;
        animShaftFrame.P = P_attach_shaft_anim;
        bool animFrameCalculated = false;

        if (foundAnimatedShaftInfo->orientInterpolation != PrimitiveVariable::Invalid && shaftOrientData)
        {
            Imath::V4f qOrient_anim;
            if (foundAnimatedShaftInfo->orientInterpolation == PrimitiveVariable::Uniform && foundAnimatedShaftInfo->orientQuaternion)
            {
                qOrient_anim = *foundAnimatedShaftInfo->orientQuaternion;
                animShaftFrame.fromQuaternion(qOrient_anim);
                if (animShaftFrame.valid)
                    animFrameCalculated = true;
            }
            else if (foundAnimatedShaftInfo->orientInterpolation == PrimitiveVariable::Vertex)
            {
                const size_t shaftGlobalOrientIndex = animShaftCurveGlobalPointStartIndex + shaft_pt_id;
                if (shaftGlobalOrientIndex < shaftOrientData->readable().size())
                {
                    qOrient_anim = shaftOrientData->readable()[shaftGlobalOrientIndex];
                    animShaftFrame.fromQuaternion(qOrient_anim);
                    if (animShaftFrame.valid)
                        animFrameCalculated = true;
                }
            }
        }
        Imath::V3f currentAnimShaftTangent;
        if (!animFrameCalculated)
        {
            currentAnimShaftTangent = calculateTangent(*foundAnimatedShaftInfo->points, animShaftCurveGlobalPointStartIndex, foundAnimatedShaftInfo->numPoints, shaft_pt_id);
            if (currentAnimShaftTangent.length() < 0.5f)
                animShaftFrame.valid = false;
            else if (foundAnimatedShaftInfo->upVector)
            {
                animShaftFrame.fromUpVector(currentAnimShaftTangent, *foundAnimatedShaftInfo->upVector);
                if (animShaftFrame.valid)
                    animFrameCalculated = true;
            }
        }
        if (!animFrameCalculated && animShaftFrame.valid)
        {
            if (currentAnimShaftTangent.length() < 0.5f && !animFrameCalculated)
            {
                currentAnimShaftTangent = calculateTangent(*foundAnimatedShaftInfo->points, animShaftCurveGlobalPointStartIndex, foundAnimatedShaftInfo->numPoints, shaft_pt_id);
            }
            if (currentAnimShaftTangent.length() > 0.5f)
            {
                animShaftFrame.fromDefault(currentAnimShaftTangent);
                if (animShaftFrame.valid)
                    animFrameCalculated = true;
            }
            else
            {
                animShaftFrame.valid = false;
            }
        }
        if (!animShaftFrame.valid)
        {
            IECore::msg(IECore::Msg::Debug, staticTypeName(), boost::format("Could not calculate valid animated frame for shaft attach point on barb %d. Using default identity.") % barbIdx);
        }

        // d. Calculate New Barb Root Position
        Imath::M44f M_shaft_anim_rot;
        M_shaft_anim_rot.setValue(
            animShaftFrame.T.x, animShaftFrame.T.y, animShaftFrame.T.z, 0.0f,
            animShaftFrame.B.x, animShaftFrame.B.y, animShaftFrame.B.z, 0.0f,
            animShaftFrame.N.x, animShaftFrame.N.y, animShaftFrame.N.z, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f); // Rotational part only
        Imath::V3f P_barb_root_anim_offset_world;
        M_shaft_anim_rot.multVecMatrix(barb_root_offset_local, P_barb_root_anim_offset_world);
        Imath::V3f P_barb_root_anim = animShaftFrame.P + P_barb_root_anim_offset_world;

        // e. Transform Barb Points
        Imath::M44f M_shaft_rest;
        M_shaft_rest.setValue(
            bind_T_rest.x, bind_T_rest.y, bind_T_rest.z, 0.0f,
            bind_B_rest.x, bind_B_rest.y, bind_B_rest.z, 0.0f,
            bind_N_rest.x, bind_N_rest.y, bind_N_rest.z, 0.0f,
            bind_P_rest.x, bind_P_rest.y, bind_P_rest.z, 1.0f);
        Imath::M44f M_shaft_anim_full; // Full matrix including translation for P_attach_shaft_anim
        M_shaft_anim_full.setValue(
            animShaftFrame.T.x, animShaftFrame.T.y, animShaftFrame.T.z, 0.0f,
            animShaftFrame.B.x, animShaftFrame.B.y, animShaftFrame.B.z, 0.0f,
            animShaftFrame.N.x, animShaftFrame.N.y, animShaftFrame.N.z, 0.0f,
            animShaftFrame.P.x, animShaftFrame.P.y, animShaftFrame.P.z, 1.0f);

        Imath::M44f M_transform_points = M_shaft_anim_full * M_shaft_rest.inverse();

        // Find the original barb root point P_barb_root_rest_original
        float minU_orig = std::numeric_limits<float>::max();
        size_t barbRootLocalIndex_orig = 0;
        for (int k_orig = 0; k_orig < numBarbCVs; ++k_orig)
        {
            const size_t barbGlobalPointIndex_orig = currentBarbPointOffset + k_orig;
            if (barbU[barbGlobalPointIndex_orig] < minU_orig)
            {
                minU_orig = barbU[barbGlobalPointIndex_orig];
                barbRootLocalIndex_orig = k_orig;
            }
        }
        const Imath::V3f P_barb_root_rest_original = barbsPOriginal[currentBarbPointOffset + barbRootLocalIndex_orig];

        for (int k = 0; k < numBarbCVs; ++k)
        {
            const size_t pointGlobalIndex = currentBarbPointOffset + k;
            Imath::V3f p_k_orig_relative_to_its_root = barbsPOriginal[pointGlobalIndex] - P_barb_root_rest_original;
            Imath::V3f p_k_transformed_relative_to_its_root;
            // The matrix M_transform_points already includes the translation from the rest shaft attach to anim shaft attach.
            // We need to transform the original point from its position relative to bind_P_rest to its new position relative to animShaftFrame.P
            // M_transform_points transforms points from the space of M_shaft_rest to the space of M_shaft_anim_full
            // A point p relative to M_shaft_rest (i.e. p_local_rest = p_world - bind_P_rest, transformed by M_shaft_rest.inverse())
            // should become p_local_anim, which is then p_world_anim = (p_local_anim * M_shaft_anim_full) + animShaftFrame.P
            // Simpler: transform each barb point (which is in world space) by M_transform_points.
            // No, that's not right. M_transform_points moves the shaft frame. The barb is relative to that.

            // Each point on the barb is p_orig.
            // Its position relative to the shaft's REST attachment point is: p_orig - bind_P_rest.
            // We want its new position: P_attach_shaft_anim + ( (p_orig - bind_P_rest) * (RotationOf(M_shaft_anim) * RotationOf(M_shaft_rest.inverse())) )
            //   + (TranslationOf(M_shaft_anim) - TranslationOf(M_shaft_rest))
            // This is equivalent to: p_orig * M_transform_points if M_transform_points is (M_shaft_anim * M_shaft_rest.inverse())

            // Let's use the P_barb_root_anim logic:
            // Each point p_k_orig has a vector from P_barb_root_rest_original: delta_k = p_k_orig - P_barb_root_rest_original
            // This delta_k needs to be rotated by the change in orientation from the rest shaft frame to the animated shaft frame.
            Imath::M44f M_rot_rest_inv = M_shaft_rest.inverse();
            M_rot_rest_inv[3][0] = 0;
            M_rot_rest_inv[3][1] = 0;
            M_rot_rest_inv[3][2] = 0; // remove translation
            Imath::M44f M_rot_anim = M_shaft_anim_full;
            M_rot_anim[3][0] = 0;
            M_rot_anim[3][1] = 0;
            M_rot_anim[3][2] = 0; // remove translation
            Imath::M44f M_delta_rotation = M_rot_anim * M_rot_rest_inv;

            Imath::V3f delta_k_original = barbsPOriginal[pointGlobalIndex] - P_barb_root_rest_original;
            Imath::V3f delta_k_rotated;
            M_delta_rotation.multDirMatrix(delta_k_original, delta_k_rotated);
            resultP[pointGlobalIndex] = P_barb_root_anim + delta_k_rotated;
        }

        currentBarbPointOffset += numBarbCVs;
    }

    resultCurves->variables["P"] = PrimitiveVariable(PrimitiveVariable::Vertex, resultPData);

    if (cleanupAttrs)
    {
        const std::vector<std::string> bindAttrsToCleanup = {
            "bind_shaftHairId", "bind_shaftPointId", "bind_shaftRest_P", "bind_shaftRest_T",
            "bind_shaftRest_B", "bind_shaftRest_N", "bind_barbRootOffsetLocal"};
        for (const std::string &attrName : bindAttrsToCleanup)
        {
            auto it = resultCurves->variables.find(attrName);
            if (it != resultCurves->variables.end())
            {
                resultCurves->variables.erase(it);
            }
        }
        // Also remove the user-specified bindAttr if it was part of FeatherAttachCurves logic copied to barb
        // (Not directly part of this node's cleanup, but good to keep in mind if that primvar exists from upstream)
    }

    // --- Logic from feather_bin_plan.md FeatherDeformBarbs/Logic (`computeProcessedObject`) ---
    // 1. Validation (partially done)
    // 2. Prepare Data Structures (map hair_id to animated shaft data)
    // 3. Iterate through Input Barb Primitives
    //    a. Read bind_* attributes
    //    b. Find corresponding animated shaft
    //    c. Calculate Animated Shaft Attachment Frame (using same framing logic as FeatherAttachBarbs)
    //    d. Calculate New Barb Root Position
    //    e. Transform Barb Points
    // 4. Create Output CurvesPrimitive with deformed points
    // 5. If cleanupBindAttributes is true, remove bind_* attributes.

    // For now, return a copy of the input barbs
    return inBarbsCurves->copy();
}

// Bound related methods - stubs for now
bool FeatherDeformBarbs::affectsProcessedObjectBound(const Gaffer::Plug *input) const
{
    return Deformer::affectsProcessedObjectBound(input) ||
           input == animatedShaftsPlug()->boundPlug() || // If shaft bounds change, output bound might change
           input == hairIdAttrNamePlug() ||
           input == shaftUpVectorPrimVarNamePlug() ||
           input == shaftPointOrientAttrNamePlug();
    // cleanupBindAttributesPlug does not affect the bound itself
}

void FeatherDeformBarbs::hashProcessedObjectBound(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    Deformer::hashProcessedObjectBound(path, context, h);
    h.append(animatedShaftsPlug()->boundHash(path));
    // inPlug()->boundHash(path) is handled by Deformer::hashProcessedObjectBound

    hairIdAttrNamePlug()->hash(h);
    shaftUpVectorPrimVarNamePlug()->hash(h);
    shaftPointOrientAttrNamePlug()->hash(h);
}

Imath::Box3f FeatherDeformBarbs::computeProcessedObjectBound(const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject) const
{
    // Stub: For now, just return the input object's bound.
    // A more accurate bound would consider the animatedShafts' bound if barbs are strictly attached.
    // Or, it would be the bound of the actually deformed points.
    // Deformer::computeBoundFromInput() is a good default if we don't override hashProcessedObject and computeProcessedObject.
    // However, since we are providing custom computeProcessedObject, we should ideally compute a tight bound.

    // For simplicity, let's try to compute a bound based on the animated shafts, assuming barbs follow them closely.
    // If animatedShaftsPlug is not connected or has no object, fallback to input bound.
    Imath::Box3f animatedShaftsBound = animatedShaftsPlug()->bound(path);
    if (animatedShaftsBound.isEmpty())
    {
        if (inputObject)
        {
            return inputObject->bound();
        }
        return Imath::Box3f(); // Default empty bound
    }

    // A more robust approach would be to compute the bound of the output of computeProcessedObject.
    // This might be expensive if called often. Gaffer's Deformer base class often relies on
    // the user connecting a separate Bound node if very tight bounds are needed and computeBoundFromInput is too slow.

    // For now, if we have animated shafts, assume the barbs will be roughly within or around that bound.
    // This is a heuristic.
    Box3f inputBound = inputObject ? inputObject->bound() : Box3f();
    animatedShaftsBound.extendBy(inputBound); // Combine for safety, though ideally deformed points define the bound.
    return animatedShaftsBound;
}

void FeatherDeformBarbs::affectsProcessedObject(const Gaffer::Plug *input, IECore::MurmurHash &h) const
{
    Deformer::affectsProcessedObject(input, h);

    if (
        input == animatedShaftsPlug() ||
        input == hairIdAttrNamePlug() ||
        input == shaftUpVectorPrimVarNamePlug() ||
        input == shaftPointOrientAttrNamePlug() ||
        input == cleanupBindAttributesPlug())
    {
        h.append(GafferScene::ScenePlug::animatedObjectHashes(context));
    }
}

void FeatherDeformBarbs::hashProcessedObject(const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    Deformer::hashProcessedObject(context, h);

    inPlug()->objectPlug()->hash(h);
    animatedShaftsPlug()->objectPlug()->hash(h);
    hairIdAttrNamePlug()->hash(h);
    shaftUpVectorPrimVarNamePlug()->hash(h);
    shaftPointOrientAttrNamePlug()->hash(h);
    cleanupBindAttributesPlug()->hash(h);
}

Gaffer::ValuePlug::CachePolicy FeatherDeformBarbs::processedObjectComputeCachePolicy() const
{
    return Gaffer::ValuePlug::CachePolicy::Standard;
}

bool FeatherDeformBarbs::affectsProcessedObjectBound(const Gaffer::Plug *input) const
{
    return Deformer::affectsProcessedObjectBound(input) ||
           input == inPlug() ||
           input == animatedShaftsPlug() ||
           input == enabledPlug();
}

void FeatherDeformBarbs::hashProcessedObjectBound(const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    Deformer::hashProcessedObjectBound(context, h);
    inPlug()->boundHash(h);
    animatedShaftsPlug()->boundHash(h);
    // We don't strictly need to hash attribute name plugs or cleanup plug for the bound
    // unless our computeProcessedObjectBound logic directly uses them to alter the bound.
    // For a simple union-of-inputs bound, they are not needed.
}

Imath::Box3f FeatherDeformBarbs::computeProcessedObjectBound(const Gaffer::Context *context, const GafferScene::ScenePlug *parent, IECore::MurmurHash &h) const
{
    // For now, a simple heuristic: take the bound of the animated shafts
    // and the input object. This might be overly large or too small if
    // shafts are not representative of the deformation space.
    // A more accurate bound would require more processing.

    if (!enabledPlug()->getValue())
    {
        return inPlug()->bound();
    }

    Imath::Box3f result = inPlug()->bound();
    if (animatedShaftsPlug()->getInput<ScenePlug>())
    {
        result.extendBy(animatedShaftsPlug()->bound());
    }
    return result;
}

void FeatherDeformBarbs::computeProcessedObject(const Gaffer::Context *context, const GafferScene::ScenePlug *parent, IECore::ConstObjectPtr &inputObject, IECore::ObjectPtr &processedObject, IECore::MurmurHash &h) const
{
    // ... existing code ...
}