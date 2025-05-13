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

        Imath::M44f toMatrix() const
        {
            Imath::M44f matrix;
            matrix[0][0] = T.x; matrix[1][0] = T.y; matrix[2][0] = T.z; matrix[3][0] = 0;
            matrix[0][1] = B.x; matrix[1][1] = B.y; matrix[2][1] = B.z; matrix[3][1] = 0;
            matrix[0][2] = N.x; matrix[1][2] = N.y; matrix[2][2] = N.z; matrix[3][2] = 0;
            matrix[0][3] = P.x; matrix[1][3] = P.y; matrix[2][3] = P.z; matrix[3][3] = 1;
            return matrix;
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
        return inputObject;
    }

    const std::string hairIdAttr = hairIdAttrNamePlug()->getValue();
    const std::string shaftUpVectorAttr = shaftUpVectorPrimVarNamePlug()->getValue();
    const std::string shaftOrientAttr = shaftPointOrientAttrNamePlug()->getValue();
    const bool cleanupAttrs = cleanupBindAttributesPlug()->getValue();

    if (hairIdAttr.empty())
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "Hair ID attribute name plug is empty.");
        return inputObject;
    }

    const V3fVectorData *barbsPData = inBarbsCurves->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);
    if (!barbsPData)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "Input barbs are missing 'P' primitive variable.");
        return inputObject;
    }
    const std::vector<Imath::V3f> &barbsPOriginal = barbsPData->readable();

    const V3fVectorData *shaftsPData = animatedShaftsCurves->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);
    if (!shaftsPData)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "Animated shafts are missing 'P' primitive variable.");
        return inputObject;
    }
    const std::vector<Imath::V3f> &animatedShaftsP = shaftsPData->readable();

    const IntVectorData *barbsVertsPerCurveData = inBarbsCurves->verticesPerCurve();
    if (!barbsVertsPerCurveData || barbsVertsPerCurveData->readable().empty())
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "Input barbs are missing verticesPerCurve data or have no curves.");
        return inputObject;
    }
    const std::vector<int> &barbsVertsPerCurve = barbsVertsPerCurveData->readable();

    const IntVectorData *shaftsVertsPerCurveData = animatedShaftsCurves->verticesPerCurve();
    if (!shaftsVertsPerCurveData || shaftsVertsPerCurveData->readable().empty())
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "Animated shafts are missing verticesPerCurve data or have no curves.");
        return inputObject;
    }
    const std::vector<int> &animatedShaftsVertsPerCurve = shaftsVertsPerCurveData->readable();

    CurvesPrimitivePtr resultCurves = new CurvesPrimitive();
    resultCurves->setTopologyUnchecked(barbsVertsPerCurveData->copy(), inBarbsCurves->basis(), inBarbsCurves->periodic());
    V3fVectorDataPtr resultPData = new V3fVectorData();
    resultPData->writable().resize(barbsPOriginal.size());
    std::vector<Imath::V3f> &resultP = resultPData->writable();

    for (const auto &[name, primVar] : inBarbsCurves->variables)
    {
        if (name != "P") resultCurves->variables[name] = primVar;
    }

    // --- Retrieve animated shaft attributes (Directly) ---
    auto shaftHairIdIt = animatedShaftsCurves->variables.find(hairIdAttr);
    if (shaftHairIdIt == animatedShaftsCurves->variables.end() || !shaftHairIdIt->second.data)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Missing required primitive variable '%s' on animated shafts.") % hairIdAttr);
        return inputObject;
    }
    const Data* rawShaftHairIdDataPtr = shaftHairIdIt->second.data.get();
    PrimitiveVariable::Interpolation shaftHairIdInterpolation = shaftHairIdIt->second.interpolation;

    const V3fVectorData *shaftUpVectorData = nullptr;
    PrimitiveVariable::Interpolation shaftUpVectorInterpolation = PrimitiveVariable::Invalid;
    if (!shaftUpVectorAttr.empty())
    {
        auto it = animatedShaftsCurves->variables.find(shaftUpVectorAttr);
        if (it != animatedShaftsCurves->variables.end() && it->second.data)
        {
            shaftUpVectorData = runTimeCast<const V3fVectorData>(it->second.data.get());
            if(shaftUpVectorData) shaftUpVectorInterpolation = it->second.interpolation;
            else IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Animated shaft up-vector attribute '%s' not V3fVectorData.") % shaftUpVectorAttr);
        }
        else IECore::msg(IECore::Msg::Debug, staticTypeName(), boost::format("Animated shaft up-vector attribute '%s' not found.") % shaftUpVectorAttr);
    }

    const V4fVectorData *shaftOrientData = nullptr;
    PrimitiveVariable::Interpolation shaftOrientDataInterpolation = PrimitiveVariable::Invalid; // Renamed to avoid conflict
    if (!shaftOrientAttr.empty())
    {
        auto it = animatedShaftsCurves->variables.find(shaftOrientAttr);
        if (it != animatedShaftsCurves->variables.end() && it->second.data)
        {
            shaftOrientData = runTimeCast<const V4fVectorData>(it->second.data.get());
            if(shaftOrientData) shaftOrientDataInterpolation = it->second.interpolation;
            else IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Animated shaft orientation attribute '%s' not V4fVectorData.") % shaftOrientAttr);
        }
        else IECore::msg(IECore::Msg::Debug, staticTypeName(), boost::format("Animated shaft orientation attribute '%s' not found.") % shaftOrientAttr);
    }

    // --- Prepare Animated Shaft Lookup Map ---
    std::map<int, AnimatedShaftInfo> intShaftLookup;
    std::map<std::string, AnimatedShaftInfo> stringShaftLookup;
    bool shaftsHaveIntHairId = false;
    bool shaftsHaveStringHairId = false;

    for (size_t i = 0; i < animatedShaftsVertsPerCurve.size(); ++i)
    {
        const int numShaftCVs = animatedShaftsVertsPerCurve[i];
        if (numShaftCVs <= 0) continue;

        AnimatedShaftInfo info;
        info.primitiveIndex = i;
        info.points = &animatedShaftsP;
        info.numPoints = numShaftCVs;
        info.upVector = nullptr;
        info.orientQuaternion = nullptr;
        info.orientInterpolation = PrimitiveVariable::Invalid;

        if (shaftUpVectorData)
        {
            if (shaftUpVectorInterpolation == PrimitiveVariable::Constant && !shaftUpVectorData->readable().empty())
                info.upVector = &(shaftUpVectorData->readable()[0]);
            else if (shaftUpVectorInterpolation == PrimitiveVariable::Uniform && shaftUpVectorData->readable().size() > i)
                info.upVector = &(shaftUpVectorData->readable()[i]);
            // Note: Vertex interpolated up-vectors are not directly stored in AnimatedShaftInfo for now.
        }

        if (shaftOrientData)
        {
            info.orientInterpolation = shaftOrientDataInterpolation;
            if (shaftOrientDataInterpolation == PrimitiveVariable::Constant && !shaftOrientData->readable().empty())
                info.orientQuaternion = &(shaftOrientData->readable()[0]);
            else if (shaftOrientDataInterpolation == PrimitiveVariable::Uniform && shaftOrientData->readable().size() > i)
                info.orientQuaternion = &(shaftOrientData->readable()[i]);
            // Vertex data for orientation will be looked up directly using shaft_pt_id later
        }

        bool idFoundForShaft = false;
        if (const IntData *idData = runTimeCast<const IntData>(rawShaftHairIdDataPtr)) {
            shaftsHaveIntHairId = true; idFoundForShaft = true;
            int id = 0;
            if (shaftHairIdInterpolation == PrimitiveVariable::Constant && !idData->readable().empty()) id = idData->readable()[0];
            else if (shaftHairIdInterpolation == PrimitiveVariable::Uniform && idData->readable().size() > i) id = idData->readable()[i];
            else if (shaftHairIdInterpolation == PrimitiveVariable::Uniform && idData->readable().size() == 1 && i == 0) id = idData->readable()[0];
            else { idFoundForShaft = false; IECore::msg(IECore::Msg::Debug, staticTypeName(), "Animated Shaft IntData hairId interp/size mismatch."); }
            if(idFoundForShaft) intShaftLookup[id] = info;
        } else if (const StringData *idData = runTimeCast<const StringData>(rawShaftHairIdDataPtr)) {
            shaftsHaveStringHairId = true; idFoundForShaft = true;
            std::string id;
            if (shaftHairIdInterpolation == PrimitiveVariable::Constant && !idData->readable().empty()) id = idData->readable()[0];
            else if (shaftHairIdInterpolation == PrimitiveVariable::Uniform && idData->readable().size() > i) id = idData->readable()[i];
            else if (shaftHairIdInterpolation == PrimitiveVariable::Uniform && idData->readable().size() == 1 && i == 0) id = idData->readable()[0];
            else { idFoundForShaft = false; IECore::msg(IECore::Msg::Debug, staticTypeName(), "Animated Shaft StringData hairId interp/size mismatch."); }
            if(idFoundForShaft) stringShaftLookup[id] = info;
        } else if (const IntVectorData *idVecData = runTimeCast<const IntVectorData>(rawShaftHairIdDataPtr)) {
            shaftsHaveIntHairId = true; idFoundForShaft = true;
            int id = 0;
            if (shaftHairIdInterpolation == PrimitiveVariable::Constant && !idVecData->readable().empty()) id = idVecData->readable()[0];
            else if (shaftHairIdInterpolation == PrimitiveVariable::Uniform && idVecData->readable().size() > i) id = idVecData->readable()[i];
            else { idFoundForShaft = false; IECore::msg(IECore::Msg::Debug, staticTypeName(), "Animated Shaft IntVectorData hairId interp/size mismatch."); }
            if(idFoundForShaft) intShaftLookup[id] = info;
        } else if (const StringVectorData *idVecData = runTimeCast<const StringVectorData>(rawShaftHairIdDataPtr)) {
            shaftsHaveStringHairId = true; idFoundForShaft = true;
            std::string id;
            if (shaftHairIdInterpolation == PrimitiveVariable::Constant && !idVecData->readable().empty()) id = idVecData->readable()[0];
            else if (shaftHairIdInterpolation == PrimitiveVariable::Uniform && idVecData->readable().size() > i) id = idVecData->readable()[i];
            else { idFoundForShaft = false; IECore::msg(IECore::Msg::Debug, staticTypeName(), "Animated Shaft StringVectorData hairId interp/size mismatch."); }
            if(idFoundForShaft) stringShaftLookup[id] = info;
        }
        if (!idFoundForShaft) {
             IECore::msg(IECore::Msg::Debug, staticTypeName(), boost::format("Skipping animated shaft %d due to unhandled hairId type or structure.") % i);
        }
    }

    if (!shaftsHaveIntHairId && !shaftsHaveStringHairId)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Animated shaft hair ID attribute '%s' has unsupported data type or no valid entries found.") % hairIdAttr);
        return inputObject;
    }

    std::vector<size_t> animatedShaftCurveStartPointIndices(animatedShaftsVertsPerCurve.size());
    size_t runningAnimatedShaftPointTotal = 0;
    for (size_t i = 0; i < animatedShaftsVertsPerCurve.size(); ++i)
    {
        animatedShaftCurveStartPointIndices[i] = runningAnimatedShaftPointTotal;
        runningAnimatedShaftPointTotal += animatedShaftsVertsPerCurve[i];
    }

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

    const FloatVectorData *barbParamUData = nullptr;
    auto barbParamIt = inBarbsCurves->variables.find("barbParamAttrName"); // Check if the plug-specified name exists as a primvar
    if(barbParamIt != inBarbsCurves->variables.end()){
        if(const StringData* paramNameData = runTimeCast<const StringData>(barbParamIt->second.data.get())){
            barbParamUData = inBarbsCurves->variableData<FloatVectorData>(paramNameData->readable(), PrimitiveVariable::Vertex);
        }
    }
    if(!barbParamUData) { // Fallback to default "curveu"
        barbParamUData = inBarbsCurves->variableData<FloatVectorData>("curveu", PrimitiveVariable::Vertex);
    }
    if (!barbParamUData)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "Missing 'curveu' (or equivalent specified by barbParamAttrName in FeatherAttachBarbs) attribute on input barbs needed to find barb root.");
        return inputObject;
    }
    const std::vector<float> &barbU = barbParamUData->readable();

    size_t currentBarbPointOffset = 0;
    const size_t numBarbs = barbsVertsPerCurve.size();

    for (size_t barbIdx = 0; barbIdx < numBarbs; ++barbIdx)
    {
        const int numBarbCVs = barbsVertsPerCurve[barbIdx];
        if (numBarbCVs <= 0)
        {
            currentBarbPointOffset += numBarbCVs;
            continue;
        }

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
            for (int k = 0; k < numBarbCVs; ++k) resultP[currentBarbPointOffset + k] = barbsPOriginal[currentBarbPointOffset + k];
            currentBarbPointOffset += numBarbCVs;
            continue;
        }

        const int shaft_pt_id = bind_shaftPointId_data->readable()[barbIdx];
        const Imath::V3f bind_P_rest = bind_shaftRest_P_data->readable()[barbIdx];
        const Imath::V3f bind_T_rest = bind_shaftRest_T_data->readable()[barbIdx];
        const Imath::V3f bind_B_rest = bind_shaftRest_B_data->readable()[barbIdx];
        const Imath::V3f bind_N_rest = bind_shaftRest_N_data->readable()[barbIdx];
        const Imath::V3f barb_root_offset_local = bind_barbRootOffsetLocal_data->readable()[barbIdx];

        const AnimatedShaftInfo *foundAnimatedShaftInfo = nullptr;
        if (barbBoundToIntShaft && shaftsHaveIntHairId)
        {
            auto it = intShaftLookup.find(shaft_hair_id_int);
            if (it != intShaftLookup.end()) foundAnimatedShaftInfo = &it->second;
        }
        else if (!barbBoundToIntShaft && shaftsHaveStringHairId)
        {
            auto it = stringShaftLookup.find(shaft_hair_id_str);
            if (it != stringShaftLookup.end()) foundAnimatedShaftInfo = &it->second;
        }
        else
        {
            IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Hair ID type mismatch or missing lookup for barb %d with shaft ID %s%d.") % barbIdx % shaft_hair_id_str % shaft_hair_id_int);
            for (int k = 0; k < numBarbCVs; ++k) resultP[currentBarbPointOffset + k] = barbsPOriginal[currentBarbPointOffset + k];
            currentBarbPointOffset += numBarbCVs;
            continue;
        }

        if (!foundAnimatedShaftInfo)
        {
            IECore::msg(IECore::Msg::Detail, staticTypeName(), boost::format("Animated shaft not found for barb %d.") % barbIdx);
            for (int k = 0; k < numBarbCVs; ++k) resultP[currentBarbPointOffset + k] = barbsPOriginal[currentBarbPointOffset + k];
            currentBarbPointOffset += numBarbCVs;
            continue;
        }

        if (shaft_pt_id < 0 || static_cast<size_t>(shaft_pt_id) >= foundAnimatedShaftInfo->numPoints)
        {
            IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Invalid bind_shaftPointId %d for barb %d (animated shaft has %d points).") % shaft_pt_id % barbIdx % foundAnimatedShaftInfo->numPoints);
            for (int k = 0; k < numBarbCVs; ++k) resultP[currentBarbPointOffset + k] = barbsPOriginal[currentBarbPointOffset + k];
            currentBarbPointOffset += numBarbCVs;
            continue;
        }

        const size_t animShaftCurveGlobalPointStartIndex = animatedShaftCurveStartPointIndices[foundAnimatedShaftInfo->primitiveIndex];
        const Imath::V3f P_attach_shaft_anim = (*foundAnimatedShaftInfo->points)[animShaftCurveGlobalPointStartIndex + shaft_pt_id];

        Frame animShaftFrame;
        animShaftFrame.P = P_attach_shaft_anim;
        bool animFrameCalculated = false;

        if (foundAnimatedShaftInfo->orientInterpolation != PrimitiveVariable::Invalid && shaftOrientData) {
            Imath::V4f qOrient_anim; bool orientValid = false;
            if (foundAnimatedShaftInfo->orientInterpolation == PrimitiveVariable::Uniform && foundAnimatedShaftInfo->orientQuaternion) {
                qOrient_anim = *foundAnimatedShaftInfo->orientQuaternion; orientValid = true;
            } else if (foundAnimatedShaftInfo->orientInterpolation == PrimitiveVariable::Vertex) {
                const size_t shaftGlobalOrientIndex = animShaftCurveGlobalPointStartIndex + shaft_pt_id;
                if (shaftGlobalOrientIndex < shaftOrientData->readable().size()) {
                    qOrient_anim = shaftOrientData->readable()[shaftGlobalOrientIndex]; orientValid = true;
                }
            }
            if(orientValid) { animShaftFrame.fromQuaternion(qOrient_anim); if (animShaftFrame.valid) animFrameCalculated = true; }
        }

        Imath::V3f currentAnimShaftTangent;
        if (!animFrameCalculated) {
            currentAnimShaftTangent = calculateTangent(*foundAnimatedShaftInfo->points, animShaftCurveGlobalPointStartIndex, foundAnimatedShaftInfo->numPoints, shaft_pt_id);
            if (currentAnimShaftTangent.length() >= 0.5f) {
                if (foundAnimatedShaftInfo->upVector) {
                    animShaftFrame.fromUpVector(currentAnimShaftTangent, *foundAnimatedShaftInfo->upVector);
                    if (animShaftFrame.valid) animFrameCalculated = true;
                }
            } else animShaftFrame.valid = false;
        }

        if (!animFrameCalculated && animShaftFrame.valid) {
            if (currentAnimShaftTangent.length() < 0.5f) currentAnimShaftTangent = calculateTangent(*foundAnimatedShaftInfo->points, animShaftCurveGlobalPointStartIndex, foundAnimatedShaftInfo->numPoints, shaft_pt_id);
            if (currentAnimShaftTangent.length() >= 0.5f) {
                animShaftFrame.fromDefault(currentAnimShaftTangent);
                if (animShaftFrame.valid) animFrameCalculated = true;
            } else animShaftFrame.valid = false;
        }

        if (!animShaftFrame.valid) {
            IECore::msg(IECore::Msg::Debug, staticTypeName(), boost::format("Could not calculate valid animated frame for shaft attach point on barb %d. Using default identity.") % barbIdx);
        }

        Imath::M44f M_shaft_anim_rot;
        M_shaft_anim_rot.setValue(
            animShaftFrame.T.x, animShaftFrame.T.y, animShaftFrame.T.z, 0.0f,
            animShaftFrame.B.x, animShaftFrame.B.y, animShaftFrame.B.z, 0.0f,
            animShaftFrame.N.x, animShaftFrame.N.y, animShaftFrame.N.z, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f);
        Imath::V3f P_barb_root_anim_offset_world;
        M_shaft_anim_rot.multVecMatrix(barb_root_offset_local, P_barb_root_anim_offset_world);
        Imath::V3f P_barb_root_anim = animShaftFrame.P + P_barb_root_anim_offset_world;

        Imath::M44f M_shaft_rest;
        M_shaft_rest.setValue(
            bind_T_rest.x, bind_T_rest.y, bind_T_rest.z, 0.0f,
            bind_B_rest.x, bind_B_rest.y, bind_B_rest.z, 0.0f,
            bind_N_rest.x, bind_N_rest.y, bind_N_rest.z, 0.0f,
            bind_P_rest.x, bind_P_rest.y, bind_P_rest.z, 1.0f);

        float minU_orig = std::numeric_limits<float>::max();
        size_t barbRootLocalIndex_orig = 0;
        for (int k_orig = 0; k_orig < numBarbCVs; ++k_orig) {
            const size_t barbGlobalPointIndex_orig = currentBarbPointOffset + k_orig;
            if (barbGlobalPointIndex_orig >= barbU.size()) {
                 IECore::msg(IECore::Msg::Error, staticTypeName(), "barbU access out of bounds during root finding!");
                 // This barb is problematic, copy original points and skip transforming it
                 for (int k_copy = 0; k_copy < numBarbCVs; ++k_copy) resultP[currentBarbPointOffset + k_copy] = barbsPOriginal[currentBarbPointOffset + k_copy];
                 goto next_barb; // Skip to next iteration of outer loop
            }
            if (barbU[barbGlobalPointIndex_orig] < minU_orig) {
                minU_orig = barbU[barbGlobalPointIndex_orig];
                barbRootLocalIndex_orig = k_orig;
            }
        }
        const Imath::V3f P_barb_root_rest_original = barbsPOriginal[currentBarbPointOffset + barbRootLocalIndex_orig];

        Imath::M44f M_rot_rest_inv = M_shaft_rest.inverse();
        M_rot_rest_inv[3][0] = 0; M_rot_rest_inv[3][1] = 0; M_rot_rest_inv[3][2] = 0;
        Imath::M44f M_rot_anim = animShaftFrame.toMatrix(); // Get full matrix for anim frame
        M_rot_anim[3][0] = 0; M_rot_anim[3][1] = 0; M_rot_anim[3][2] = 0;
        Imath::M44f M_delta_rotation = M_rot_anim * M_rot_rest_inv;

        for (int k = 0; k < numBarbCVs; ++k)
        {
            const size_t pointGlobalIndex = currentBarbPointOffset + k;
            Imath::V3f delta_k_original = barbsPOriginal[pointGlobalIndex] - P_barb_root_rest_original;
            Imath::V3f delta_k_rotated;
            M_delta_rotation.multDirMatrix(delta_k_original, delta_k_rotated);
            resultP[pointGlobalIndex] = P_barb_root_anim + delta_k_rotated;
        }

        next_barb:; // Label for goto
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
            if (it != resultCurves->variables.end()) resultCurves->variables.erase(it);
        }
    }

    return resultCurves;
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

Imath::Box3f FeatherDeformBarbs::computeProcessedObjectBound(const ScenePath &path, const Gaffer::Context *context) const
{
    // Fetch the input object (barbs) using inPlug()
    IECore::ConstObjectPtr inputObject = inPlug()->object( path );

    Imath::Box3f animatedShaftsBound = animatedShaftsPlug()->bound(path); // This uses the context

    if (animatedShaftsBound.isEmpty())
    {
        if (inputObject)
        {
            return inputObject->bound();
        }
        return Imath::Box3f(); // Default empty bound
    }

    Imath::Box3f inputBoundValue = inputObject ? inputObject->bound() : Imath::Box3f();
    animatedShaftsBound.extendBy(inputBoundValue); // Combine for safety
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