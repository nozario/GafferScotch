#include "GafferScotch/FeatherAttachBarbs.h"

#include "GafferScene/SceneAlgo.h"

#include "IECoreScene/CurvesPrimitive.h"
// #include "IECoreScene/ObjectAlgo.h"
#include "IECore/NullObject.h"

// Potentially needed later for detailed logic
#include "IECoreScene/PrimitiveVariable.h"

using namespace Gaffer;
using namespace GafferScene;
using namespace GafferScotch;
using namespace IECore;
using namespace IECoreScene;

IE_CORE_DEFINERUNTIMETYPED(FeatherAttachBarbs);

size_t FeatherAttachBarbs::g_firstPlugIndex = 0;

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
        return Imath::V3f(0.0f); // Or handle as an error/default orientation
    }

    // Helper to get a tangent for a point on a curve
    Imath::V3f calculateTangent(const std::vector<Imath::V3f> &points, size_t curveStartIndex, size_t numCurvePoints, size_t pointLocalIndex)
    {
        if (numCurvePoints <= 1)
            return Imath::V3f(1, 0, 0); // Default for single point or empty curve

        const Imath::V3f &pt = points[curveStartIndex + pointLocalIndex];
        if (pointLocalIndex < numCurvePoints - 1)
        {
            return safeNormalize(points[curveStartIndex + pointLocalIndex + 1] - pt);
        }
        else // Last point on the curve
        {
            return safeNormalize(pt - points[curveStartIndex + pointLocalIndex - 1]);
        }
    }

    // Frame calculation struct/helper
    struct Frame
    {
        Imath::V3f P = Imath::V3f(0.0f);
        Imath::V3f T = Imath::V3f(1.0f, 0.0f, 0.0f);
        Imath::V3f B = Imath::V3f(0.0f, 1.0f, 0.0f);
        Imath::V3f N = Imath::V3f(0.0f, 0.0f, 1.0f);
        bool valid = true;

        void fromQuaternion(const Imath::V4f &q)
        {
            // Assuming q is a valid quaternion (normalized)
            Imath::M44f m = Imath::M44f().setQuat(Imath::Quatf(q.x, q.y, q.z, q.w)); // V4f is (v.x, v.y, v.z, s)
            T = safeNormalize(Imath::V3f(m[0][0], m[1][0], m[2][0]));                // X-axis
            B = safeNormalize(Imath::V3f(m[0][1], m[1][1], m[2][1]));                // Y-axis
            N = safeNormalize(Imath::V3f(m[0][2], m[1][2], m[2][2]));                // Z-axis
            // Check for degenerate frame
            if (T.length() < 0.5f || B.length() < 0.5f || N.length() < 0.5f || std::abs(T.dot(B)) > 0.1f || std::abs(T.dot(N)) > 0.1f || std::abs(B.dot(N)) > 0.1f)
            {
                valid = false;
            }
        }

        void fromUpVector(const Imath::V3f &tangent, const Imath::V3f &up)
        {
            T = safeNormalize(tangent);
            B = safeNormalize(T.cross(up));
            if (B.length() < 0.5f) // Tangent and up are parallel
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
            {
                refUp = Imath::V3f(1, 0, 0); // Use X if T is aligned with Y
            }
            B = safeNormalize(T.cross(refUp));
            if (B.length() < 0.5f)
            {
                valid = false;
                return;
            } // Should not happen if T is valid and refUp is chosen correctly
            N = safeNormalize(B.cross(T));
            if (N.length() < 0.5f)
                valid = false;
        }
    };

}

FeatherAttachBarbs::FeatherAttachBarbs(const std::string &name)
    : ObjectProcessor(name)
{
    storeIndexOfNextChild(g_firstPlugIndex);

    addChild(new ScenePlug("inShafts", Plug::In, Plug::Default & ~Plug::Serialisable));
    addChild(new ScenePlug("inBarbs", Plug::In, Plug::Default & ~Plug::Serialisable));
    inPlug()->setInput(inBarbsPlug()->objectPlug()); // Default pass-through for the main input object

    addChild(new StringPlug("hairIdAttrName", Plug::In, "hair_id"));
    addChild(new StringPlug("shaftPointIdAttrName", Plug::In, "shaft_pt"));
    addChild(new StringPlug("barbParamAttrName", Plug::In, "curveu"));
    addChild(new StringPlug("shaftUpVectorPrimVarName", Plug::In, ""));
    addChild(new StringPlug("shaftPointOrientAttrName", Plug::In, ""));

    // Fast pass-throughs for things we don't modify by default
    outPlug()->attributesPlug()->setInput(inPlug()->attributesPlug());
    outPlug()->transformPlug()->setInput(inPlug()->transformPlug());
    outPlug()->boundPlug()->setInput(inPlug()->boundPlug());
}

FeatherAttachBarbs::~FeatherAttachBarbs()
{
}

ScenePlug *FeatherAttachBarbs::inShaftsPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

const ScenePlug *FeatherAttachBarbs::inShaftsPlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex);
}

ScenePlug *FeatherAttachBarbs::inBarbsPlug()
{
    return getChild<ScenePlug>(g_firstPlugIndex + 1);
}

const ScenePlug *FeatherAttachBarbs::inBarbsPlug() const
{
    return getChild<ScenePlug>(g_firstPlugIndex + 1);
}

Gaffer::StringPlug *FeatherAttachBarbs::hairIdAttrNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 2);
}

const Gaffer::StringPlug *FeatherAttachBarbs::hairIdAttrNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 2);
}

Gaffer::StringPlug *FeatherAttachBarbs::shaftPointIdAttrNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 3);
}

const Gaffer::StringPlug *FeatherAttachBarbs::shaftPointIdAttrNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 3);
}

Gaffer::StringPlug *FeatherAttachBarbs::barbParamAttrNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 4);
}

const Gaffer::StringPlug *FeatherAttachBarbs::barbParamAttrNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 4);
}

Gaffer::StringPlug *FeatherAttachBarbs::shaftUpVectorPrimVarNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 5);
}

const Gaffer::StringPlug *FeatherAttachBarbs::shaftUpVectorPrimVarNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 5);
}

Gaffer::StringPlug *FeatherAttachBarbs::shaftPointOrientAttrNamePlug()
{
    return getChild<StringPlug>(g_firstPlugIndex + 6);
}

const Gaffer::StringPlug *FeatherAttachBarbs::shaftPointOrientAttrNamePlug() const
{
    return getChild<StringPlug>(g_firstPlugIndex + 6);
}

bool FeatherAttachBarbs::affectsProcessedObject(const Gaffer::Plug *input) const
{
    return ObjectProcessor::affectsProcessedObject(input) ||
           input == inShaftsPlug()->objectPlug() ||
           input == inShaftsPlug()->boundPlug() || // If shaft bounds change, output might change
           input == inBarbsPlug()->objectPlug() || // Main input object
           input == hairIdAttrNamePlug() ||
           input == shaftPointIdAttrNamePlug() ||
           input == barbParamAttrNamePlug() ||
           input == shaftUpVectorPrimVarNamePlug() ||
           input == shaftPointOrientAttrNamePlug();
}

void FeatherAttachBarbs::hashProcessedObject(const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h) const
{
    ObjectProcessor::hashProcessedObject(path, context, h);

    // Hash inputs specific to FeatherAttachBarbs
    h.append(inShaftsPlug()->objectHash(path)); // Hashing the specific object at the current path from inShafts
    // inBarbsPlug()->objectHash( path ) is implicitly included by ObjectProcessor::hashProcessedObject if inPlug() is connected to it.
    // Ensure we are hashing the correct input object if inPlug() is not directly inBarbsPlug()->objectPlug()
    ConstObjectPtr currentInputObject = inPlug()->object(path); // Renamed to avoid conflict
    if (currentInputObject)
    { // Ensure input object is valid before hashing to avoid errors
        currentInputObject->hash(h);
    }

    hairIdAttrNamePlug()->hash(h);
    shaftPointIdAttrNamePlug()->hash(h);
    barbParamAttrNamePlug()->hash(h);
    shaftUpVectorPrimVarNamePlug()->hash(h);
    shaftPointOrientAttrNamePlug()->hash(h);
}

IECore::ConstObjectPtr FeatherAttachBarbs::computeProcessedObject(const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject) const
{
    const CurvesPrimitive *inBarbsCurves = runTimeCast<const CurvesPrimitive>(inputObject);
    if (!inBarbsCurves)
    {
        return inputObject;
    }

    ConstObjectPtr shaftsObject = inShaftsPlug()->object(path);
    const CurvesPrimitive *inShaftsCurves = runTimeCast<const CurvesPrimitive>(shaftsObject.get());

    if (!inShaftsCurves)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "Shaft input is not a CurvesPrimitive or not found at expected path.");
        return inputObject;
    }

    const std::string hairIdAttr = hairIdAttrNamePlug()->getValue();
    const std::string shaftPointIdAttr = shaftPointIdAttrNamePlug()->getValue();
    const std::string barbParamAttr = barbParamAttrNamePlug()->getValue();
    const std::string shaftUpVectorAttr = shaftUpVectorPrimVarNamePlug()->getValue();
    const std::string shaftOrientAttr = shaftPointOrientAttrNamePlug()->getValue();

    if (hairIdAttr.empty() || shaftPointIdAttr.empty() || barbParamAttr.empty())
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "One or more critical attribute name plugs are empty.");
        return inputObject;
    }

    const V3fVectorData *barbsPData = inBarbsCurves->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);
    if (!barbsPData) return inputObject;
    const std::vector<Imath::V3f> &barbsP = barbsPData->readable();

    const V3fVectorData *shaftsPData = inShaftsCurves->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);
    if (!shaftsPData) return inputObject;
    const std::vector<Imath::V3f> &shaftsP = shaftsPData->readable();

    const IntVectorData *barbsVertsPerCurveData = inBarbsCurves->verticesPerCurve();
    if (!barbsVertsPerCurveData || barbsVertsPerCurveData->readable().empty()) return inputObject;
    const std::vector<int> &barbsVertsPerCurve = barbsVertsPerCurveData->readable();

    const IntVectorData *shaftsVertsPerCurveData = inShaftsCurves->verticesPerCurve();
    if (!shaftsVertsPerCurveData || shaftsVertsPerCurveData->readable().empty()) return inputObject;
    const std::vector<int> &shaftsVertsPerCurve = shaftsVertsPerCurveData->readable();

    CurvesPrimitivePtr resultCurves = new CurvesPrimitive();
    resultCurves->setTopologyUnchecked(barbsVertsPerCurveData->copy(), inBarbsCurves->basis(), inBarbsCurves->periodic());
    for (const auto &[name, primVar] : inBarbsCurves->variables)
    {
        if (name != "P") resultCurves->variables[name] = primVar;
    }
    resultCurves->variables["P"] = PrimitiveVariable(PrimitiveVariable::Vertex, barbsPData->copy());

    // --- Shaft Attribute Fetching (Directly) ---
    auto shaftHairIdIt = inShaftsCurves->variables.find(hairIdAttr);
    if (shaftHairIdIt == inShaftsCurves->variables.end() || !shaftHairIdIt->second.data)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Missing required primitive variable '%s' on input shafts.") % hairIdAttr);
        return inputObject;
    }
    const Data* rawShaftHairIdDataPtr = shaftHairIdIt->second.data.get();
    PrimitiveVariable::Interpolation shaftHairIdInterpolation = shaftHairIdIt->second.interpolation;


    const V3fVectorData *shaftUpVectorData = nullptr;
    PrimitiveVariable::Interpolation shaftUpVectorInterpolation = PrimitiveVariable::Invalid;
    if (!shaftUpVectorAttr.empty())
    {
        auto it = inShaftsCurves->variables.find(shaftUpVectorAttr);
        if (it != inShaftsCurves->variables.end() && it->second.data) {
            shaftUpVectorData = runTimeCast<const V3fVectorData>(it->second.data.get());
            if(shaftUpVectorData) shaftUpVectorInterpolation = it->second.interpolation;
            else IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Shaft up-vector attribute '%s' not V3fVectorData.") % shaftUpVectorAttr);
        } else {
             IECore::msg(IECore::Msg::Debug, staticTypeName(), boost::format("Shaft up-vector attribute '%s' not found.") % shaftUpVectorAttr);
        }
    }

    const V4fVectorData *shaftOrientData = nullptr;
    PrimitiveVariable::Interpolation shaftOrientInterpolation = PrimitiveVariable::Invalid;
    if (!shaftOrientAttr.empty())
    {
        auto it = inShaftsCurves->variables.find(shaftOrientAttr);
        if (it != inShaftsCurves->variables.end() && it->second.data) {
            shaftOrientData = runTimeCast<const V4fVectorData>(it->second.data.get());
            if(shaftOrientData) shaftOrientInterpolation = it->second.interpolation;
            else IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Shaft orientation attribute '%s' not V4fVectorData.") % shaftOrientAttr);
        } else {
            IECore::msg(IECore::Msg::Debug, staticTypeName(), boost::format("Shaft orientation attribute '%s' not found.") % shaftOrientAttr);
        }
    }

    std::map<int, ShaftInfo> intShaftLookup;
    std::map<std::string, ShaftInfo> stringShaftLookup;
    bool shaftsHaveIntHairId = false;
    bool shaftsHaveStringHairId = false;
    size_t currentShaftPrimitivePointOffset = 0; // Renamed to avoid conflict

    for (size_t i = 0; i < shaftsVertsPerCurve.size(); ++i)
    {
        const int numShaftCVs = shaftsVertsPerCurve[i];
        if (numShaftCVs <= 0) continue;

        ShaftInfo info;
        info.primitiveIndex = i;
        info.points = &shaftsP;
        info.numPoints = numShaftCVs;
        info.upVector = nullptr;
        info.orientQuaternion = nullptr;
        info.orientInterpolation = PrimitiveVariable::Invalid;

        if (shaftUpVectorData) {
            if (shaftUpVectorInterpolation == PrimitiveVariable::Constant && !shaftUpVectorData->readable().empty())
                info.upVector = &(shaftUpVectorData->readable()[0]);
            else if (shaftUpVectorInterpolation == PrimitiveVariable::Uniform && shaftUpVectorData->readable().size() > i)
                info.upVector = &(shaftUpVectorData->readable()[i]);
        }

        if (shaftOrientData) {
            info.orientInterpolation = shaftOrientInterpolation;
            if (shaftOrientInterpolation == PrimitiveVariable::Constant && !shaftOrientData->readable().empty())
                info.orientQuaternion = &(shaftOrientData->readable()[0]);
            else if (shaftOrientInterpolation == PrimitiveVariable::Uniform && shaftOrientData->readable().size() > i)
                info.orientQuaternion = &(shaftOrientData->readable()[i]);
            // Vertex data handled per-point later
        }

        bool idFoundForShaft = false;
        if (const IntData *idData = runTimeCast<const IntData>(rawShaftHairIdDataPtr)) {
            shaftsHaveIntHairId = true; idFoundForShaft = true;
            int id = 0;
            if (shaftHairIdInterpolation == PrimitiveVariable::Constant && !idData->readable().empty()) id = idData->readable()[0];
            else if (shaftHairIdInterpolation == PrimitiveVariable::Uniform && idData->readable().size() > i) id = idData->readable()[i];
            else if (shaftHairIdInterpolation == PrimitiveVariable::Uniform && idData->readable().size()==1 && i==0) id = idData->readable()[0]; // Special case: Uniform with 1 element for single prim
            else { idFoundForShaft = false; IECore::msg(IECore::Msg::Debug, staticTypeName(), "Shaft IntData hairId interp/size mismatch."); }
            if(idFoundForShaft) intShaftLookup[id] = info;
        } else if (const StringData *idData = runTimeCast<const StringData>(rawShaftHairIdDataPtr)) {
            shaftsHaveStringHairId = true; idFoundForShaft = true;
            std::string id;
            if (shaftHairIdInterpolation == PrimitiveVariable::Constant && !idData->readable().empty()) id = idData->readable()[0];
            else if (shaftHairIdInterpolation == PrimitiveVariable::Uniform && idData->readable().size() > i) id = idData->readable()[i];
            else if (shaftHairIdInterpolation == PrimitiveVariable::Uniform && idData->readable().size()==1 && i==0) id = idData->readable()[0];
            else { idFoundForShaft = false; IECore::msg(IECore::Msg::Debug, staticTypeName(), "Shaft StringData hairId interp/size mismatch."); }
            if(idFoundForShaft) stringShaftLookup[id] = info;
        } else if (const IntVectorData *idVecData = runTimeCast<const IntVectorData>(rawShaftHairIdDataPtr)) {
            shaftsHaveIntHairId = true; idFoundForShaft = true;
            int id = 0;
            if (shaftHairIdInterpolation == PrimitiveVariable::Constant && !idVecData->readable().empty()) id = idVecData->readable()[0];
            else if (shaftHairIdInterpolation == PrimitiveVariable::Uniform && idVecData->readable().size() > i) id = idVecData->readable()[i];
             else { idFoundForShaft = false; IECore::msg(IECore::Msg::Debug, staticTypeName(), "Shaft IntVectorData hairId interp/size mismatch."); }
            if(idFoundForShaft) intShaftLookup[id] = info;
        } else if (const StringVectorData *idVecData = runTimeCast<const StringVectorData>(rawShaftHairIdDataPtr)) {
            shaftsHaveStringHairId = true; idFoundForShaft = true;
            std::string id;
            if (shaftHairIdInterpolation == PrimitiveVariable::Constant && !idVecData->readable().empty()) id = idVecData->readable()[0];
            else if (shaftHairIdInterpolation == PrimitiveVariable::Uniform && idVecData->readable().size() > i) id = idVecData->readable()[i];
            else { idFoundForShaft = false; IECore::msg(IECore::Msg::Debug, staticTypeName(), "Shaft StringVectorData hairId interp/size mismatch."); }
            if(idFoundForShaft) stringShaftLookup[id] = info;
        }

        if (!idFoundForShaft) {
             IECore::msg(IECore::Msg::Debug, staticTypeName(), boost::format("Skipping shaft %d due to unhandled hairId type or structure.") % i);
        }
        currentShaftPrimitivePointOffset += numShaftCVs;
    }

    if (!shaftsHaveIntHairId && !shaftsHaveStringHairId) {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Shaft hair ID attribute '%s' has unsupported data type or no valid entries found.") % hairIdAttr);
        return inputObject;
    }

    const size_t numBarbs = barbsVertsPerCurve.size();
    DataPtr bind_shaftHairId_data;
    if (shaftsHaveIntHairId) bind_shaftHairId_data = new IntVectorData();
    else bind_shaftHairId_data = new StringVectorData();
    std::vector<int> *bind_shaftHairId_int_writable = shaftsHaveIntHairId ? &(static_cast<IntVectorData *>(bind_shaftHairId_data.get())->writable()) : nullptr;
    std::vector<std::string> *bind_shaftHairId_str_writable = shaftsHaveStringHairId ? &(static_cast<StringVectorData *>(bind_shaftHairId_data.get())->writable()) : nullptr;
    if (bind_shaftHairId_int_writable) bind_shaftHairId_int_writable->reserve(numBarbs);
    if (bind_shaftHairId_str_writable) bind_shaftHairId_str_writable->reserve(numBarbs);

    IntVectorDataPtr bind_shaftPointId_data = new IntVectorData();
    bind_shaftPointId_data->writable().reserve(numBarbs);
    V3fVectorDataPtr bind_shaftRest_P_data = new V3fVectorData();
    bind_shaftRest_P_data->writable().reserve(numBarbs);
    V3fVectorDataPtr bind_shaftRest_T_data = new V3fVectorData();
    bind_shaftRest_T_data->writable().reserve(numBarbs);
    V3fVectorDataPtr bind_shaftRest_B_data = new V3fVectorData();
    bind_shaftRest_B_data->writable().reserve(numBarbs);
    V3fVectorDataPtr bind_shaftRest_N_data = new V3fVectorData();
    bind_shaftRest_N_data->writable().reserve(numBarbs);
    V3fVectorDataPtr bind_barbRootOffsetLocal_data = new V3fVectorData();
    bind_barbRootOffsetLocal_data->writable().reserve(numBarbs);

    std::vector<size_t> shaftCurveStartPointIndices(shaftsVertsPerCurve.size());
    size_t runningShaftPointTotal = 0;
    for (size_t i = 0; i < shaftsVertsPerCurve.size(); ++i) {
        shaftCurveStartPointIndices[i] = runningShaftPointTotal;
        runningShaftPointTotal += shaftsVertsPerCurve[i];
    }

    // --- Barb Attribute Fetching (Directly) ---
    auto barbHairIdIt = inBarbsCurves->variables.find(hairIdAttr);
    if (barbHairIdIt == inBarbsCurves->variables.end() || !barbHairIdIt->second.data) {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Missing required primitive variable '%s' on input barbs (for hair ID).") % hairIdAttr);
        // Fill defaults and return
        for (size_t barbIdx = 0; barbIdx < numBarbs; ++barbIdx) {
            if (bind_shaftHairId_int_writable) bind_shaftHairId_int_writable->push_back(0); else if (bind_shaftHairId_str_writable) bind_shaftHairId_str_writable->push_back("");
            bind_shaftPointId_data->writable().push_back(-1); bind_shaftRest_P_data->writable().push_back(Imath::V3f(0)); bind_shaftRest_T_data->writable().push_back(Imath::V3f(1,0,0)); bind_shaftRest_B_data->writable().push_back(Imath::V3f(0,1,0)); bind_shaftRest_N_data->writable().push_back(Imath::V3f(0,0,1)); bind_barbRootOffsetLocal_data->writable().push_back(Imath::V3f(0));
        }
        // Add to resultCurves...
        return resultCurves; // Simplified early exit
    }
    const Data* rawBarbHairIdDataPtr = barbHairIdIt->second.data.get();
    PrimitiveVariable::Interpolation barbHairIdInterpolation = barbHairIdIt->second.interpolation;

    auto barbShaftPointIdIt = inBarbsCurves->variables.find(shaftPointIdAttr);
    if (barbShaftPointIdIt == inBarbsCurves->variables.end() || !barbShaftPointIdIt->second.data) {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Missing required primitive variable '%s' on input barbs (for shaft point ID).") % shaftPointIdAttr);
        // Fill defaults and return
        return resultCurves; // Simplified early exit
    }
    const Data* rawBarbShaftPointIdDataPtr = barbShaftPointIdIt->second.data.get();
    PrimitiveVariable::Interpolation barbShaftPointIdInterpolation = barbShaftPointIdIt->second.interpolation;

    auto barbParamAttrIt = inBarbsCurves->variables.find(barbParamAttr);
    if (barbParamAttrIt == inBarbsCurves->variables.end() || !barbParamAttrIt->second.data ||
        barbParamAttrIt->second.interpolation != PrimitiveVariable::Vertex) {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Missing or non-Vertex primitive variable '%s' on input barbs.") % barbParamAttr);
        return resultCurves; // Simplified early exit
    }
    const FloatVectorData* barbParamUData = runTimeCast<const FloatVectorData>(barbParamAttrIt->second.data.get());
    if (!barbParamUData) {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Primitive variable '%s' is not FloatVectorData.") % barbParamAttr);
        return resultCurves;
    }
    const std::vector<float> &barbU = barbParamUData->readable();


    size_t currentBarbPointOffset = 0;
    for (size_t barbIdx = 0; barbIdx < numBarbs; ++barbIdx) {
        const int numBarbCVs = barbsVertsPerCurve[barbIdx];
        auto fill_defaults_and_continue = [&]() {
            if (bind_shaftHairId_int_writable) bind_shaftHairId_int_writable->push_back(0); else if (bind_shaftHairId_str_writable) bind_shaftHairId_str_writable->push_back("");
            bind_shaftPointId_data->writable().push_back(-1); bind_shaftRest_P_data->writable().push_back(Imath::V3f(0)); bind_shaftRest_T_data->writable().push_back(Imath::V3f(1,0,0)); bind_shaftRest_B_data->writable().push_back(Imath::V3f(0,1,0)); bind_shaftRest_N_data->writable().push_back(Imath::V3f(0,0,1)); bind_barbRootOffsetLocal_data->writable().push_back(Imath::V3f(0));
            currentBarbPointOffset += numBarbCVs;
        };

        if (numBarbCVs <= 0) {
            fill_defaults_and_continue();
            continue;
        }

        int barb_hair_id_int = 0;
        std::string barb_hair_id_str;
        bool barbHasIntHairId = false;
        bool barbAttrsValid = true;

        if (const IntData *idData = runTimeCast<const IntData>(rawBarbHairIdDataPtr)) {
            barbHasIntHairId = true;
            if (barbHairIdInterpolation == PrimitiveVariable::Constant && !idData->readable().empty()) barb_hair_id_int = idData->readable()[0];
            else if (barbHairIdInterpolation == PrimitiveVariable::Uniform && idData->readable().size() > barbIdx) barb_hair_id_int = idData->readable()[barbIdx];
            else if (barbHairIdInterpolation == PrimitiveVariable::Uniform && idData->readable().size()==1 && barbIdx==0) barb_hair_id_int = idData->readable()[0];
            else { barbAttrsValid = false; }
        } else if (const StringData *idData = runTimeCast<const StringData>(rawBarbHairIdDataPtr)) {
            if (barbHairIdInterpolation == PrimitiveVariable::Constant && !idData->readable().empty()) barb_hair_id_str = idData->readable()[0];
            else if (barbHairIdInterpolation == PrimitiveVariable::Uniform && idData->readable().size() > barbIdx) barb_hair_id_str = idData->readable()[barbIdx];
            else if (barbHairIdInterpolation == PrimitiveVariable::Uniform && idData->readable().size()==1 && barbIdx==0) barb_hair_id_str = idData->readable()[0];
            else { barbAttrsValid = false; }
        } else if (const IntVectorData *idVecData = runTimeCast<const IntVectorData>(rawBarbHairIdDataPtr)) {
            barbHasIntHairId = true;
            if (barbHairIdInterpolation == PrimitiveVariable::Constant && !idVecData->readable().empty()) barb_hair_id_int = idVecData->readable()[0];
            else if (barbHairIdInterpolation == PrimitiveVariable::Uniform && idVecData->readable().size() > barbIdx) barb_hair_id_int = idVecData->readable()[barbIdx];
            else { barbAttrsValid = false; }
        } else if (const StringVectorData *idVecData = runTimeCast<const StringVectorData>(rawBarbHairIdDataPtr)) {
            if (barbHairIdInterpolation == PrimitiveVariable::Constant && !idVecData->readable().empty()) barb_hair_id_str = idVecData->readable()[0];
            else if (barbHairIdInterpolation == PrimitiveVariable::Uniform && idVecData->readable().size() > barbIdx) barb_hair_id_str = idVecData->readable()[barbIdx];
            else { barbAttrsValid = false; }
        } else { barbAttrsValid = false; }

        if (!barbAttrsValid) {
            IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Barb %d: HairID attribute '%s' has unsupported data type or interp/size mismatch.") % barbIdx % hairIdAttr);
            fill_defaults_and_continue(); continue;
        }
        barbAttrsValid = true; // Reset for next attribute

        int shaft_pt_id = -1;
        if (const IntData *idData = runTimeCast<const IntData>(rawBarbShaftPointIdDataPtr)) {
            if (barbShaftPointIdInterpolation == PrimitiveVariable::Constant && !idData->readable().empty()) shaft_pt_id = idData->readable()[0];
            else if (barbShaftPointIdInterpolation == PrimitiveVariable::Uniform && idData->readable().size() > barbIdx) shaft_pt_id = idData->readable()[barbIdx];
            else if (barbShaftPointIdInterpolation == PrimitiveVariable::Uniform && idData->readable().size()==1 && barbIdx==0) shaft_pt_id = idData->readable()[0];
            else { barbAttrsValid = false; }
        } else if (const IntVectorData *idVecData = runTimeCast<const IntVectorData>(rawBarbShaftPointIdDataPtr)) {
            if (barbShaftPointIdInterpolation == PrimitiveVariable::Constant && !idVecData->readable().empty()) shaft_pt_id = idVecData->readable()[0];
            else if (barbShaftPointIdInterpolation == PrimitiveVariable::Uniform && idVecData->readable().size() > barbIdx) shaft_pt_id = idVecData->readable()[barbIdx];
            else { barbAttrsValid = false; }
        } else { barbAttrsValid = false; }

        if (!barbAttrsValid) {
            IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Barb %d: ShaftPointID attribute '%s' has unsupported data type or interp/size mismatch.") % barbIdx % shaftPointIdAttr);
            fill_defaults_and_continue(); continue;
        }

        const ShaftInfo *foundShaftInfo = nullptr;
        if (barbHasIntHairId && shaftsHaveIntHairId) {
            auto it = intShaftLookup.find(barb_hair_id_int);
            if (it != intShaftLookup.end()) foundShaftInfo = &it->second;
        } else if (!barbHasIntHairId && shaftsHaveStringHairId) {
            auto it = stringShaftLookup.find(barb_hair_id_str);
            if (it != stringShaftLookup.end()) foundShaftInfo = &it->second;
        } else if (barbHasIntHairId != shaftsHaveIntHairId) {
            IECore::msg(IECore::Msg::Warning, staticTypeName(), "Hair ID type mismatch between barbs and shafts.");
            fill_defaults_and_continue(); continue;
        }

        if (!foundShaftInfo) {
            IECore::msg(IECore::Msg::Detail, staticTypeName(), boost::format("Shaft with hair_id not found for barb %d.") % barbIdx);
            fill_defaults_and_continue(); continue;
        }

        if (shaft_pt_id < 0 || static_cast<size_t>(shaft_pt_id) >= foundShaftInfo->numPoints) {
            IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Invalid shaft_pt_id %d for barb %d (shaft has %d points).") % shaft_pt_id % barbIdx % foundShaftInfo->numPoints);
            fill_defaults_and_continue(); continue;
        }

        const size_t shaftCurveGlobalPointStartIndex = shaftCurveStartPointIndices[foundShaftInfo->primitiveIndex];
        const Imath::V3f shaftAttachP = (*foundShaftInfo->points)[shaftCurveGlobalPointStartIndex + shaft_pt_id];

        Frame shaftFrame;
        shaftFrame.P = shaftAttachP;
        bool frameCalculated = false;

        if (foundShaftInfo->orientInterpolation != PrimitiveVariable::Invalid && shaftOrientData) {
            Imath::V4f qOrient; bool orientValid = false;
            if (foundShaftInfo->orientInterpolation == PrimitiveVariable::Uniform && foundShaftInfo->orientQuaternion) {
                qOrient = *foundShaftInfo->orientQuaternion; orientValid = true;
            } else if (foundShaftInfo->orientInterpolation == PrimitiveVariable::Vertex) {
                const size_t shaftGlobalOrientIndex = shaftCurveGlobalPointStartIndex + shaft_pt_id;
                if (shaftGlobalOrientIndex < shaftOrientData->readable().size()) {
                    qOrient = shaftOrientData->readable()[shaftGlobalOrientIndex]; orientValid = true;
                }
            }
            if(orientValid) { shaftFrame.fromQuaternion(qOrient); if (shaftFrame.valid) frameCalculated = true; }
        }

        Imath::V3f currentShaftTangent;
        if (!frameCalculated) {
            currentShaftTangent = calculateTangent(*foundShaftInfo->points, shaftCurveGlobalPointStartIndex, foundShaftInfo->numPoints, shaft_pt_id);
            if (currentShaftTangent.length() >= 0.5f) {
                if (foundShaftInfo->upVector) {
                    shaftFrame.fromUpVector(currentShaftTangent, *foundShaftInfo->upVector);
                    if (shaftFrame.valid) frameCalculated = true;
                }
            } else shaftFrame.valid = false;
        }

        if (!frameCalculated && shaftFrame.valid) {
            if (currentShaftTangent.length() < 0.5f) currentShaftTangent = calculateTangent(*foundShaftInfo->points, shaftCurveGlobalPointStartIndex, foundShaftInfo->numPoints, shaft_pt_id);
            if (currentShaftTangent.length() >= 0.5f) {
                shaftFrame.fromDefault(currentShaftTangent);
                if (shaftFrame.valid) frameCalculated = true;
            } else shaftFrame.valid = false;
        }

        if (!shaftFrame.valid) {
            IECore::msg(IECore::Msg::Debug, staticTypeName(), boost::format("Could not calculate valid frame for shaft attach point on barb %d. Using default identity.") % barbIdx);
             // Frame constructor sets a default identity
        }

        float minU = std::numeric_limits<float>::max();
        size_t barbRootLocalIndex = 0;
        for (int k = 0; k < numBarbCVs; ++k) {
            const size_t barbGlobalPointIndex = currentBarbPointOffset + k;
             if (barbGlobalPointIndex >= barbU.size()) { // Bounds check
                IECore::msg(IECore::Msg::Error, staticTypeName(), "barbU access out of bounds!");
                // Handle error: skip barb or use default root index
                barbAttrsValid = false; // Mark as invalid to skip this barb
                break;
            }
            if (barbU[barbGlobalPointIndex] < minU) {
                minU = barbU[barbGlobalPointIndex];
                barbRootLocalIndex = k;
            }
        }
         if (!barbAttrsValid) { // Check if loop above failed
            fill_defaults_and_continue(); continue;
        }

        const Imath::V3f barbRootP = barbsP[currentBarbPointOffset + barbRootLocalIndex];
        Imath::M44f M_shaft_rest;
        M_shaft_rest.setValue(
            shaftFrame.T.x, shaftFrame.T.y, shaftFrame.T.z, 0.0f,
            shaftFrame.B.x, shaftFrame.B.y, shaftFrame.B.z, 0.0f,
            shaftFrame.N.x, shaftFrame.N.y, shaftFrame.N.z, 0.0f,
            shaftFrame.P.x, shaftFrame.P.y, shaftFrame.P.z, 1.0f);
        Imath::V3f world_offset_vector = barbRootP - shaftFrame.P;
        Imath::M44f M_shaft_rest_inverse = M_shaft_rest.inverse();
        Imath::V3f barbRootOffsetLocal_value;
        M_shaft_rest_inverse.multDirMatrix(world_offset_vector, barbRootOffsetLocal_value);

        if (bind_shaftHairId_int_writable) bind_shaftHairId_int_writable->push_back(barb_hair_id_int);
        else if (bind_shaftHairId_str_writable) bind_shaftHairId_str_writable->push_back(barb_hair_id_str);
        bind_shaftPointId_data->writable().push_back(shaft_pt_id);
        bind_shaftRest_P_data->writable().push_back(shaftFrame.P);
        bind_shaftRest_T_data->writable().push_back(shaftFrame.T);
        bind_shaftRest_B_data->writable().push_back(shaftFrame.B);
        bind_shaftRest_N_data->writable().push_back(shaftFrame.N);
        bind_barbRootOffsetLocal_data->writable().push_back(barbRootOffsetLocal_value);

        currentBarbPointOffset += numBarbCVs;
    }

    if (shaftsHaveIntHairId) resultCurves->variables["bind_shaftHairId"] = PrimitiveVariable(PrimitiveVariable::Uniform, bind_shaftHairId_data);
    else resultCurves->variables["bind_shaftHairId"] = PrimitiveVariable(PrimitiveVariable::Uniform, bind_shaftHairId_data); // String if not int
    resultCurves->variables["bind_shaftPointId"] = PrimitiveVariable(PrimitiveVariable::Uniform, bind_shaftPointId_data);
    resultCurves->variables["bind_shaftRest_P"] = PrimitiveVariable(PrimitiveVariable::Uniform, bind_shaftRest_P_data);
    resultCurves->variables["bind_shaftRest_T"] = PrimitiveVariable(PrimitiveVariable::Uniform, bind_shaftRest_T_data);
    resultCurves->variables["bind_shaftRest_B"] = PrimitiveVariable(PrimitiveVariable::Uniform, bind_shaftRest_B_data);
    resultCurves->variables["bind_shaftRest_N"] = PrimitiveVariable(PrimitiveVariable::Uniform, bind_shaftRest_N_data);
    resultCurves->variables["bind_barbRootOffsetLocal"] = PrimitiveVariable(PrimitiveVariable::Uniform, bind_barbRootOffsetLocal_data);

    return resultCurves;
}