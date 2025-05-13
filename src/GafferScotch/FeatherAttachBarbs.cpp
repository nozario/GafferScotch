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
    ConstObjectPtr inputObject = inPlug()->object(path);
    if (inputObject)
    { // Ensure input object is valid before hashing to avoid errors
        inputObject->hash(h);
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
        return inputObject; // Not curves, pass through
    }

    ConstObjectPtr shaftsObject = inShaftsPlug()->object(path);
    const CurvesPrimitive *inShaftsCurves = runTimeCast<const CurvesPrimitive>(shaftsObject.get());

    if (!inShaftsCurves)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "Shaft input is not a CurvesPrimitive or not found at expected path.");
        return inputObject;
    }

    // Retrieve attribute names from plugs
    const std::string hairIdAttr = hairIdAttrNamePlug()->getValue();
    const std::string shaftPointIdAttr = shaftPointIdAttrNamePlug()->getValue();
    const std::string barbParamAttr = barbParamAttrNamePlug()->getValue();
    const std::string shaftUpVectorAttr = shaftUpVectorPrimVarNamePlug()->getValue();
    const std::string shaftOrientAttr = shaftPointOrientAttrNamePlug()->getValue();

    // --- Validation of essential components ---
    if (hairIdAttr.empty() || shaftPointIdAttr.empty() || barbParamAttr.empty())
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "One or more critical attribute name plugs are empty.");
        return inputObject;
    }

    // Get P from barbs
    const V3fVectorData *barbsPData = inBarbsCurves->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);
    if (!barbsPData)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "Input barbs are missing 'P' primitive variable.");
        return inputObject;
    }
    const std::vector<Imath::V3f> &barbsP = barbsPData->readable();

    // Get P from shafts
    const V3fVectorData *shaftsPData = inShaftsCurves->variableData<V3fVectorData>("P", PrimitiveVariable::Vertex);
    if (!shaftsPData)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "Input shafts are missing 'P' primitive variable.");
        return inputObject;
    }
    const std::vector<Imath::V3f> &shaftsP = shaftsPData->readable();

    // Get verticesPerCurve from barbs
    const IntVectorData *barbsVertsPerCurveData = inBarbsCurves->verticesPerCurve();
    if (!barbsVertsPerCurveData)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "Input barbs are missing verticesPerCurve data.");
        return inputObject;
    }
    const std::vector<int> &barbsVertsPerCurve = barbsVertsPerCurveData->readable();
    if (barbsVertsPerCurve.empty())
    {
        return inputObject; // No curves to process
    }

    // Get verticesPerCurve from shafts
    const IntVectorData *shaftsVertsPerCurveData = inShaftsCurves->verticesPerCurve();
    if (!shaftsVertsPerCurveData)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "Input shafts are missing verticesPerCurve data.");
        return inputObject;
    }
    const std::vector<int> &shaftsVertsPerCurve = shaftsVertsPerCurveData->readable();
    if (shaftsVertsPerCurve.empty())
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), "Input shafts have no curves to process.");
        return inputObject; // No shaft curves to process
    }

    // --- Prepare output curves ---
    CurvesPrimitivePtr resultCurves = new CurvesPrimitive();
    resultCurves->setTopologyUnchecked(
        barbsVertsPerCurveData->copy(),
        inBarbsCurves->basis(),
        inBarbsCurves->periodic());

    // Copy existing primvars from barbs to result, except P which will be computed if necessary (though for Attach, P is usually not changed)
    for (const auto &[name, primVar] : inBarbsCurves->variables)
    {
        if (name != "P") // P is taken from barbsPData directly if needed, or not modified by Attach
        {
            resultCurves->variables[name] = primVar;
        }
    }
    // Ensure P is present in the output, using the original barb positions for now.
    resultCurves->variables["P"] = PrimitiveVariable(PrimitiveVariable::Vertex, barbsPData->copy());

    // --- Retrieve binding-critical attributes (and validate existence) ---

    // Barb attributes
    const PrimitiveVariable::DataAndInterpolation barbHairId = SceneAlgo::primitiveVariable(inBarbsCurves, hairIdAttr);
    const PrimitiveVariable::DataAndInterpolation barbShaftPointId = SceneAlgo::primitiveVariable(inBarbsCurves, shaftPointIdAttr);
    const FloatVectorData *barbParamUData = inBarbsCurves->variableData<FloatVectorData>(barbParamAttr, PrimitiveVariable::Vertex);

    if (!barbHairId.data || !barbShaftPointId.data || !barbParamUData)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Missing one or more required primitive variables on input barbs: '%s', '%s', '%s'.") % hairIdAttr % shaftPointIdAttr % barbParamAttr);
        return inputObject;
    }

    // Shaft attributes
    const PrimitiveVariable::DataAndInterpolation shaftHairId = SceneAlgo::primitiveVariable(inShaftsCurves, hairIdAttr);
    if (!shaftHairId.data)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Missing required primitive variable '%s' on input shafts.") % hairIdAttr);
        return inputObject;
    }

    const V3fVectorData *shaftUpVectorData = nullptr;
    if (!shaftUpVectorAttr.empty())
    {
        shaftUpVectorData = inShaftsCurves->variableData<V3fVectorData>(shaftUpVectorAttr, PrimitiveVariable::Uniform); // Assuming Uniform as per plan
        if (!shaftUpVectorData)
        {
            IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Shaft up-vector attribute '%s' not found or not V3fVectorData (Uniform).") % shaftUpVectorAttr);
            // This is optional, so we can continue
        }
    }

    const V4fVectorData *shaftOrientData = nullptr;
    PrimitiveVariable::Interpolation shaftOrientInterpolation = PrimitiveVariable::Invalid;
    if (!shaftOrientAttr.empty())
    {
        // Check for Vertex interpolation first, then Uniform
        shaftOrientData = inShaftsCurves->variableData<V4fVectorData>(shaftOrientAttr, PrimitiveVariable::Vertex);
        if (shaftOrientData)
        {
            shaftOrientInterpolation = PrimitiveVariable::Vertex;
        }
        else
        {
            shaftOrientData = inShaftsCurves->variableData<V4fVectorData>(shaftOrientAttr, PrimitiveVariable::Uniform);
            if (shaftOrientData)
            {
                shaftOrientInterpolation = PrimitiveVariable::Uniform;
            }
        }

        if (!shaftOrientData)
        {
            IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Shaft orientation attribute '%s' not found or not V4fVectorData (Vertex or Uniform).") % shaftOrientAttr);
            // This is optional, so we can continue
        }
    }

    // --- Actual logic from feather_bin_plan.md will go here ---
    // 2. Prepare Data Structures (map hair_id to shaft data)
    struct ShaftInfo
    {
        size_t primitiveIndex;
        const std::vector<Imath::V3f> *points; // Points for this specific shaft curve
        size_t numPoints;
        const Imath::V3f *upVector;                           // Optional, points to a single V3f if uniform and valid
        const Imath::V4f *orientQuaternion;                   // Optional, points to a single V4f if uniform and valid
        PrimitiveVariable::Interpolation orientInterpolation; // If orientQuaternion is per-vertex
    };

    std::map<int, ShaftInfo> intShaftLookup;
    std::map<std::string, ShaftInfo> stringShaftLookup;
    bool shaftsHaveIntHairId = false;
    bool shaftsHaveStringHairId = false;

    // Process shaft hair IDs
    size_t currentShaftPointOffset = 0;
    for (size_t i = 0; i < shaftsVertsPerCurve.size(); ++i)
    {
        const int numShaftCVs = shaftsVertsPerCurve[i];
        if (numShaftCVs <= 0)
            continue;

        ShaftInfo info;
        info.primitiveIndex = i;
        // This needs to be a sub-vector or a pointer to the start of this curve's points in shaftsP
        // For now, let's pass all points and use an offset, though direct sub-vector is cleaner if possible.
        // Actual points for this curve: std::vector<Imath::V3f>(shaftsP.begin() + currentShaftPointOffset, shaftsP.begin() + currentShaftPointOffset + numShaftCVs)
        info.points = &shaftsP; // This will be combined with currentShaftPointOffset for lookup
        info.numPoints = numShaftCVs;
        info.upVector = nullptr;
        info.orientQuaternion = nullptr;
        info.orientInterpolation = PrimitiveVariable::Invalid;

        if (shaftUpVectorData && shaftUpVectorData->readable().size() > i) // Uniform check for up-vector
        {
            info.upVector = &(shaftUpVectorData->readable()[i]);
            // As per plan, shaftUpVector is Uniform, so one per shaft primitive
            // If shaftUpVectorData->readable().size() == 1, it applies to all, otherwise per prim.
            if (shaftUpVectorData->interpolation() == PrimitiveVariable::Uniform && shaftUpVectorData->readable().size() == 1)
            {
                info.upVector = &(shaftUpVectorData->readable()[0]);
            }
            else if (shaftUpVectorData->interpolation() == PrimitiveVariable::Uniform && shaftUpVectorData->readable().size() > i)
            {
                info.upVector = &(shaftUpVectorData->readable()[i]);
            }
            else
            {
                info.upVector = nullptr; // Mismatch or invalid data for this prim
            }
        }

        if (shaftOrientData)
        {
            info.orientInterpolation = shaftOrientInterpolation; // From previous check
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
            // If Vertex, we handle it per point later. For Uniform, we store the per-primitive quaternion here.
        }

        // Get hairId for this shaft primitive
        if (const IntData *idData = runTimeCast<const IntData>(shaftHairId.data.get()))
        {
            shaftsHaveIntHairId = true;
            int id = 0;
            if (shaftHairId.interpolation == PrimitiveVariable::Uniform && idData->readable().size() == 1)
                id = idData->readable()[0];
            else if ((shaftHairId.interpolation == PrimitiveVariable::Uniform || shaftHairId.interpolation == PrimitiveVariable::Primitive) && idData->readable().size() > i)
                id = idData->readable()[i];
            else
            { /* Warning or skip */
                continue;
            }
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
            { /* Warning or skip */
                continue;
            }
            stringShaftLookup[id] = info;
        }
        else if (const IntVectorData *idVecData = runTimeCast<const IntVectorData>(shaftHairId.data.get())) // Handle IntVectorData for Uniform/Primitive
        {
            shaftsHaveIntHairId = true;
            int id = 0;
            if ((shaftHairId.interpolation == PrimitiveVariable::Uniform || shaftHairId.interpolation == PrimitiveVariable::Primitive) && idVecData->readable().size() > i)
                id = idVecData->readable()[i];
            else if (shaftHairId.interpolation == PrimitiveVariable::Constant && !idVecData->readable().empty())
                id = idVecData->readable()[0]; // Constant applies to all
            else
            { /* Warning or skip */
                continue;
            }
            intShaftLookup[id] = info;
        }
        else if (const StringVectorData *idVecData = runTimeCast<const StringVectorData>(shaftHairId.data.get())) // Handle StringVectorData for Uniform/Primitive
        {
            shaftsHaveStringHairId = true;
            std::string id;
            if ((shaftHairId.interpolation == PrimitiveVariable::Uniform || shaftHairId.interpolation == PrimitiveVariable::Primitive) && idVecData->readable().size() > i)
                id = idVecData->readable()[i];
            else if (shaftHairId.interpolation == PrimitiveVariable::Constant && !idVecData->readable().empty())
                id = idVecData->readable()[0]; // Constant applies to all
            else
            { /* Warning or skip */
                continue;
            }
            stringShaftLookup[id] = info;
        }
        currentShaftPointOffset += numShaftCVs;
    }

    if (!shaftsHaveIntHairId && !shaftsHaveStringHairId)
    {
        IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Shaft hair ID attribute '%s' has unsupported data type or interpolation.") % hairIdAttr);
        return inputObject;
    }

    // --- Initialize output primitive variables for binding data ---
    // These will be uniform, one value per output barb primitive.
    // Get numBarbs from barbsVertsPerCurve.size()
    const size_t numBarbs = barbsVertsPerCurve.size();

    DataPtr bind_shaftHairId_data;
    if (shaftsHaveIntHairId)
        bind_shaftHairId_data = new IntVectorData();
    else
        bind_shaftHairId_data = new StringVectorData();
    std::vector<int> *bind_shaftHairId_int_writable = shaftsHaveIntHairId ? &(static_cast<IntVectorData *>(bind_shaftHairId_data.get())->writable()) : nullptr;
    std::vector<std::string> *bind_shaftHairId_str_writable = shaftsHaveStringHairId ? &(static_cast<StringVectorData *>(bind_shaftHairId_data.get())->writable()) : nullptr;
    if (bind_shaftHairId_int_writable)
        bind_shaftHairId_int_writable->reserve(numBarbs);
    if (bind_shaftHairId_str_writable)
        bind_shaftHairId_str_writable->reserve(numBarbs);

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

    // Pre-calculate shaft point offsets for easier lookup within the global shaftsP vector
    std::vector<size_t> shaftCurveStartPointIndices(shaftsVertsPerCurve.size());
    size_t runningShaftPointTotal = 0;
    for (size_t i = 0; i < shaftsVertsPerCurve.size(); ++i)
    {
        shaftCurveStartPointIndices[i] = runningShaftPointTotal;
        runningShaftPointTotal += shaftsVertsPerCurve[i];
    }

    // 3. Iterate through Barb Primitives
    size_t currentBarbPointOffset = 0;
    for (size_t barbIdx = 0; barbIdx < numBarbs; ++barbIdx)
    {
        const int numBarbCVs = barbsVertsPerCurve[barbIdx];
        if (numBarbCVs <= 0)
        {
            // Add empty/default bind data if we must maintain counts
            if (bind_shaftHairId_int_writable)
                bind_shaftHairId_int_writable->push_back(0);
            if (bind_shaftHairId_str_writable)
                bind_shaftHairId_str_writable->push_back("");
            bind_shaftPointId_data->writable().push_back(-1);
            bind_shaftRest_P_data->writable().push_back(Imath::V3f(0));
            bind_shaftRest_T_data->writable().push_back(Imath::V3f(1, 0, 0));
            bind_shaftRest_B_data->writable().push_back(Imath::V3f(0, 1, 0));
            bind_shaftRest_N_data->writable().push_back(Imath::V3f(0, 0, 1));
            bind_barbRootOffsetLocal_data->writable().push_back(Imath::V3f(0));
            currentBarbPointOffset += numBarbCVs; // Ensure offset is still updated
            continue;
        }

        // a. Read barb attributes
        int barb_hair_id_int = 0;
        std::string barb_hair_id_str;
        bool barbHasIntHairId = false;

        if (const IntData *idData = runTimeCast<const IntData>(barbHairId.data.get()))
        {
            barbHasIntHairId = true;
            if (barbHairId.interpolation == PrimitiveVariable::Uniform && idData->readable().size() == 1)
                barb_hair_id_int = idData->readable()[0];
            else if ((barbHairId.interpolation == PrimitiveVariable::Uniform || barbHairId.interpolation == PrimitiveVariable::Primitive) && idData->readable().size() > barbIdx)
                barb_hair_id_int = idData->readable()[barbIdx];
            else
            { /* Warning or skip barb */
                IECore::msg(IECore::Msg::Warning, staticTypeName(), "Invalid hairId on barb");
                continue;
            }
        }
        else if (const StringData *idData = runTimeCast<const StringData>(barbHairId.data.get()))
        {
            if (barbHairId.interpolation == PrimitiveVariable::Uniform && idData->readable().size() == 1)
                barb_hair_id_str = idData->readable()[0];
            else if ((barbHairId.interpolation == PrimitiveVariable::Uniform || barbHairId.interpolation == PrimitiveVariable::Primitive) && idData->readable().size() > barbIdx)
                barb_hair_id_str = idData->readable()[barbIdx];
            else
            { /* Warning or skip barb */
                IECore::msg(IECore::Msg::Warning, staticTypeName(), "Invalid hairId on barb");
                continue;
            }
        }
        else if (const IntVectorData *idVecData = runTimeCast<const IntVectorData>(barbHairId.data.get()))
        {
            barbHasIntHairId = true;
            if ((barbHairId.interpolation == PrimitiveVariable::Uniform || barbHairId.interpolation == PrimitiveVariable::Primitive) && idVecData->readable().size() > barbIdx)
                barb_hair_id_int = idVecData->readable()[barbIdx];
            else if (barbHairId.interpolation == PrimitiveVariable::Constant && !idVecData->readable().empty())
                barb_hair_id_int = idVecData->readable()[0];
            else
            { /* Warning or skip barb */
                IECore::msg(IECore::Msg::Warning, staticTypeName(), "Invalid hairId on barb");
                continue;
            }
        }
        else if (const StringVectorData *idVecData = runTimeCast<const StringVectorData>(barbHairId.data.get()))
        {
            if ((barbHairId.interpolation == PrimitiveVariable::Uniform || barbHairId.interpolation == PrimitiveVariable::Primitive) && idVecData->readable().size() > barbIdx)
                barb_hair_id_str = idVecData->readable()[barbIdx];
            else if (barbHairId.interpolation == PrimitiveVariable::Constant && !idVecData->readable().empty())
                barb_hair_id_str = idVecData->readable()[0];
            else
            { /* Warning or skip barb */
                IECore::msg(IECore::Msg::Warning, staticTypeName(), "Invalid hairId on barb");
                continue;
            }
        }
        else
        {
            IECore::msg(IECore::Msg::Warning, staticTypeName(), "Unsupported hairId type on barb");
            continue;
        }

        int shaft_pt_id = -1;
        if (const IntData *idData = runTimeCast<const IntData>(barbShaftPointId.data.get()))
        {
            if (barbShaftPointId.interpolation == PrimitiveVariable::Uniform && idData->readable().size() == 1)
                shaft_pt_id = idData->readable()[0];
            else if ((barbShaftPointId.interpolation == PrimitiveVariable::Uniform || barbShaftPointId.interpolation == PrimitiveVariable::Primitive) && idData->readable().size() > barbIdx)
                shaft_pt_id = idData->readable()[barbIdx];
            else
            { /* Warning or skip barb */
                IECore::msg(IECore::Msg::Warning, staticTypeName(), "Invalid shaftPointId on barb");
                continue;
            }
        }
        else if (const IntVectorData *idVecData = runTimeCast<const IntVectorData>(barbShaftPointId.data.get()))
        {
            if ((barbShaftPointId.interpolation == PrimitiveVariable::Uniform || barbShaftPointId.interpolation == PrimitiveVariable::Primitive) && idVecData->readable().size() > barbIdx)
                shaft_pt_id = idVecData->readable()[barbIdx];
            else if (barbShaftPointId.interpolation == PrimitiveVariable::Constant && !idVecData->readable().empty())
                shaft_pt_id = idVecData->readable()[0];
            else
            { /* Warning or skip barb */
                IECore::msg(IECore::Msg::Warning, staticTypeName(), "Invalid shaftPointId on barb");
                continue;
            }
        }
        else
        {
            IECore::msg(IECore::Msg::Warning, staticTypeName(), "Unsupported shaftPointId type on barb");
            continue;
        }

        // b. Find parent shaft
        const ShaftInfo *foundShaftInfo = nullptr;
        if (barbHasIntHairId && shaftsHaveIntHairId)
        {
            auto it = intShaftLookup.find(barb_hair_id_int);
            if (it != intShaftLookup.end())
                foundShaftInfo = &it->second;
        }
        else if (!barbHasIntHairId && shaftsHaveStringHairId)
        {
            auto it = stringShaftLookup.find(barb_hair_id_str);
            if (it != stringShaftLookup.end())
                foundShaftInfo = &it->second;
        }
        else if (barbHasIntHairId != shaftsHaveIntHairId) // Mismatch
        {
            IECore::msg(IECore::Msg::Warning, staticTypeName(), "Hair ID type mismatch between barbs and shafts.");
            // Fill with default and continue to next barb
            if (bind_shaftHairId_int_writable)
                bind_shaftHairId_int_writable->push_back(barb_hair_id_int);
            else if (bind_shaftHairId_str_writable)
                bind_shaftHairId_str_writable->push_back(barb_hair_id_str); // Store original barb id
            bind_shaftPointId_data->writable().push_back(shaft_pt_id);
            bind_shaftRest_P_data->writable().push_back(Imath::V3f(0));
            bind_shaftRest_T_data->writable().push_back(Imath::V3f(1, 0, 0));
            bind_shaftRest_B_data->writable().push_back(Imath::V3f(0, 1, 0));
            bind_shaftRest_N_data->writable().push_back(Imath::V3f(0, 0, 1));
            bind_barbRootOffsetLocal_data->writable().push_back(Imath::V3f(0));
            currentBarbPointOffset += numBarbCVs;
            continue;
        }

        if (!foundShaftInfo)
        {
            IECore::msg(IECore::Msg::Detail, staticTypeName(), boost::format("Shaft with hair_id not found for barb %d.") % barbIdx);
            // Fill with default and continue to next barb
            if (bind_shaftHairId_int_writable)
                bind_shaftHairId_int_writable->push_back(barb_hair_id_int);
            else if (bind_shaftHairId_str_writable)
                bind_shaftHairId_str_writable->push_back(barb_hair_id_str);
            bind_shaftPointId_data->writable().push_back(shaft_pt_id);
            bind_shaftRest_P_data->writable().push_back(Imath::V3f(0));
            bind_shaftRest_T_data->writable().push_back(Imath::V3f(1, 0, 0));
            bind_shaftRest_B_data->writable().push_back(Imath::V3f(0, 1, 0));
            bind_shaftRest_N_data->writable().push_back(Imath::V3f(0, 0, 1));
            bind_barbRootOffsetLocal_data->writable().push_back(Imath::V3f(0));
            currentBarbPointOffset += numBarbCVs;
            continue;
        }

        // Validate shaft_pt_id against the number of points in the found shaft
        if (shaft_pt_id < 0 || static_cast<size_t>(shaft_pt_id) >= foundShaftInfo->numPoints)
        {
            IECore::msg(IECore::Msg::Warning, staticTypeName(), boost::format("Invalid shaft_pt_id %d for barb %d (shaft has %d points).") % shaft_pt_id % barbIdx % foundShaftInfo->numPoints);
            // Fill with default and continue
            if (bind_shaftHairId_int_writable)
                bind_shaftHairId_int_writable->push_back(barb_hair_id_int);
            else if (bind_shaftHairId_str_writable)
                bind_shaftHairId_str_writable->push_back(barb_hair_id_str);
            bind_shaftPointId_data->writable().push_back(shaft_pt_id);
            bind_shaftRest_P_data->writable().push_back(Imath::V3f(0));
            bind_shaftRest_T_data->writable().push_back(Imath::V3f(1, 0, 0));
            bind_shaftRest_B_data->writable().push_back(Imath::V3f(0, 1, 0));
            bind_shaftRest_N_data->writable().push_back(Imath::V3f(0, 0, 1));
            bind_barbRootOffsetLocal_data->writable().push_back(Imath::V3f(0));
            currentBarbPointOffset += numBarbCVs;
            continue;
        }

        const size_t shaftCurveGlobalPointStartIndex = shaftCurveStartPointIndices[foundShaftInfo->primitiveIndex];
        const Imath::V3f shaftAttachP = (*foundShaftInfo->points)[shaftCurveGlobalPointStartIndex + shaft_pt_id];

        // c. Calculate Shaft Attachment Frame (Rest)
        Frame shaftFrame;
        shaftFrame.P = shaftAttachP;

        bool frameCalculated = false;

        // 1. Try using shaftPointOrientAttrName
        if (foundShaftInfo->orientInterpolation != PrimitiveVariable::Invalid && shaftOrientData)
        {
            Imath::V4f qOrient;
            if (foundShaftInfo->orientInterpolation == PrimitiveVariable::Uniform && foundShaftInfo->orientQuaternion)
            {
                qOrient = *foundShaftInfo->orientQuaternion;
                shaftFrame.fromQuaternion(qOrient);
                if (shaftFrame.valid)
                    frameCalculated = true;
            }
            else if (foundShaftInfo->orientInterpolation == PrimitiveVariable::Vertex)
            {
                const size_t shaftGlobalOrientIndex = shaftCurveGlobalPointStartIndex + shaft_pt_id;
                if (shaftGlobalOrientIndex < shaftOrientData->readable().size())
                {
                    qOrient = shaftOrientData->readable()[shaftGlobalOrientIndex];
                    shaftFrame.fromQuaternion(qOrient);
                    if (shaftFrame.valid)
                        frameCalculated = true;
                }
            }
        }

        // 2. If not calculated, try tangent and up-vector
        Imath::V3f currentShaftTangent;
        if (!frameCalculated)
        {
            currentShaftTangent = calculateTangent(*foundShaftInfo->points, shaftCurveGlobalPointStartIndex, foundShaftInfo->numPoints, shaft_pt_id);
            if (currentShaftTangent.length() < 0.5f)
            {                             // Invalid tangent
                shaftFrame.valid = false; // Mark frame as invalid
            }
            else
            {
                if (foundShaftInfo->upVector)
                {
                    shaftFrame.fromUpVector(currentShaftTangent, *foundShaftInfo->upVector);
                    if (shaftFrame.valid)
                        frameCalculated = true;
                }
            }
        }

        // 3. If still not calculated, use fallback default method
        if (!frameCalculated && shaftFrame.valid) // shaftFrame.valid check ensures tangent was good
        {
            if (currentShaftTangent.length() < 0.5f && !frameCalculated)
            { // Recalculate if not done yet and previous failed
                currentShaftTangent = calculateTangent(*foundShaftInfo->points, shaftCurveGlobalPointStartIndex, foundShaftInfo->numPoints, shaft_pt_id);
            }
            if (currentShaftTangent.length() > 0.5f)
            {
                shaftFrame.fromDefault(currentShaftTangent);
                if (shaftFrame.valid)
                    frameCalculated = true;
            }
            else
            {
                shaftFrame.valid = false; // No valid tangent to begin with
            }
        }

        if (!shaftFrame.valid)
        {
            IECore::msg(IECore::Msg::Debug, staticTypeName(), boost::format("Could not calculate valid frame for shaft attach point on barb %d. Using default identity.") % barbIdx);
            // Default frame is already set in Frame constructor if all else fails
        }

        // d. Calculate Barb Root Offset
        //    Identify the root point of the current barb (vertex with minimum barbParamAttrName value)
        const std::vector<float> &barbU = barbParamUData->readable();
        float minU = std::numeric_limits<float>::max();
        size_t barbRootLocalIndex = 0;
        for (int k = 0; k < numBarbCVs; ++k)
        {
            const size_t barbGlobalPointIndex = currentBarbPointOffset + k;
            if (barbU[barbGlobalPointIndex] < minU)
            {
                minU = barbU[barbGlobalPointIndex];
                barbRootLocalIndex = k;
            }
        }
        const Imath::V3f barbRootP = barbsP[currentBarbPointOffset + barbRootLocalIndex];

        //    Construct M_shaft_rest from shaftFrame.T,B,N and shaftFrame.P
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

        // e. Store binding attributes
        if (bind_shaftHairId_int_writable)
            bind_shaftHairId_int_writable->push_back(barb_hair_id_int);
        else if (bind_shaftHairId_str_writable)
            bind_shaftHairId_str_writable->push_back(barb_hair_id_str);
        bind_shaftPointId_data->writable().push_back(shaft_pt_id);
        bind_shaftRest_P_data->writable().push_back(shaftFrame.P);
        bind_shaftRest_T_data->writable().push_back(shaftFrame.T);
        bind_shaftRest_B_data->writable().push_back(shaftFrame.B);
        bind_shaftRest_N_data->writable().push_back(shaftFrame.N);
        bind_barbRootOffsetLocal_data->writable().push_back(barbRootOffsetLocal_value);

        currentBarbPointOffset += numBarbCVs;
    }

    // Add the new primitive variables to the output curves
    if (shaftsHaveIntHairId)
        resultCurves->variables["bind_shaftHairId"] = PrimitiveVariable(PrimitiveVariable::Uniform, bind_shaftHairId_data);
    else
        resultCurves->variables["bind_shaftHairId"] = PrimitiveVariable(PrimitiveVariable::Uniform, bind_shaftHairId_data);
    resultCurves->variables["bind_shaftPointId"] = PrimitiveVariable(PrimitiveVariable::Uniform, bind_shaftPointId_data);
    resultCurves->variables["bind_shaftRest_P"] = PrimitiveVariable(PrimitiveVariable::Uniform, bind_shaftRest_P_data);
    resultCurves->variables["bind_shaftRest_T"] = PrimitiveVariable(PrimitiveVariable::Uniform, bind_shaftRest_T_data);
    resultCurves->variables["bind_shaftRest_B"] = PrimitiveVariable(PrimitiveVariable::Uniform, bind_shaftRest_B_data);
    resultCurves->variables["bind_shaftRest_N"] = PrimitiveVariable(PrimitiveVariable::Uniform, bind_shaftRest_N_data);
    resultCurves->variables["bind_barbRootOffsetLocal"] = PrimitiveVariable(PrimitiveVariable::Uniform, bind_barbRootOffsetLocal_data);

    return resultCurves;
}