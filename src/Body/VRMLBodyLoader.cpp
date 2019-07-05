/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "VRMLBodyLoader.h"
#include "Body.h"
#include "ForceSensor.h"
#include "RateGyroSensor.h"
#include "AccelerationSensor.h"
#include "Camera.h"
#include "RangeCamera.h"
#include "RangeSensor.h"
#include "PointLight.h"
#include "SpotLight.h"
#include <cnoid/FileUtil>
#include <cnoid/Exception>
#include <cnoid/EasyScanner>
#include <cnoid/VRMLParser>
#include <cnoid/VRMLToSGConverter>
#include <cnoid/ValueTree>
#include <cnoid/NullOut>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class VRMLBodyLoaderImpl
{
public:
    enum {
        PROTO_UNDEFINED = 0,
        PROTO_HUMANOID,
        PROTO_JOINT,
        PROTO_SEGMENT,
        PROTO_SURFACE,
        PROTO_DEVICE,
        PROTO_EXTRAJOINT,
        NUM_PROTOS
    };

    typedef std::bitset<NUM_PROTOS> ProtoIdSet;

    struct SegmentInfo
    {
        Vector3 c;
        double m;
    };
        
    struct LinkInfo
    {
        Link* link;
        double m;
        Vector3 c;
        Matrix3 I;
        vector<SegmentInfo> segments;
        SgGroupPtr visualShape;
        SgGroupPtr collisionShape;
        bool isSurfaceNodeUsed;
    };
        
    VRMLParser vrmlParser;
    Body* body;
    VRMLProtoInstancePtr rootJointNode;
    vector<VRMLProtoInstancePtr> extraJointNodes;
    vector<bool> validJointIdSet;
    size_t numValidJointIds;
    VRMLToSGConverter sgConverter;
    int divisionNumber;
    ostream* os_;
    bool isVerbose;
    int messageIndent;

    typedef std::function<DevicePtr(VRMLProtoInstance* node)> DeviceFactory;
    typedef map<string, DeviceFactory> DeviceFactoryMap;
    static DeviceFactoryMap deviceFactories;

    typedef map<Link*, VRMLNodePtr> LinkOriginalMap;
    LinkOriginalMap linkOriginalMap;

    ostream& os() { return *os_; }

    void putMessage(const std::string& message){
        os() << string(messageIndent, ' ') + message + "\n";
    }
        
    VRMLBodyLoaderImpl();
    ~VRMLBodyLoaderImpl();
    VRMLNodePtr getOriginalNode(Link* link);
    bool load(Body* body, const std::string& filename);
    void readTopNodes();
    void checkHumanoidProto(VRMLProto* proto);
    void checkJointProto(VRMLProto* proto);
    void checkSegmentProto(VRMLProto* proto);
    void checkSurfaceProto(VRMLProto* proto);
    void checkSensorProtoCommon(VRMLProto* proto);
    void checkDeviceProtoCommon(VRMLProto* proto);
    void checkVisionSensorProto(VRMLProto* proto);
    void checkRangeSensorProto(VRMLProto* proto);
    void checkSpotLightDeviceProto(VRMLProto* proto);
    void checkExtraJointProto(VRMLProto* proto);
    void readHumanoidNode(VRMLProtoInstance* humanoidNode);
    Link* readJointNode(VRMLProtoInstance* jointNode, const Matrix3& parentRs);
    Link* createLink(VRMLProtoInstance* jointNode, const Matrix3& parentRs);
    void readJointSubNodes(LinkInfo& iLink, MFNode& childNodes, const ProtoIdSet& acceptableProtoIds, const Affine3& T);
    void readSegmentNode(LinkInfo& iLink, VRMLProtoInstance* segmentNode, const Affine3& T);
    void readSurfaceNode(LinkInfo& iLink, VRMLProtoInstance* segmentShapeNode, const Affine3& T);
    void readDeviceNode(LinkInfo& iLink, VRMLProtoInstance* deviceNode, const Affine3& T);
    static void readDeviceCommonParameters(Device& device, VRMLProtoInstance* node);
    static ForceSensorPtr createForceSensor(VRMLProtoInstance* node);
    static RateGyroSensorPtr createRateGyroSensor(VRMLProtoInstance* node);
    static AccelerationSensorPtr createAccelerationSensor(VRMLProtoInstance* node);
    static CameraPtr createCamera(VRMLProtoInstance* node);
    static RangeSensorPtr createRangeSensor(VRMLProtoInstance* node);
    static void readLightDeviceCommonParameters(Light& light, VRMLProtoInstance* node);
    static SpotLightPtr createSpotLight(VRMLProtoInstance* node);
    void setExtraJoints();
};

VRMLBodyLoaderImpl::DeviceFactoryMap VRMLBodyLoaderImpl::deviceFactories;

}

namespace {

typedef void (VRMLBodyLoaderImpl::*ProtoCheckFunc)(VRMLProto* proto);
    
struct ProtoInfo
{
    ProtoInfo() { }
    ProtoInfo(int id, ProtoCheckFunc func) : id(id), protoCheckFunc(func) { }
    int id;
    ProtoCheckFunc protoCheckFunc;
};
    
typedef map<string, ProtoInfo> ProtoInfoMap;
ProtoInfoMap protoInfoMap;

void throwExceptionOfIllegalField(VRMLProto* proto, const std::string& name, const char* label)
{
    throw invalid_argument(
        format(_("Proto \"{0}\" must have the \"{1}\" field of {2} type"), proto->protoName, name, label));
}

template <typename TValue>
void requireField(VRMLProto* proto, const std::string& name){
    VRMLVariantField* field = proto->findField(name);
    if(!field || !stdx::holds_alternative<TValue>(*field)){
        throwExceptionOfIllegalField(proto, name, labelOfVRMLfieldType<TValue>());
    }
}

template <typename TValue>
VRMLVariantField* addField(VRMLProto* proto, const std::string& name, const TValue& defaultValue) {
    VRMLVariantField* field = proto->findField(name);
    if(!field){
        field = &proto->field(name);
        (*field) = defaultValue;
    } else if(!stdx::holds_alternative<TValue>(*field)){
        throwExceptionOfIllegalField(proto, name, labelOfVRMLfieldType<TValue>());
    }
    return field;
}

template <typename TValue>
VRMLVariantField* addField(VRMLProto* proto, const std::string& name) {
    return addField(proto, name, TValue());
}

double getLimitValue(VRMLVariantField& field, double defaultValue)
{
    MFFloat& values = stdx::get<MFFloat>(field);
    if(values.empty()){
        return defaultValue;
    }
    return values[0];
}

template<class ValueType> ValueType getValue(VRMLProtoInstance* node, const char* fieldName)
{
    VRMLProtoFieldMap::const_iterator p = node->fields.find(fieldName);
    if(p == node->fields.end()){
        BOOST_THROW_EXCEPTION(
            nonexistent_key_error()
            << error_info_key(fieldName)
            << error_info_message(
                format(_("Node \"{0}\" should have the field \"{1}\""),
                       node->proto->protoName, fieldName)));
    }
    return stdx::get<ValueType>(p->second);
}
       
void readVRMLfield(VRMLVariantField& field, string& out_s)
{
    switch(stdx::get_variant_index(field)){
    case SFSTRING:
        out_s = stdx::get<SFString>(field);
        break;
    case MFSTRING:
    {
        MFString& strings = stdx::get<MFString>(field);
        out_s = "";
        for(size_t i=0; i < strings.size(); i++){
            out_s += strings[i] + "\n";
        }
    }
    break;
    default:
        break;
    }
}

bool checkAndReadVRMLfield(VRMLProtoInstance* node, const char* key, bool& out_value)
{
    VRMLVariantField* field = node->findField(key);
    if(field && stdx::get_variant_index(*field) == SFBOOL){
        out_value = stdx::get<SFBool>(*field);
        return true;
    }
    return false;
}

bool checkAndReadVRMLfield(VRMLProtoInstance* node, const char* key, int& out_value)
{
    VRMLVariantField* field = node->findField(key);
    if(field && stdx::get_variant_index(*field) == SFINT32){
        out_value = stdx::get<SFInt32>(*field);
        return true;
    }
    return false;
}

void readVRMLfield(VRMLVariantField& field, double& out_value)
{
    out_value = stdx::get<SFFloat>(field);
}

bool checkAndReadVRMLfield(VRMLProtoInstance* node, const char* key, SFVec3f& out_value)
{
    VRMLVariantField* field = node->findField(key);
    if(field && stdx::get_variant_index(*field) == SFVEC3F){
        out_value = stdx::get<SFVec3f>(*field);
        return true;
    }
    return false;
}
   
void readVRMLfield(VRMLVariantField& field, Vector3& out_v)
{
    out_v = stdx::get<SFVec3f>(field);
}

void readVRMLfield(VRMLVariantField& field, Matrix3& out_R)
{
    if(stdx::get_variant_index(field) == SFROTATION){
        out_R = stdx::get<SFRotation>(field).toRotationMatrix();

    } else if(stdx::get_variant_index(field) == MFFLOAT){
        MFFloat& mf = stdx::get<MFFloat>(field);
        if(mf.size() >= 9){
            out_R <<
                mf[0], mf[1], mf[2],
                mf[3], mf[4], mf[5],
                mf[6], mf[7], mf[8];
        }
    }
}

}


VRMLBodyLoader::VRMLBodyLoader()
{
    impl = new VRMLBodyLoaderImpl();
}


VRMLBodyLoaderImpl::VRMLBodyLoaderImpl()
{
    divisionNumber = sgConverter.divisionNumber();
    isVerbose = false;
    body = 0;
    os_ = &nullout();
    
    if(protoInfoMap.empty()){
        protoInfoMap["Humanoid"] = ProtoInfo(PROTO_HUMANOID, &VRMLBodyLoaderImpl::checkHumanoidProto);
        protoInfoMap["Joint"] = ProtoInfo(PROTO_JOINT, &VRMLBodyLoaderImpl::checkJointProto);
        protoInfoMap["Segment"] = ProtoInfo(PROTO_SEGMENT, &VRMLBodyLoaderImpl::checkSegmentProto);
        protoInfoMap["Surface"] = ProtoInfo(PROTO_SURFACE, &VRMLBodyLoaderImpl::checkSurfaceProto);
        protoInfoMap["ForceSensor"] = ProtoInfo(PROTO_DEVICE, &VRMLBodyLoaderImpl::checkSensorProtoCommon);
        protoInfoMap["Gyro"] = ProtoInfo(PROTO_DEVICE, &VRMLBodyLoaderImpl::checkSensorProtoCommon);
        protoInfoMap["AccelerationSensor"] = ProtoInfo(PROTO_DEVICE, &VRMLBodyLoaderImpl::checkSensorProtoCommon);
        protoInfoMap["RangeSensor"] = ProtoInfo(PROTO_DEVICE, &VRMLBodyLoaderImpl::checkSensorProtoCommon);
        protoInfoMap["PressureSensor"] = ProtoInfo(PROTO_DEVICE, &VRMLBodyLoaderImpl::checkSensorProtoCommon);
        protoInfoMap["VisionSensor"] = ProtoInfo(PROTO_DEVICE, &VRMLBodyLoaderImpl::checkVisionSensorProto);
        protoInfoMap["RangeSensor"] = ProtoInfo(PROTO_DEVICE, &VRMLBodyLoaderImpl::checkRangeSensorProto);
        protoInfoMap["SpotLightDevice"] = ProtoInfo(PROTO_DEVICE, &VRMLBodyLoaderImpl::checkSpotLightDeviceProto);
        protoInfoMap["ExtraJoint"] = ProtoInfo(PROTO_EXTRAJOINT, &VRMLBodyLoaderImpl::checkExtraJointProto);
    }
    
    if(deviceFactories.empty()){
        deviceFactories["ForceSensor"]        = &VRMLBodyLoaderImpl::createForceSensor;
        deviceFactories["Gyro"]               = &VRMLBodyLoaderImpl::createRateGyroSensor;
        deviceFactories["AccelerationSensor"] = &VRMLBodyLoaderImpl::createAccelerationSensor;
        //sensorTypeMap["PressureSensor"]     = Sensor::PRESSURE;
        //sensorTypeMap["PhotoInterrupter"]   = Sensor::PHOTO_INTERRUPTER;
        //sensorTypeMap["TorqueSensor"]       = Sensor::TORQUE;
        deviceFactories["RangeSensor"]        = &VRMLBodyLoaderImpl::createRangeSensor;
        deviceFactories["VisionSensor"]       = &VRMLBodyLoaderImpl::createCamera;
        deviceFactories["SpotLightDevice"]    = &VRMLBodyLoaderImpl::createSpotLight;
    }
}


VRMLBodyLoader::~VRMLBodyLoader()
{
    delete impl;
}


VRMLBodyLoaderImpl::~VRMLBodyLoaderImpl()
{

}


void VRMLBodyLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
    impl->vrmlParser.setMessageSink(os);
    impl->sgConverter.setMessageSink(os);
}


void VRMLBodyLoader::setVerbose(bool on)
{
    impl->isVerbose = on;
}


/**
   \todo fully implement this mode
*/
void VRMLBodyLoader::enableShapeLoading(bool on)
{
    impl->sgConverter.setTriangulationEnabled(on);
    impl->sgConverter.setNormalGenerationEnabled(on);
}
    

void VRMLBodyLoader::setDefaultDivisionNumber(int n)
{
    impl->divisionNumber = n;
}


VRMLNodePtr VRMLBodyLoader::getOriginalNode(Link* link)
{
    return impl->getOriginalNode(link);
}


VRMLNodePtr VRMLBodyLoaderImpl::getOriginalNode(Link* link)
{
    LinkOriginalMap::iterator it;
    it = linkOriginalMap.find(link);
    if (it == linkOriginalMap.end()) {
        return NULL;
    }
    return it->second;
}


bool VRMLBodyLoader::load(Body* body, const std::string& filename)
{
    body->clearDevices();
    body->clearExtraJoints();
    return impl->load(body, filename);
}


bool VRMLBodyLoaderImpl::load(Body* body, const std::string& filename)
{
    bool result = false;

    this->body = body;
    rootJointNode = 0;
    extraJointNodes.clear();
    validJointIdSet.clear();
    numValidJointIds = 0;
    
    try {
        sgConverter.setDivisionNumber(divisionNumber);
        vrmlParser.load(filename);
        readTopNodes();
        if(body->modelName().empty()){
            body->setModelName(getBasename(filename));
        }
        result = true;
        os().flush();
        
    } catch(const ValueNode::Exception& ex){
        os() << ex.message() << endl;
    } catch(EasyScanner::Exception & ex){
        os() << ex.getFullMessage() << endl;
    } catch(const nonexistent_key_error& error){
        if(const std::string* message = boost::get_error_info<error_info_message>(error)){
            os() << *message << endl;
        }
    } catch(const std::exception& ex){
        os() << ex.what() << endl;
    }
    
    return result;
}


void VRMLBodyLoaderImpl::readTopNodes()
{
    bool humanoidNodeLoaded = false;

    VRMLGroupPtr nonHumanoidNodeGroup;
    
    while(VRMLNodePtr node = vrmlParser.readNode()){
        if(node->isCategoryOf(PROTO_DEF_NODE)){
            VRMLProto* proto = static_cast<VRMLProto*>(node.get());
            ProtoInfoMap::iterator p = protoInfoMap.find(proto->protoName);
            if(p != protoInfoMap.end()){
                ProtoInfo& info = p->second;
                (this->*info.protoCheckFunc)(proto);
            }
            continue;
        } else if(node->isCategoryOf(PROTO_INSTANCE_NODE)){
            VRMLProtoInstance* instance = static_cast<VRMLProtoInstance*>(node.get());
            if(instance->proto->protoName == "Humanoid") {
                if(humanoidNodeLoaded){
                    throw invalid_argument(_("Humanoid nodes more than one are defined."));
                }
                readHumanoidNode(instance);
                humanoidNodeLoaded = true;
                continue;
            } else if(instance->proto->protoName == "ExtraJoint") {
                extraJointNodes.push_back(instance);
                continue;
            }
        }
        if(!nonHumanoidNodeGroup){
            nonHumanoidNodeGroup = new VRMLGroup;
        }
        nonHumanoidNodeGroup->children.push_back(node);
    }
    vrmlParser.checkEOF();

    bool loaded = humanoidNodeLoaded;
    
    if(humanoidNodeLoaded){
        setExtraJoints();
    } else if(!nonHumanoidNodeGroup->children.empty()){
        SgNodePtr scene = sgConverter.convert(nonHumanoidNodeGroup);
        if(scene){
            Link* link = body->createLink();
            link->setName("Root");
            link->setShape(scene);
            link->setMass(1.0);
            link->setInertia(Matrix3::Identity());
            body->setRootLink(link);
            loaded = true;
        }
    }
    if(!loaded){
        throw invalid_argument(_("There are no VRML nodes which can be loaded as a Body."));
    }
}


void VRMLBodyLoaderImpl::checkHumanoidProto(VRMLProto* proto)
{
    // required fields
    requireField<SFVec3f>(proto, "center");
    requireField<MFNode>(proto, "humanoidBody");
    requireField<SFRotation>(proto, "rotation");
    requireField<SFVec3f>(proto, "translation");

    // optional fields
    addField<MFString>(proto, "info");
    addField<SFString>(proto, "name");
    addField<SFString>(proto, "version");
    addField<SFRotation>(proto, "scaleOrientation");
    addField<SFVec3f>(proto, "scale", SFVec3f::Constant(1.0));
}


void VRMLBodyLoaderImpl::checkJointProto(VRMLProto* proto)
{
    // required fields
    requireField<SFVec3f>(proto, "center");
    requireField<MFNode>(proto, "children");
    requireField<SFRotation>(proto, "rotation");
    requireField<SFVec3f>(proto, "translation");
    requireField<SFString>(proto, "jointType");
    requireField<SFInt32>(proto, "jointId");

    VRMLVariantField* field;

    field = proto->findField("jointAxis");
    if(!field){
        throw invalid_argument(_("Prototype of Humanoid must have the \"jointAxis\" field"));
    }
    if(!stdx::holds_alternative<SFString>(*field) && !stdx::holds_alternative<SFVec3f>(*field)){
        throw invalid_argument(_("The type of \"jointAxis\" field in \"Humanoid\" prototype must be SFString or SFVec3f"));
    }

    // optional fields
    addField<MFFloat>(proto, "llimit");
    addField<MFFloat>(proto, "ulimit");
    addField<MFFloat>(proto, "lvlimit");
    addField<MFFloat>(proto, "uvlimit");
    addField<MFFloat>(proto, "climit");
    addField<SFRotation>(proto, "limitOrientation");
    addField<SFString>(proto, "name");

    addField<SFFloat>(proto, "gearRatio", 1.0);
    addField<SFFloat>(proto, "rotorInertia", 0.0);
    addField<SFFloat>(proto, "rotorResistor", 0.0);
    addField<SFFloat>(proto, "torqueConst", 1.0);
    addField<SFFloat>(proto, "encoderPulse", 1.0);

    addField<SFFloat>(proto, "jointValue", 0.0);
    addField<SFVec3f>(proto, "scale", SFVec3f::Constant(1.0));

    if(proto->findField("equivalentInertia")){
        os() << _("The \"equivalentInertia\" field of the Joint node is obsolete.") << endl;
    }
}


void VRMLBodyLoaderImpl::checkSegmentProto(VRMLProto* proto)
{
    requireField<SFVec3f>(proto, "centerOfMass");
    requireField<SFFloat>(proto, "mass");
    requireField<MFFloat>(proto, "momentsOfInertia");
    addField<SFString>(proto, "name");
}


void VRMLBodyLoaderImpl::checkSurfaceProto(VRMLProto* proto)
{
    requireField<MFNode>(proto, "visual");
    requireField<MFNode>(proto, "collision");
}


void VRMLBodyLoaderImpl::checkSensorProtoCommon(VRMLProto* proto)
{
    requireField<SFInt32>(proto, "sensorId");
    requireField<SFVec3f>(proto, "translation");
    requireField<SFRotation>(proto, "rotation");
}


void VRMLBodyLoaderImpl::checkDeviceProtoCommon(VRMLProto* proto)
{
    requireField<SFVec3f>(proto, "translation");
    requireField<SFRotation>(proto, "rotation");
}


void VRMLBodyLoaderImpl::checkVisionSensorProto(VRMLProto* proto)
{
    checkDeviceProtoCommon(proto);

    requireField<SFString>(proto, "type");
    requireField<SFInt32>(proto, "width");
    requireField<SFInt32>(proto, "height");
    requireField<SFFloat>(proto, "fieldOfView");
    requireField<SFFloat>(proto, "frontClipDistance");
    requireField<SFFloat>(proto, "backClipDistance");
    addField<SFFloat>(proto, "frameRate", 30.0);
}


void VRMLBodyLoaderImpl::checkRangeSensorProto(VRMLProto* proto)
{
    checkDeviceProtoCommon(proto);

    requireField<SFFloat>(proto, "scanAngle");
    requireField<SFFloat>(proto, "scanStep");
    requireField<SFFloat>(proto, "scanRate");
    requireField<SFFloat>(proto, "maxDistance");
    addField<SFFloat>(proto, "minDistance", 0.01);
}


void VRMLBodyLoaderImpl::checkSpotLightDeviceProto(VRMLProto* proto)
{
    checkDeviceProtoCommon(proto);

    requireField<SFVec3f>(proto, "attenuation");
    requireField<SFFloat>(proto, "beamWidth");
    requireField<SFColor>(proto, "color");
    requireField<SFFloat>(proto, "cutOffAngle");
    requireField<SFVec3f>(proto, "direction");
    requireField<SFFloat>(proto, "intensity");
    requireField<SFBool>(proto, "on");
    addField<SFFloat>(proto, "cutOffExponent", 1.0);
}


void VRMLBodyLoaderImpl::checkExtraJointProto(VRMLProto* proto)
{
    requireField<SFString>(proto, "link1Name");
    requireField<SFString>(proto, "link2Name");
    requireField<SFVec3f>(proto, "link1LocalPos");
    requireField<SFVec3f>(proto, "link2LocalPos");
    requireField<SFString>(proto, "jointType");
    requireField<SFVec3f>(proto, "jointAxis");
}

        
void VRMLBodyLoaderImpl::readHumanoidNode(VRMLProtoInstance* humanoidNode)
{
    if(isVerbose) putMessage("Humanoid node");
    
    body->setModelName(humanoidNode->defName);

    MFNode& nodes = stdx::get<MFNode>(humanoidNode->fields["humanoidBody"]);

    if(nodes.size() == 0){
        throw invalid_argument(_("The Humanoid node does not have a Joint node in its \"humanoidBody\" field."));
    } else if(nodes.size() > 1){
        throw invalid_argument(_("The Humanoid node must have a unique Joint node in its \"humanoidBody\" field."));
    } 

    string info;
    readVRMLfield(humanoidNode->fields["info"], info);
    body->info()->write( "humanoid info", info );

    if(nodes[0]->isCategoryOf(PROTO_INSTANCE_NODE)){
        VRMLProtoInstance* jointNode = dynamic_cast<VRMLProtoInstance*>(nodes[0].get());
        if(jointNode && jointNode->proto->protoName == "Joint"){
            rootJointNode = jointNode;
            Matrix3 Rs = Matrix3::Identity();
            Link* rootLink = readJointNode(jointNode, Rs);

            VRMLProtoFieldMap& f = jointNode->fields;
            Vector3 defaultRootPos;
            readVRMLfield(f["translation"], defaultRootPos);
            Matrix3 defaultRootR;
            readVRMLfield(f["rotation"], defaultRootR);

            rootLink->setOffsetTranslation(defaultRootPos);
            rootLink->setOffsetRotation(defaultRootR);

            body->setRootLink(rootLink);

            // Warn empty joint ids
            if(numValidJointIds < validJointIdSet.size()){
                for(size_t i=0; i < validJointIdSet.size(); ++i){
                    if(!validJointIdSet[i]){
                        os() << format(_("Warning: Joint ID {} is not specified."), i) << endl;
                    }
                }
            }

            body->installCustomizer();
        }
    }
}


static void setShape(Link* link, SgGroup* shape, bool isVisual)
{
    SgNodePtr node;
    if(shape->empty()){
        node = new SgNode;
    } else {
        SgInvariantGroup* invariant = new SgInvariantGroup;
        if(link->Rs().isApprox(Matrix3::Identity())){
            shape->copyChildrenTo(invariant);
        } else {
            SgPosTransform* transformRs = new SgPosTransform;
            transformRs->setRotation(link->Rs());
            shape->copyChildrenTo(transformRs);
            invariant->addChild(transformRs);
        }
        node = invariant;
    }
    if(node){
        if(isVisual){
            link->setVisualShape(node);
        } else {
            link->setCollisionShape(node);
        }
    }
}


Link* VRMLBodyLoaderImpl::readJointNode(VRMLProtoInstance* jointNode, const Matrix3& parentRs)
{
    if(isVerbose) putMessage(string("Joint node") + jointNode->defName);

    Link* link = createLink(jointNode, parentRs);

    LinkInfo iLink;
    iLink.link = link;

    iLink.m = 0.0;
    iLink.c = Vector3::Zero();
    iLink.I = Matrix3::Zero();

    iLink.visualShape = new SgGroup;
    iLink.collisionShape = new SgGroup;
    iLink.isSurfaceNodeUsed = false;

    MFNode& childNodes = stdx::get<MFNode>(jointNode->fields["children"]);
    Affine3 T(Affine3::Identity());
    ProtoIdSet acceptableProtoIds;
    acceptableProtoIds.set(PROTO_JOINT);
    acceptableProtoIds.set(PROTO_SEGMENT);
    acceptableProtoIds.set(PROTO_DEVICE);
    readJointSubNodes(iLink, childNodes, acceptableProtoIds, T);

    Matrix3& I = iLink.I;
    for(size_t i=0; i < iLink.segments.size(); ++i){
        const SegmentInfo& segment = iLink.segments[i];
        const Vector3 o = segment.c - iLink.c;
        const double& x = o.x();
        const double& y = o.y();
        const double& z = o.z();
        const double& m = segment.m;
        I(0,0) +=  m * (y * y + z * z);
        I(0,1) += -m * (x * y);
        I(0,2) += -m * (x * z);
        I(1,0) += -m * (y * x);
        I(1,1) +=  m * (z * z + x * x);
        I(1,2) += -m * (y * z);
        I(2,0) += -m * (z * x);
        I(2,1) += -m * (z * y);
        I(2,2) +=  m * (x * x + y * y);
    }

    link->setMass(iLink.m);
    link->setCenterOfMass(link->Rs() * iLink.c);
    link->setInertia(link->Rs() * iLink.I * link->Rs().transpose());

    setShape(link, iLink.visualShape, true);

    if(iLink.isSurfaceNodeUsed){
        setShape(link, iLink.collisionShape, false);
    } else {
        link->setCollisionShape(link->visualShape());
    }
        
    return link;
}


Link* VRMLBodyLoaderImpl::createLink(VRMLProtoInstance* jointNode, const Matrix3& parentRs)
{
    Link* link = body->createLink();
    link->setName(jointNode->defName);
    VRMLProtoFieldMap& jf = jointNode->fields;
    
    link->setJointId(stdx::get<SFInt32>(jf["jointId"]));
    if(link->jointId() >= 0){
        if(link->jointId() >= static_cast<int>(validJointIdSet.size())){
            validJointIdSet.resize(link->jointId() + 1, false);
        }
        if(!validJointIdSet[link->jointId()]){
            ++numValidJointIds;
            validJointIdSet[link->jointId()] = true;
        } else {
            os() << format(_("Warning: Joint ID {} is duplicated."), link->jointId()) << endl;
        }
    }

    if(jointNode != rootJointNode){
        Vector3 b;
        readVRMLfield(jf["translation"], b);
        link->setOffsetTranslation(parentRs * b);
        Matrix3 R;
        readVRMLfield(jf["rotation"], R);
        link->setAccumulatedSegmentRotation(parentRs * R);
    }

    string jointType;
    readVRMLfield(jf["jointType"], jointType);
    
    if(jointType == "fixed" ){
        link->setJointType(Link::FIXED_JOINT);
    } else if(jointType == "free" ){
        link->setJointType(Link::FREE_JOINT);
    } else if(jointType == "rotate" ){
        link->setJointType(Link::ROTATIONAL_JOINT);
    } else if(jointType == "slide" ){
        link->setJointType(Link::SLIDE_JOINT);
    } else if(jointType == "pseudoContinuousTrack"){
        link->setJointType(Link::PSEUDO_CONTINUOUS_TRACK);
        link->setActuationMode(Link::JOINT_SURFACE_VELOCITY);
        os() << format(
            _("Warning: A deprecated joint type 'pseudoContinousTrack'is specified for {}."), link->name())
             << endl;
    } else {
        throw invalid_argument(format(_("JointType \"{}\" is not supported."), jointType));
    }

    if(link->jointType() == Link::FREE_JOINT || link->jointType() == Link::FIXED_JOINT){
        link->setJointAxis(Vector3::Zero());

    } else {
        Vector3 jointAxis;
        VRMLVariantField& jointAxisField = jf["jointAxis"];
        switch(stdx::get_variant_index(jointAxisField)){
        case SFSTRING:
        {
            SFString& axisLabel = stdx::get<SFString>(jointAxisField);
            if(axisLabel == "X"){
                jointAxis = Vector3::UnitX();
            } else if(axisLabel == "Y"){
                jointAxis = Vector3::UnitY();
            } else if(axisLabel == "Z"){
                jointAxis = Vector3::UnitZ();
            }
        }
        break;
        case SFVEC3F:
            readVRMLfield(jointAxisField, jointAxis);
            break;
        default:
            jointAxis = Vector3::UnitZ();
            break;
        }
        link->setJointAxis(link->Rs() * jointAxis);
    }

    double Ir, gearRatio, torqueConst, encoderPulse, rotorResistor;
    readVRMLfield(jf["rotorInertia"], Ir);
    readVRMLfield(jf["gearRatio"], gearRatio);
    readVRMLfield(jf["torqueConst"], torqueConst);
    readVRMLfield(jf["encoderPulse"], encoderPulse);
    readVRMLfield(jf["rotorResistor"], rotorResistor);

    double equivalentInertia = 0.0;
    VRMLVariantField* field = jointNode->findField("equivalentInertia");
    if(field){
        equivalentInertia = stdx::get<SFFloat>(*field);
    }
    
    if( equivalentInertia == 0.0 ){
      link->setEquivalentRotorInertia(gearRatio * gearRatio * Ir);
    } else {
      link->setEquivalentRotorInertia(equivalentInertia);
    }

    link->setInfo("rotorInertia", Ir);
    link->setInfo("gearRatio", gearRatio);
    link->setInfo("torqueConst", torqueConst);
    link->setInfo("encoderPulse", encoderPulse);
    link->setInfo("rotorResistor", rotorResistor);

    double maxlimit = numeric_limits<double>::max();

    link->setJointRange(
        getLimitValue(jf["llimit"],  -maxlimit),
        getLimitValue(jf["ulimit"],  +maxlimit));

    link->setJointVelocityRange(
        getLimitValue(jf["lvlimit"], -maxlimit),
        getLimitValue(jf["uvlimit"], +maxlimit));

    double climit = getLimitValue(jf["climit"], +maxlimit);
    link->setInfo("climit", climit);

    return link;
}    


void VRMLBodyLoaderImpl::readJointSubNodes(LinkInfo& iLink, MFNode& childNodes, const ProtoIdSet& acceptableProtoIds, const Affine3& T)
{
    for(size_t i = 0; i < childNodes.size(); ++i){
        bool doTraverse = false;
        VRMLNode* childNode = childNodes[i].get();
        if(!childNode->isCategoryOf(PROTO_INSTANCE_NODE)){
            doTraverse = true;
        } else {
            VRMLProtoInstance* protoInstance = static_cast<VRMLProtoInstance*>(childNode);
            int id = PROTO_UNDEFINED;
            const string& protoName = protoInstance->proto->protoName;
            ProtoInfoMap::iterator p = protoInfoMap.find(protoName);
            if(p == protoInfoMap.end()){
                doTraverse = true;
                childNode = protoInstance->actualNode.get();
            } else {
                id = p->second.id;
                if(!acceptableProtoIds.test(id)){
                    throw invalid_argument(format(_("{} node is not in a correct place."), protoName));
                }
                if(isVerbose){
                    messageIndent += 2;
                }
                switch(id){
                case PROTO_JOINT:
                    if(!T.matrix().isApprox(Affine3::MatrixType::Identity())){
                        throw invalid_argument(
                            format(_("Joint node \"{}\" is not in a correct place."), protoInstance->defName));
                    }
                    iLink.link->appendChild(readJointNode(protoInstance, iLink.link->Rs()));
                    break;
                case PROTO_SEGMENT:
                    readSegmentNode(iLink, protoInstance, T);
                    linkOriginalMap[iLink.link] = childNodes[i];
                    break;
                case PROTO_SURFACE:
                    readSurfaceNode(iLink, protoInstance, T);
                    break;
                case PROTO_DEVICE:
                    readDeviceNode(iLink, protoInstance, T);
                    break;
                default:
                    doTraverse = true;
                    break;
                }
                if(isVerbose){
                    messageIndent -= 2;
                }
            }
        }
        if(doTraverse && childNode->isCategoryOf(GROUPING_NODE)){
            VRMLGroup* group = static_cast<VRMLGroup*>(childNode);
            if(VRMLTransform* transform = dynamic_cast<VRMLTransform*>(group)){
                readJointSubNodes(iLink, group->getChildren(), acceptableProtoIds, T * transform->toAffine3d());
            } else {
                readJointSubNodes(iLink, group->getChildren(), acceptableProtoIds, T);
            }
        }
    }
}


void VRMLBodyLoaderImpl::readSegmentNode(LinkInfo& iLink, VRMLProtoInstance* segmentNode, const Affine3& T)
{
    if(isVerbose) putMessage(string("Segment node ") + segmentNode->defName);
    
    /*
      Mass = Sigma mass 
      C = (Sigma mass * T * c) / Mass 
      I = Sigma(R * I * Rt + G)       
      R = Rotation matrix part of T   
      G = y*y+z*z, -x*y, -x*z, -y*x, z*z+x*x, -y*z, -z*x, -z*y, x*x+y*y    
      (x, y, z ) = T * c - C
    */

    VRMLProtoFieldMap& sf = segmentNode->fields;
    SegmentInfo iSegment;
    readVRMLfield(sf["mass"], iSegment.m);
    Vector3 c;
    readVRMLfield(sf["centerOfMass"], c);
    iSegment.c = T.linear() * c + T.translation();
    if (iLink.m + iSegment.m > 0){
        iLink.c = (iSegment.c * iSegment.m + iLink.c * iLink.m) / (iLink.m + iSegment.m);
    }
    iLink.m += iSegment.m;
    iLink.segments.push_back(iSegment);
    
    Matrix3 I;
    readVRMLfield(sf["momentsOfInertia"], I);
    iLink.I.noalias() += T.linear() * I * T.linear().transpose();

    MFNode& childNodes = stdx::get<MFNode>(segmentNode->fields["children"]);
    ProtoIdSet acceptableProtoIds;
    acceptableProtoIds.set(PROTO_SURFACE);
    acceptableProtoIds.set(PROTO_DEVICE);
    readJointSubNodes(iLink, childNodes, acceptableProtoIds, T);

    SgNodePtr node = sgConverter.convert(segmentNode);
    if(node){
        if(T.isApprox(Affine3::Identity())){
            node->setName(segmentNode->defName);
            iLink.visualShape->addChild(node);
        } else {
            SgPosTransform* transform = new SgPosTransform(T);
            transform->addChild(node);
            transform->setName(segmentNode->defName);
            iLink.visualShape->addChild(transform);
        }
    } else {
        node = new SgNode;
        node->setName(segmentNode->defName);
        iLink.visualShape->addChild(node);
    }
}


void VRMLBodyLoaderImpl::readSurfaceNode(LinkInfo& iLink, VRMLProtoInstance* segmentShapeNode, const Affine3& T)
{
    if(isVerbose) putMessage(string("Surface node ") + segmentShapeNode->defName);
    
    iLink.isSurfaceNodeUsed = true;

    // check if another Surface node does not appear in the subtree
    MFNode& visualNodes = stdx::get<MFNode>(segmentShapeNode->fields["visual"]);
    ProtoIdSet acceptableProtoIds;
    readJointSubNodes(iLink, visualNodes, acceptableProtoIds, T);
    MFNode& collisionNodes = stdx::get<MFNode>(segmentShapeNode->fields["collision"]);
    readJointSubNodes(iLink, collisionNodes, acceptableProtoIds, T);

    SgGroup* group;
    SgPosTransform* transform = 0;
    if(T.isApprox(Affine3::Identity())){
        group = iLink.collisionShape;
    } else {
        transform = new SgPosTransform(T);
        group = transform;
    }
    for(size_t i=0; i < collisionNodes.size(); ++i){
        SgNodePtr node = sgConverter.convert(collisionNodes[i]);
        if(node){
            group->addChild(node);
        }
    }
    if(transform && !transform->empty()){
        transform->setName(segmentShapeNode->defName);
        iLink.collisionShape->addChild(transform);
    }
}


void VRMLBodyLoaderImpl::readDeviceNode(LinkInfo& iLink, VRMLProtoInstance* deviceNode, const Affine3& T)
{
    const string& typeName = deviceNode->proto->protoName;
    if(isVerbose) putMessage(typeName + " node " + deviceNode->defName);
    
    DeviceFactoryMap::iterator p = deviceFactories.find(typeName);
    if(p == deviceFactories.end()){
        os() << format(_("Sensor type {} is not supported.\n"), typeName) << endl;
    } else {
        DeviceFactory& factory = p->second;
        DevicePtr device = factory(deviceNode);
        if(device){
            device->setLink(iLink.link);
            const Matrix3 RsT = iLink.link->Rs();
            device->setLocalTranslation(RsT * (T * device->localTranslation()));
            device->setLocalRotation(RsT * (T.linear() * device->localRotation()));
            body->addDevice(device);

            SgNodePtr node = sgConverter.convert(deviceNode);
            if(node){
                iLink.visualShape->addChild(node);
            }
        }
    }
}


void VRMLBodyLoaderImpl::readDeviceCommonParameters(Device& device, VRMLProtoInstance* node)
{
    device.setName(node->defName);
            
    int id = -1;
    if(!checkAndReadVRMLfield(node, "deviceId", id)){
        checkAndReadVRMLfield(node, "sensorId", id);
    }
    device.setId(id);

    device.setLocalTranslation(getValue<SFVec3f>(node, "translation"));

    Matrix3 R;
    readVRMLfield(node->fields["rotation"], R);
    device.setLocalRotation(R);
}


ForceSensorPtr VRMLBodyLoaderImpl::createForceSensor(VRMLProtoInstance* node)
{
    ForceSensorPtr sensor = new ForceSensor();
    readDeviceCommonParameters(*sensor, node);

    SFVec3f f_max, t_max;
    if(checkAndReadVRMLfield(node, "maxForce", f_max)){
        sensor->F_max().head<3>() = f_max;
    }
    if(checkAndReadVRMLfield(node, "maxTorque", t_max)){
        sensor->F_max().tail<3>() = t_max;
    }
    return sensor;
}


RateGyroSensorPtr VRMLBodyLoaderImpl::createRateGyroSensor(VRMLProtoInstance* node)
{
    RateGyroSensorPtr sensor = new RateGyroSensor();
    readDeviceCommonParameters(*sensor, node);

    SFVec3f w_max;
    if(checkAndReadVRMLfield(node, "maxAngularVelocity", w_max)){
        sensor->w_max() = w_max;
    }
    return sensor;
}


AccelerationSensorPtr VRMLBodyLoaderImpl::createAccelerationSensor(VRMLProtoInstance* node)
{
    AccelerationSensorPtr sensor = new AccelerationSensor();
    readDeviceCommonParameters(*sensor, node);

    SFVec3f dv_max;
    if(checkAndReadVRMLfield(node, "maxAngularVelocity", dv_max)){
        sensor->dv_max() = dv_max;
    }
    return sensor;
}


CameraPtr VRMLBodyLoaderImpl::createCamera(VRMLProtoInstance* node)
{
    CameraPtr camera;
    RangeCamera* range = 0;
    
    const SFString& type = stdx::get<SFString>(node->fields["type"]);
    if(type == "COLOR"){
        camera = new Camera;
        camera->setImageType(Camera::COLOR_IMAGE);
    } else if(type == "DEPTH"){
        range = new RangeCamera;
        range->setOrganized(true);
        range->setImageType(Camera::NO_IMAGE);
    } else if(type == "COLOR_DEPTH"){
        range = new RangeCamera;
        range->setOrganized(true);
        range->setImageType(Camera::COLOR_IMAGE);
    } else if(type == "POINT_CLOUD"){
        range = new RangeCamera;
        range->setOrganized(false);
        range->setImageType(Camera::NO_IMAGE);
    } else if(type == "COLOR_POINT_CLOUD"){
        range = new RangeCamera;
        range->setOrganized(false);
        range->setImageType(Camera::COLOR_IMAGE);
    }

    if(range){
        camera = range;
    } else {
        camera = new Camera;
    }
        
    readDeviceCommonParameters(*camera, node);
    
    bool on = true;
    if(checkAndReadVRMLfield(node, "on", on)){
        camera->on(on);
    }
    camera->setResolution(getValue<SFInt32>(node, "width"), getValue<SFInt32>(node, "height"));
    camera->setFieldOfView(getValue<SFFloat>(node, "fieldOfView"));
    camera->setNearClipDistance(getValue<SFFloat>(node, "frontClipDistance"));
    camera->setFarClipDistance(getValue<SFFloat>(node, "backClipDistance"));
    camera->setFrameRate(getValue<SFFloat>(node, "frameRate"));
    
    return camera;
}


RangeSensorPtr VRMLBodyLoaderImpl::createRangeSensor(VRMLProtoInstance* node)
{
    RangeSensorPtr rangeSensor = new RangeSensor;
    
    readDeviceCommonParameters(*rangeSensor, node);
    
    bool on = true;
    if(checkAndReadVRMLfield(node, "on", on)){
        rangeSensor->on(on);
    }
    rangeSensor->setYawRange(getValue<SFFloat>(node, "scanAngle"));
    rangeSensor->setPitchRange(0.0);
    const double scanStep = getValue<SFFloat>(node, "scanStep");
    rangeSensor->setYawStep(scanStep);
    rangeSensor->setMinDistance(getValue<SFFloat>(node, "minDistance"));
    rangeSensor->setMaxDistance(getValue<SFFloat>(node, "maxDistance"));
    rangeSensor->setFrameRate(getValue<SFFloat>(node, "scanRate"));
    
    return rangeSensor;
}


void VRMLBodyLoaderImpl::readLightDeviceCommonParameters(Light& light, VRMLProtoInstance* node)
{
    readDeviceCommonParameters(light, node);
    
    light.on(getValue<SFBool>(node, "on"));
    light.setColor(getValue<SFColor>(node, "color"));
    light.setIntensity(getValue<SFFloat>(node, "intensity"));
}


SpotLightPtr VRMLBodyLoaderImpl::createSpotLight(VRMLProtoInstance* node)
{
    SpotLightPtr light = new SpotLight();
    
    readLightDeviceCommonParameters(*light, node);

    light->setDirection(getValue<SFVec3f>(node, "direction"));
    light->setBeamWidth(getValue<SFFloat>(node, "beamWidth"));
    light->setCutOffAngle(getValue<SFFloat>(node, "cutOffAngle"));
    light->setCutOffExponent(getValue<SFFloat>(node, "cutOffExponent"));
    SFVec3f attenuation = getValue<SFVec3f>(node, "attenuation");
    light->setConstantAttenuation(attenuation[0]);
    light->setLinearAttenuation(attenuation[1]);
    light->setQuadraticAttenuation(attenuation[2]);

    return light;
}


void VRMLBodyLoaderImpl::setExtraJoints()
{
    for(size_t i=0; i < extraJointNodes.size(); ++i){

        VRMLProtoFieldMap& f = extraJointNodes[i]->fields;
        ExtraJoint joint;

        string link1Name, link2Name;
        readVRMLfield(f["link1Name"], link1Name);
        readVRMLfield(f["link2Name"], link2Name);
        joint.link[0] = body->link(link1Name);
        joint.link[1] = body->link(link2Name);
        joint.body[0] = joint.body[1] = body;

        for(int j=0; j < 2; ++j){
            if(!joint.link[j]){
                throw invalid_argument(
                    format(_("Field \"link{}Name\" of a ExtraJoint node does not specify a valid link name"), (j+1)));
            }
        }

        SFString& jointType = stdx::get<SFString>(f["jointType"]);
        if(jointType == "piston"){
            joint.type = ExtraJoint::EJ_PISTON;
            joint.axis = stdx::get<SFVec3f>(f["jointAxis"]);
        } else if(jointType == "ball"){
            joint.type = ExtraJoint::EJ_BALL;
        } else {
            throw invalid_argument(format(_("JointType \"{}\" is not supported."), jointType));
        }
            
        readVRMLfield(f["link1LocalPos"], joint.point[0]);
        readVRMLfield(f["link2LocalPos"], joint.point[1]);

        body->addExtraJoint(joint);
    }
}
