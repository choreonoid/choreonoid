/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "YAMLBodyLoader.h"
#include "Body.h"
#include "ForceSensor.h"
#include "RateGyroSensor.h"
#include "AccelerationSensor.h"
#include "Camera.h"
#include "RangeCamera.h"
#include "RangeSensor.h"
#include "PointLight.h"
#include "SpotLight.h"

//#include <cnoid/FileUtil>
//#include <cnoid/Exception>
//#include <cnoid/EasyScanner>

#include <cnoid/YAMLReader>
#include <cnoid/NullOut>
#include <boost/dynamic_bitset.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;


namespace {

typedef boost::function<DevicePtr(Mapping* node)> DeviceFactory;
typedef map<string, DeviceFactory> DeviceFactoryMap;
DeviceFactoryMap deviceFactories;

}

namespace cnoid {

class YAMLBodyLoaderImpl
{
public:

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
        
    YAMLReader reader;
    Body* body;


    dynamic_bitset<> validJointIdSet;
    int numValidJointIds;

    int divisionNumber;
    ostream* os_;
    bool isVerbose;

    ostream& os() { return *os_; }

    void putMessage(const std::string& message){
        os() << string(messageIndent, ' ') + message + "\n";
    }
        
    YAMLBodyLoaderImpl();
    ~YAMLBodyLoaderImpl();
};

}


YAMLBodyLoader::YAMLBodyLoader()
{
    impl = new YAMLBodyLoaderImpl();
}


YAMLBodyLoaderImpl::YAMLBodyLoaderImpl()
{
    divisionNumber = 20;
    isVerbose = false;
    body = 0;
    os_ = &nullout();
    
    if(deviceFactories.empty()){
        deviceFactories["ForceSensor"]        = &YAMLBodyLoaderImpl::createForceSensor;
        deviceFactories["RateGyroSensor"]     = &YAMLBodyLoaderImpl::createRateGyroSensor;
        deviceFactories["AccelerationSensor"] = &YAMLBodyLoaderImpl::createAccelerationSensor;
        deviceFactories["RangeSensor"]        = &YAMLBodyLoaderImpl::createRangeSensor;
        deviceFactories["VisionSensor"]       = &YAMLBodyLoaderImpl::createCamera;
        deviceFactories["SpotLightDevice"]    = &YAMLBodyLoaderImpl::createSpotLight;
    }
}


YAMLBodyLoader::~YAMLBodyLoader()
{
    delete impl;
}


YAMLBodyLoaderImpl::~YAMLBodyLoaderImpl()
{

}


const char* YAMLBodyLoader::format() const
{
    return "Choreonoid-Body-Ver1.0";
}


void YAMLBodyLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
}


void YAMLBodyLoader::setVerbose(bool on)
{
    impl->isVerbose = on;
}


void YAMLBodyLoader::enableShapeLoading(bool on)
{

}
    

void YAMLBodyLoader::setDefaultDivisionNumber(int n)
{
    impl->divisionNumber = n;
}


bool YAMLBodyLoader::load(Body* body, const std::string& filename)
{
    return impl->load(body, filename);
}


bool YAMLBodyLoaderImpl::load(Body* body, const std::string& filename)
{
    bool result = false;

    this->body = body;

    body->clearDevices();
    body->clearExtraJoints();

    rootJointNode = 0;
    extraJointNodes.clear();
    validJointIdSet.clear();
    numValidJointIds = 0;
    
    try {
        YAMLReader reader;
        top = reader.loadDocument(filename)->toMapping();

        if(readBody(top)){
            if(body->modelName().empty()){
                body->setModelName(getBasename(filename));
            }
            result = true;
        }
        
    } catch(const ValueNode::Exception& ex){
        os() << ex.message();
    } catch(const nonexistent_key_error& error){
        if(const std::string* message = get_error_info<error_info_message>(error)){
            os() << *message << endl;
        }
    }
        
    os().flush();
        
    return result;
}


bool YAMLBodyLoaderImpl::readBody(Mapping* topNode)
{
    double version = topNode->get("bodyFormatVersion", 1.0);
    if(version > 1.0){
        topNode->throwException(_("This version of the body format is not supported."));
    }

    string symbol;
    if(topNode->read("name", symbol)){
        body->setModelName(symbol);
    }

    Listing& linkNodes = *topNode->find("links")->toListing();

    for(int i=0; i < linkNodes.size(); ++i){
        Mapping& linkNode = linkNodes[i].toMapping();
        readLink(linkNode);
    }

    string rootLink;
    if(!topNode->read("rootLink", rootLink)){
        topNode->throwException(_("There is no \"rootLink\" value for specifying the root link."));
    }

    LinkMap::iterator p = linkMap.find(rootLink);
    if(p == linkMap.end()){
        topNode["rootLink"].throwException(
            format(_("Link \"%1%\" specified as \"rootLink\" is not defined."))
            % rootLink);
    }
            
    body->setRootLink(rootLink);

    // Warn empty joint ids
    if(numValidJointIds < validJointIdSet.size()){
        for(size_t i=0; i < validJointIdSet.size(); ++i){
            if(!validJointIdSet[i]){
                os() << str(format("Warning: Joint ID %1% is not specified.") % i) << endl;
            }
        }
    }
        
    body->installCustomizer();

    return true;
}


void YAMLBodyLoaderImpl::readLink(Mapping& linkNode)
{
    Link* link = body->createLink();

    try {
        link->setName(linkNode["name"].toString());
        LinkMap::iterator p = linkMap.find(link->name());
        if(!linkMap.insert(make_par(link->name, link)).second){
            linkNode["name"]->throwException(format(_("Duplicated link name \"%1%\"")) % link->name());
        }
        
        string jointType;
        if(linkNode.read("jointType", jointType)){
            if(jointType == "revolute"){
                link->setJointType(Link::REVOLUTE_JOINT);
            } else if(jointType == "slide"){
                link->setJointType(Link::SLIDE_JOINT);
            } else if(jointType == "free"){
                link->setJointType(Link::FREE_JOINT);
            } else if(jointType == "fixed"){
                link->setJointType(Link::FIXED_JOINT);
            } else if(jointType == "crawler"){
                link->setJointType(Link::CRAWLER_JOINT);
            }
        }
        
        int jointId;
        if(linkNode.read("jointId", jointId)){
            link->setJointId(jointId);
        }
        
        /*
          \todo A link node may contain more than one rigid bodies and
          the paremeters of them must be summed up
        */
        double m;
        if(linkNode.read("mass")){
            link->setMass(m);
        }
        Matrix3 I;
        if(read(linkNode, "inertia", I)){
            link->setInertia(I);
        }
        
        ValueNode& node = *linkNode.find("elements");
        if(node.isValid()){
            Affine3 T = Affine3::Identity();
            readElements(node, T, link);
        }
    } catch(...){
        delete link;
        throw;
    }
}


void YAMLBodyLoaderImpl::readElements(ValueNode& node, const Affine3& T, Link* link)
{

}
