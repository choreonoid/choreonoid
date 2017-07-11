/**
   @author Shizuko Hattori
*/

#include "RTMPointCloudIOItem.h"
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/Config>
#include <cnoid/RangeSensor>
#include <cnoid/RangeCamera>
#include <cnoid/MessageView>
#include <cnoid/OpenRTMUtil>
#include <cnoid/PointSetItem>
#include <cnoid/LazyCaller>
#include <cnoid/corba/PointCloud.hh>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/InterfaceDataTypes.hh>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;
using namespace RTC;


namespace cnoid {

class PointCloudInPort
{
public:
    InPort<PointCloudTypes::PointCloud> inPort;
    PointCloudTypes::PointCloud timedPointCloud;
    string portName;
    RangeCameraPtr rangeCamera;

    PointCloudInPort(RangeCamera* rangeCamera, const string& name)
        : rangeCamera(rangeCamera),
          portName(name),
          inPort(name.c_str(), timedPointCloud){ };

};
typedef std::shared_ptr<PointCloudInPort> PointCloudInPortPtr;

class RangeDataInPort
{
public:
    InPort<RTC::RangeData> inPort;
    RangeData timedRangeData;
    string portName;
    RangeSensorPtr rangeSensor;

    RangeDataInPort(RangeSensor* rangeSensor, const string& name)
        : rangeSensor(rangeSensor),
          portName(name),
          inPort(name.c_str(), timedRangeData){ };
};
typedef std::shared_ptr<RangeDataInPort> RangeDataInPortPtr;


class PointCloudIORTC : public DataFlowComponentBase {
public:

    typedef map<string, PointCloudInPortPtr> PointCloudInPortMap;
    PointCloudInPortMap pointCloudInPorts;

    typedef map<string, RangeDataInPortPtr> RangeDataInPortMap;
    RangeDataInPortMap rangeDataInPorts;

    struct PointCloudField {
        unsigned long offset;
        PointCloudTypes::DataType type;
        unsigned long count;
    };
    std::map<string, PointCloudField> pointCloudField;

    static void registerFactory(RTC::Manager* manager, const char* componentTypeName);

    PointCloudIORTC(RTC::Manager* manager);
    ~PointCloudIORTC(){ };

    bool createPort( DeviceList<RangeCamera> cameras, vector<string> pointCloudPortNames,
            DeviceList<RangeSensor> rangeSensors, vector<string> rangeSensorPortNames );
    float readFloatPointCloudData(unsigned char* src, PointCloudField& fe, bool is_bigendian);

    virtual ReturnCode_t onInitialize();
    virtual ReturnCode_t onActivated(UniqueId ec_id);
    virtual ReturnCode_t onDeactivated(UniqueId ec_id);
    virtual ReturnCode_t onExecute(UniqueId ec_id);
};


void PointCloudIORTC::registerFactory(RTC::Manager* manager, const char* componentTypeName)
{
  static const char* pointCloudIORTC_spec[] = {
    "implementation_id", "PointCloudIO",
    "type_name",         "PointCloudIO",
    "description",       "This component is the pointCloud visualization component.",
    "version",           CNOID_VERSION_STRING,
    "vendor",            "AIST",
    "category",          "choreonoid",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    "conf.default.debugLevel", "0",
    ""
  };

  RTC::Properties profile(pointCloudIORTC_spec);
  profile.setDefault("type_name", componentTypeName);
  manager->registerFactory(profile,
                           RTC::Create<PointCloudIORTC>,
                           RTC::Delete<PointCloudIORTC>);
}


PointCloudIORTC::PointCloudIORTC(Manager* manager)
    : DataFlowComponentBase(manager)
{
    pointCloudField.clear();
    pointCloudField["x"].offset = 0;
    pointCloudField["x"].type = PointCloudTypes::FLOAT32;
    pointCloudField["x"].count = 4;
    pointCloudField["y"].offset = 4;
    pointCloudField["y"].type = PointCloudTypes::FLOAT32;
    pointCloudField["y"].count = 4;
    pointCloudField["z"].offset = 8;
    pointCloudField["z"].type = PointCloudTypes::FLOAT32;
    pointCloudField["z"].count = 4;
    pointCloudField["r"].offset = 12;
    pointCloudField["r"].type = PointCloudTypes::UINT8;
    pointCloudField["r"].count = 1;
    pointCloudField["g"].offset = 13;
    pointCloudField["g"].type = PointCloudTypes::UINT8;
    pointCloudField["g"].count = 1;
    pointCloudField["b"].offset = 14;
    pointCloudField["b"].type = PointCloudTypes::UINT8;
    pointCloudField["b"].count = 1;
}


bool PointCloudIORTC::createPort(DeviceList<RangeCamera> cameras, vector<string> pointCloudPortNames,
        DeviceList<RangeSensor> rangeSensors, vector<string> rangeSensorPortNames)
{
    bool ret = true;

    for(int i=0; i<cameras.size(); i++){
        PointCloudInPort* pointCloudInPort = new PointCloudInPort(cameras[i], pointCloudPortNames[i]);
        if(!addInPort(pointCloudInPort->portName.c_str(), pointCloudInPort->inPort))
            ret = false;
        else
            pointCloudInPorts.insert(make_pair(pointCloudInPort->portName, PointCloudInPortPtr(pointCloudInPort)));
    }

    for(int i=0; i<rangeSensors.size(); i++){
        RangeDataInPort* rangeDataInPort = new RangeDataInPort(rangeSensors[i], rangeSensorPortNames[i]);
        if(!addInPort(rangeDataInPort->portName.c_str(), rangeDataInPort->inPort))
            ret = false;
        else
            rangeDataInPorts.insert(make_pair(rangeDataInPort->portName, RangeDataInPortPtr(rangeDataInPort)));
    }

    return ret;
}


ReturnCode_t PointCloudIORTC::onInitialize()
{
    return RTC_OK;
}


ReturnCode_t PointCloudIORTC::onActivated(UniqueId ec_id)
{
     return RTC_OK;
}

ReturnCode_t PointCloudIORTC::onDeactivated(UniqueId ec_id)
{
    return RTC_OK;
}


float PointCloudIORTC::readFloatPointCloudData(unsigned char* src, PointCloudField& fe, bool is_bigendian)
{
    unsigned char* src_;
    unsigned char buf[8];
    if(is_bigendian){
        for(int i=0; i<fe.count; i++)
            buf[i] = src[fe.count-1-i];
        src_ = buf;
    }else
        src_ = src;

    switch(fe.type){
    case PointCloudTypes::FLOAT32 :
        return *((float *)src_);

    case PointCloudTypes::FLOAT64 :
        return *((double *)src_);
    }
    return 0;

}


ReturnCode_t PointCloudIORTC::onExecute(UniqueId ec_id)
{
    for(PointCloudInPortMap::iterator it = pointCloudInPorts.begin();
            it != pointCloudInPorts.end(); it++){
        InPort<PointCloudTypes::PointCloud>& inport_ = it->second->inPort;
        if(inport_.isNew()){
            do {
                inport_.read();
            }while(inport_.isNew());

            PointCloudTypes::PointCloud& timedPointCloud = it->second->timedPointCloud;
            RangeCamera* rangeCamera = it->second->rangeCamera;

            int numPoints = timedPointCloud.height * timedPointCloud.width;
            bool rgb = false;
            for(int i=0; i<timedPointCloud.fields.length(); i++){
                string name = string(timedPointCloud.fields[i].name);
                pointCloudField[name].offset = timedPointCloud.fields[i].offset;
                pointCloudField[name].type = timedPointCloud.fields[i].data_type;
                pointCloudField[name].count = timedPointCloud.fields[i].count;
                if(i==5)
                    rgb = true;
            }
            unsigned char* src = (unsigned char*)timedPointCloud.data.get_buffer();
            std::shared_ptr<RangeCamera::PointData> tmpPoints = std::make_shared< vector<Vector3f> >();
            std::shared_ptr<Image> tmpImage;
            unsigned char* pixels = 0;
            if(rgb){
                tmpImage = std::make_shared<Image>();
                tmpImage->setSize(timedPointCloud.width, timedPointCloud.height, 3);
                pixels = tmpImage->pixels();
            }

            tmpPoints->clear();
            tmpPoints->reserve(numPoints);

            PointCloudField& fex = pointCloudField["x"];
            PointCloudField& fey = pointCloudField["y"];
            PointCloudField& fez = pointCloudField["z"];
            PointCloudField& fer = pointCloudField["r"];
            PointCloudField& feg = pointCloudField["g"];
            PointCloudField& feb = pointCloudField["b"];

            for(size_t i=0; i<numPoints; i++, src+=timedPointCloud.point_step){
                Vector3f point;
                point.x() = readFloatPointCloudData(&src[fex.offset], fex, timedPointCloud.is_bigendian );
                point.y() = readFloatPointCloudData(&src[fey.offset], fey, timedPointCloud.is_bigendian );
                point.z() = readFloatPointCloudData(&src[fez.offset], fez, timedPointCloud.is_bigendian );
                tmpPoints->push_back(point);

                if(rgb && pixels){
                    pixels[0] = src[fer.offset];
                    pixels[1] = src[feg.offset];
                    pixels[2] = src[feb.offset];
                    pixels += 3;
                }
            }
            rangeCamera->setPoints(tmpPoints);
            if(rgb && pixels)
                rangeCamera->setImage(tmpImage);

            callLater(bind(&RangeCamera::notifyStateChange,rangeCamera));
        }
    }

    for(RangeDataInPortMap::iterator it = rangeDataInPorts.begin();
            it != rangeDataInPorts.end(); it++){
        InPort<RTC::RangeData>& inport_ = it->second->inPort;
        if(inport_.isNew()){
            do {
                inport_.read();
            }while(inport_.isNew());

            RangeData& timedRangeData = it->second->timedRangeData;
            RangeSensor* rangeSensor = it->second->rangeSensor;

            double* src = timedRangeData.ranges.get_buffer();
            int numPoints = timedRangeData.ranges.length();
            std::shared_ptr<RangeSensor::RangeData> tmpRangeData = std::make_shared< vector<double> >();
            tmpRangeData->clear();
            for(int i=0; i < numPoints; i++)
                tmpRangeData->push_back(src[i]);

            rangeSensor->setRangeData(tmpRangeData);

            callLater(bind(&RangeSensor::notifyStateChange,rangeSensor));
        }
    }
    return RTC::RTC_OK;
}


class RTMPointCloudIOItemImpl
{
public:
    RTMPointCloudIOItem* self;
    MessageView* mv;

    Body* body;
    DeviceList<RangeCamera> cameras;
    DeviceList<RangeSensor> rangeSensors;
    PointCloudIORTC* pointCloudIORTC;
    string componentName;
    vector<string> pointCloudPortNames;
    vector<string> rangeSensorPortNames;

    RTMPointCloudIOItemImpl(RTMPointCloudIOItem* self);
    RTMPointCloudIOItemImpl(RTMPointCloudIOItem* self, const RTMPointCloudIOItemImpl& org);
    ~RTMPointCloudIOItemImpl();

    bool onComponentNamePropertyChanged(const string& name);
    bool onPointCloudPortNamePropertyChanged(int i, const string& name);
    bool onRangeSensorPortNamePropertyChanged(int i, const string& name);

    bool recreateRTC();
    bool createRTC();
    void deleteRTC();
    void changeOwnerBodyItem(BodyItem* bodyItem);
    void doPutProperties(PutPropertyFunction& putProperty);
    void store(Archive& archive);
    void restore(const Archive& archive);
};


void RTMPointCloudIOItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<RTMPointCloudIOItem>(N_("RTMPointCloudIOItem"));
    ext->itemManager().addCreationPanel<RTMPointCloudIOItem>();
}


RTMPointCloudIOItem::RTMPointCloudIOItem()
{
    impl = new RTMPointCloudIOItemImpl(this);
    bodyItem = 0;
}


RTMPointCloudIOItemImpl::RTMPointCloudIOItemImpl(RTMPointCloudIOItem* self)
    : self(self),
      mv(MessageView::instance())
{
    pointCloudIORTC = 0;
    body = 0;
}


RTMPointCloudIOItem::RTMPointCloudIOItem(const RTMPointCloudIOItem& org)
    : Item(org)
{
    impl = new RTMPointCloudIOItemImpl(this, *org.impl);
    bodyItem = 0;
}


RTMPointCloudIOItemImpl::RTMPointCloudIOItemImpl(RTMPointCloudIOItem* self, const RTMPointCloudIOItemImpl& org)
    : RTMPointCloudIOItemImpl(self)
{
    pointCloudIORTC = 0;
    body = 0;
}


RTMPointCloudIOItem::~RTMPointCloudIOItem()
{
    delete impl;
}


RTMPointCloudIOItemImpl::~RTMPointCloudIOItemImpl()
{
    deleteRTC();
}


void RTMPointCloudIOItem::onPositionChanged()
{
    BodyItem* ownerBodyItem = findOwnerItem<BodyItem>();
    if(ownerBodyItem){
        if(bodyItem != ownerBodyItem){
            impl->changeOwnerBodyItem(ownerBodyItem);
            bodyItem = ownerBodyItem;
        }
    }else{
        impl->deleteRTC();
        impl->body = 0;
        bodyItem = 0;
    }
}


void RTMPointCloudIOItemImpl::changeOwnerBodyItem(BodyItem* bodyItem)
{
    body = bodyItem->body();
    cameras = body->devices<RangeCamera>();
    rangeSensors = body->devices<RangeSensor>();

    if(componentName.empty())
        componentName = self->name();

    pointCloudPortNames.resize(cameras.size());
    for(size_t i=0; i<cameras.size(); i++){
        if(pointCloudPortNames[i].empty())
            pointCloudPortNames[i] = cameras[i]->name();
    }

    rangeSensorPortNames.resize(rangeSensors.size());
    for(size_t i=0; i<rangeSensors.size(); i++){
        if(rangeSensorPortNames[i].empty())
            rangeSensorPortNames[i] = rangeSensors[i]->name();
    }

   //self->notifyUpdate();

    deleteRTC();
    createRTC();
}


void RTMPointCloudIOItem::onDisconnectedFromRoot()
{
    impl->deleteRTC();
}


bool RTMPointCloudIOItemImpl::createRTC()
{
    RTC::Manager* rtcManager = &RTC::Manager::instance();
    PointCloudIORTC::registerFactory(rtcManager, componentName.c_str());

    /// --- create RTC ---
    boost::format param("PointCloudIO?instance_name=%1%&exec_cxt.periodic.type=ChoreonoidExecutionContext&exec_cxt.periodic.rate=1000000");
    RTC::RtcBase* rtc = createManagedRTC(str(param % componentName).c_str());
    if(!rtc){
        MessageView::instance()->putln(fmt(_("RTC \"%1%\" cannot be created.")) % componentName);
        return false;
    }
    MessageView::instance()->putln(fmt(_("RTC \"%1%\" has been created.")) % componentName);

    pointCloudIORTC = dynamic_cast<PointCloudIORTC*>(rtc);
    pointCloudIORTC->createPort( cameras, pointCloudPortNames, rangeSensors, rangeSensorPortNames );

    return true;
}


void RTMPointCloudIOItemImpl::deleteRTC()
{
    if(pointCloudIORTC){
        pointCloudIORTC->exit();
        RTC::Manager::instance().cleanupComponents();
        pointCloudIORTC = 0;
    }
}


bool RTMPointCloudIOItemImpl::recreateRTC()
{
    if(body){
        deleteRTC();
        return createRTC();
    }

    return true;
}


Item* RTMPointCloudIOItem::doDuplicate() const
{
    return new RTMPointCloudIOItem(*this);
}


void RTMPointCloudIOItem::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->doPutProperties(putProperty);
}


void RTMPointCloudIOItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("ComponentName"), componentName,
            std::bind(&RTMPointCloudIOItemImpl::onComponentNamePropertyChanged, this, _1));
    for(int i=0; i<pointCloudPortNames.size(); i++){
        putProperty(_("PointCloudPortName")+to_string(i), pointCloudPortNames[i],
                std::bind(&RTMPointCloudIOItemImpl::onPointCloudPortNamePropertyChanged, this, i, _1));
    }
    for(int i=0; i<rangeSensorPortNames.size(); i++){
        putProperty(_("rangeSensorPortName")+to_string(i), rangeSensorPortNames[i],
                std::bind(&RTMPointCloudIOItemImpl::onRangeSensorPortNamePropertyChanged, this, i, _1));
    }
}


bool RTMPointCloudIOItemImpl::onComponentNamePropertyChanged(const string& name)
{
    componentName = name;

    recreateRTC();
    return true;
}


bool RTMPointCloudIOItemImpl::onPointCloudPortNamePropertyChanged(int i, const string& name)
{
    pointCloudPortNames[i] = name;

    recreateRTC();
    return true;

}


bool RTMPointCloudIOItemImpl::onRangeSensorPortNamePropertyChanged(int i, const string& name)
{
    rangeSensorPortNames[i] = name;

    recreateRTC();
    return true;

}


bool RTMPointCloudIOItem::store(Archive& archive)
{
    impl->store(archive);
    return true;
}


void RTMPointCloudIOItemImpl::store(Archive& archive)
{
    archive.write("componentName", componentName);

    Listing& list0 = *archive.createFlowStyleListing("pointCloudPortNames");
    for(size_t i=0; i<pointCloudPortNames.size(); i++){
        list0.append(pointCloudPortNames[i]);
    }

    Listing& list1 = *archive.createFlowStyleListing("rangeSensorPortNames");
    for(size_t i=0; i<rangeSensorPortNames.size(); i++){
        list1.append(rangeSensorPortNames[i]);
    }
}

bool RTMPointCloudIOItem::restore(const Archive& archive)
{
    impl->restore(archive);
    return true;
}


void RTMPointCloudIOItemImpl::restore(const Archive& archive)
{
    archive.read("componentName", componentName);

    const Listing& list0 = *archive.findListing("pointCloudPortNames");
    if(list0.isValid()){
        pointCloudPortNames.clear();
        for(size_t i=0; i<list0.size(); i++)
            pointCloudPortNames.push_back(list0[i]);
    }

    const Listing& list1 = *archive.findListing("rangeSensorPortNames");
    if(list1.isValid()){
        rangeSensorPortNames.clear();
        for(size_t i=0; i<list1.size(); i++)
            rangeSensorPortNames.push_back(list1[i]);
    }

}


}
