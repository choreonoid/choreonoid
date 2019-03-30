/**
   \file
   \author Shizuko Hattori
*/

#ifndef CNOID_OPENHRP_MODELLOADER_BODYINFO_IMPL_H
#define CNOID_OPENHRP_MODELLOADER_BODYINFO_IMPL_H

#include <cnoid/corba/OpenHRPModelLoader/ModelLoader.hh>
#include <cnoid/corba/OpenHRPModelLoader/ViewSimulator.hh>
#include "ShapeSetInfo_impl.h"
#include <cnoid/BodyLoader>
#include <cnoid/Body>
#include <cnoid/ForceSensor>
#include <string>
#include <iostream>

namespace cnoid {

class BodyInfo_impl : public virtual POA_OpenHRP::BodyInfo,
                      public virtual ShapeSetInfo_impl
{
public:

    BodyInfo_impl(PortableServer::POA_ptr poa);
    virtual ~BodyInfo_impl();

    virtual char* name() override;
    virtual char* url() override;
    virtual OpenHRP::StringSequence* info() override;
    virtual OpenHRP::LinkInfoSequence* links() override;
    virtual OpenHRP::AllLinkShapeIndexSequence* linkShapeIndices() override;
    virtual OpenHRP::ExtraJointInfoSequence* extraJoints() override;

    void loadModelFile(const std::string& filename);
    void setMessageSink(std::ostream& os_) { os = &os_; };

private :
    std::ostream* os;
    BodyPtr body;
    std::string name_;
    std::string url_;
    OpenHRP::StringSequence info_;
    OpenHRP::LinkInfoSequence links_;
    OpenHRP::AllLinkShapeIndexSequence linkShapeIndices_;
    OpenHRP::ExtraJointInfoSequence extraJoints_;

    void setLinks();
    void setLink(OpenHRP::LinkInfo& linkInfo, Link& link);
    void setSegment(OpenHRP::LinkInfo& linkInfo, Link& link);
    void setHwc(OpenHRP::LinkInfo& linkInfo, Link& link);
    void setDevices();
    void setSensor(Device* device);
    void setLight(Device* device);
    void setExtraJoints();
    void setExtraJoint(OpenHRP::ExtraJointInfo& extraJoiontInfo, ExtraJoint& joint);
};

}

#endif
