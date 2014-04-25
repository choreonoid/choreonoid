
#ifndef CNOID_OPENHRP_PLUGIN_ONLINE_VIEWER_SERVER_H_INCLUDED
#define CNOID_OPENHRP_PLUGIN_ONLINE_VIEWER_SERVER_H_INCLUDED

#ifdef OPENHRP_3_0
#include <cnoid/corba/OpenHRP/3.0/OnlineViewer.hh>
#elif OPENHRP_3_1
#include <cnoid/corba/OpenHRP/3.1/OnlineViewer.hh>
#endif

namespace cnoid {

class MessageView;
class OnlineViewerServerImpl;

class OnlineViewerServer : virtual public POA_OpenHRP::OnlineViewer,
                           virtual public PortableServer::RefCountServantBase
{
public:
    OnlineViewerServer();
    virtual ~OnlineViewerServer();

    virtual void load(const char* name, const char* url);
    virtual void update(const OpenHRP::WorldState& state);
    virtual void clearLog();
    virtual void clearData();
    virtual void drawScene(const OpenHRP::WorldState& state);
    virtual void setLineWidth(::CORBA::Float width);
    virtual void setLineScale(::CORBA::Float scale);
    virtual ::CORBA::Boolean getPosture(const char* robotId, OpenHRP::DblSequence_out posture);

#ifdef OPENHRP_3_1
    virtual void setLogName(const char* name);
#endif

private:
    OnlineViewerServerImpl* impl;
};
}

#endif
