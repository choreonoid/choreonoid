#ifndef CNOID_OPENRTM_PLUGIN_RT_SYSTEM_ITEM_EX_H
#define CNOID_OPENRTM_PLUGIN_RT_SYSTEM_ITEM_EX_H

#include <cnoid/Item>
#include <cnoid/EigenUtil>
#include <cnoid/IdPair>
#include <cnoid/CorbaUtil>
#include <rtm/idl/RTC.hh>
#include <rtm/NVUtil.h>
#include <QPoint>
#include <list>
#include <string>
#include "../RTSystem.h"
#include "../ProfileHandler.h"
#include "../exportdecl.h"

namespace cnoid {

class RTSComp;
class RTSystemItemEx;
class RTSystemItemExImpl;

typedef ref_ptr<RTSPort> RTSPortPtr;

/*!
 * @brief This is the RTSystem item.
 */
class CNOID_EXPORT RTSystemItemEx : public Item, public RTSystem
{
public:
    typedef cnoid::IdPair<RTSPort*> RTSPortPair;
    typedef std::map<RTSPortPair, RTSConnectionPtr> RTSConnectionMap;
    RTSystemItemEx();
    RTSystemItemEx(const RTSystemItemEx& org);
    virtual ~RTSystemItemEx();
    static void initializeClass(ExtensionManager* ext);

    RTSComp* addRTSComp(const std::string& name, const QPointF& pos);
    virtual RTSComp* addRTSComp(const NamingContextHelper::ObjectInfo& info, const QPointF& pos) override;
    void deleteRTSComp(const std::string& name);
    virtual RTSComp* nameToRTSComp(const std::string& name) override;
    std::map<std::string, RTSCompPtr>& rtsComps();
    virtual RTSConnection* addRTSConnection(
        const std::string& id, const std::string& name, RTSPort* sourcePort, RTSPort* targetPort,
        const std::vector<NamedValuePtr>& propList, const Vector2 pos[]) override;
    void RTSCompToConnectionList(const RTSComp* rtsComp, std::list<RTSConnection*>& rtsConnectionList, int mode = 0);
    RTSConnectionMap& rtsConnections();
    void disconnectAndRemoveConnection(RTSConnection* connection);

    bool loadRtsProfile(const std::string& filename);
    bool saveRtsProfile(const std::string& filename);

    virtual void setVendorName(const std::string& name) override;
    virtual void setVersion(const std::string& version) override;

    enum StateCheckMode {
        POLLING_MODE,
        MANUAL_MODE,
#if defined(OPENRTM_VERSION12)
        OBSERVER_MODE,
#endif
        N_STATE_CHECK_MODES
    };

    int stateCheckMode() const;

    void checkStatus();
    bool isCheckAtLoading();
    void onActivated();

    SignalProxy<void(bool)> sigStatusUpdate();

protected:
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    RTSystemItemExImpl* impl;

    friend class RTSComp;
};

typedef ref_ptr<RTSystemItemEx> RTSystemItemExPtr;
}

#endif
