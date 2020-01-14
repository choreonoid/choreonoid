/**
   @author Japan Atomic Energy Agency
*/

#pragma once

namespace Multicopter {
class EventManager;

#define VISIBLE_ROOT_NODE_SIZE (2)

class LinkManager
{
friend class EventManager;
public:

    bool initialize();

    void finalize();

    static LinkManager* instance();

    cnoid::Body* currentBody() const;

    cnoid::Link* currentLink() const;

    cnoid::Body* body(const cnoid::Link* link) const;

    void linkArray(const cnoid::Body* body, std::vector<cnoid::Link*>& linkAry);

    void mesh(const cnoid::Link* link, std::vector<Triangle3d>& meshAry);

    bool findBodyLink(const std::string& bodyName, const std::string& linkName, std::tuple<cnoid::Body*, cnoid::Link*>& ret);

    void saveBody(const cnoid::Body* body, const std::string& fileName);

    template<typename T> void childItemArray(cnoid::Item* item, std::list<T*>& itemAry, bool containSelf = false){
        if( item == nullptr ){
            return;
        }
        if( containSelf == true ){
            T* self = dynamic_cast<T*>(item);
            if( self != nullptr ){
                itemAry.push_back(self);
            }
        }
        containSelf = false;
        for(auto child = item->childItem() ; child ; child = child->nextItem()){
            T* childItem = dynamic_cast<T*>(child);
            if( childItem ){
                itemAry.push_back(childItem);
            }
            childItemArray(child, itemAry, containSelf);
        }
    }

    cnoid::SgSwitchableGroupPtr visibleRootNode(int idx = 0);

    cnoid::SgSwitchableGroupPtr nonVisibleRootNode();

    void showVisibleNode(bool flg, int idx = 0);

    void showNonVisibleNode(bool flg);

    QUuid addBodyCreateEvent(std::function<void(const QUuid&, const cnoid::Body*, const std::vector<cnoid::Link*>&, const std::vector<cnoid::Device*>&)>& ev);

    bool removeBodyCreateEvent(const QUuid& id);

    QUuid addBodyDeleteEvent(std::function<void(const QUuid&, const cnoid::Body*, const std::vector<cnoid::Link*>&, const std::vector<cnoid::Device*>&)>& ev);

    bool removeBodyDeleteEvent(const QUuid& id);

    QUuid addBodyKinemaStateChangedEvent(std::function<void(const QUuid&, const cnoid::Body*)>& ev);

    bool removeBodyKinemaStateChangedEvent(const QUuid& id);

    QUuid addDeviceStateChagedEvent(std::function<void(const QUuid&, const cnoid::Body*, const cnoid::Device*)>& ev);

    bool removeDeviceStateChagedEvent(const QUuid& id);

    void setLinkBodyMap(std::map<cnoid::Link*, cnoid::Body* > map){
        _linkBodyMap=map;
    }

    std::map<cnoid::Link*, cnoid::Body* > getLinkBodyMap(){
        return _linkBodyMap;
    }

    bool initializeSimulation();

    void finalizeSimulation();

private:        

    void onMeshFound(cnoid::MeshExtractor* meshExtractor, const cnoid::Link* link, std::vector<Eigen::Vector3d>* vtxAry, std::vector<Eigen::Vector3i>* triIdxAry);

    void setCurrentBodyLink(cnoid::Body* body, cnoid::Link* link);

    LinkManager();

    ~LinkManager();
private:
    void onBodyItemCreate(const cnoid::BodyItem* bodyItem);
    void onBodyItemDelete(const cnoid::BodyItem* bodyItem);
    void onBodyKinemaStateChanged(cnoid::Body* body);
    void onBodyDeviceStateChanged(cnoid::Body* body, cnoid::Device* dev);
    void onSimulationStarted(cnoid::SimulatorItem* simItem);

    static LinkManager* _inst;

    QUuid _bodyItemCreateEvId;
    QUuid _bodyItemDeleteEvId;
    QUuid _simStartedEvId;

    std::map<QUuid, std::function<void(const QUuid&, const cnoid::Body*, const std::vector<cnoid::Link*>&, const std::vector<cnoid::Device*>&)>> _bodyCreateEvMap;
    std::map<QUuid, std::function<void(const QUuid&, const cnoid::Body*, const std::vector<cnoid::Link*>&, const std::vector<cnoid::Device*>&)>> _bodyDeleteEvMap;
    std::map<QUuid, std::function<void(const QUuid&, const cnoid::Body*)>> _bodyKinemaStateChangedEvMap;
    std::map<QUuid, std::function<void(const QUuid&, const cnoid::Body*, const cnoid::Device*)>> _devStateChangedEvMap;

    std::map<cnoid::Body*, std::vector<cnoid::Link*>> _bodyLinkMap;
    std::map<cnoid::Link*, cnoid::Body* > _linkBodyMap;
    std::map<cnoid::Body*,std::vector<cnoid::Device*>> _bodyDevMap;
    std::map<cnoid::Body*, cnoid::Connection> _bodyKinemaStateChangedConMap;
    std::map<cnoid::Device*, cnoid::Connection> _devStateChangedConMap;


    cnoid::Body* _curBody;
    cnoid::Link* _curLink;

    cnoid::SgSwitchableGroupPtr _visRootNodeAry[VISIBLE_ROOT_NODE_SIZE];
    cnoid::SgSwitchableGroupPtr _nonVisRootNode;
    
    bool _isInitialized = false;
    bool _isFinalized = false;
    bool _isSimInit = false;
};
}
