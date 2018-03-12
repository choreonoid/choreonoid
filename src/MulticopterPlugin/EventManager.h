/**
   @author Japan Atomic Energy Agency
*/

#pragma once

namespace Multicopter {
class LinkManager;

class EventManager
{
friend class LinkManager;
public:

    static EventManager* instance();

    bool initialize();

    void finalize();

    QUuid addCurrentLinkChangedEvent(std::function<void(const cnoid::Body*, const cnoid::Link*)>& ev);

    bool removeCurrentLinkChangedEvent(const QUuid& id);

    QUuid addBodyItemCreateEvent(std::function<void(const cnoid::BodyItem*)>& ev);

    bool removeBodyItemCreateEvent(const QUuid& id);

    QUuid addBodyItemDeleteEvent(std::function<void(const cnoid::BodyItem*)>& ev);

    bool removeBodyItemDeleteEvent(const QUuid& id);

    QUuid addSimulationStartedEvent(std::function<void(cnoid::SimulatorItem*)>& ev);

    bool removeSimulationStartedEvent(const QUuid& id);

    QUuid addSimulationStartEndEvent(std::function<void(bool, double, cnoid::SimulatorItem*, cnoid::SubSimulatorItem*)>& ev);

    bool removeSimulationStartEndEvent(const QUuid& id);

    QUuid addSimulationStepEvent(std::function<void(double, cnoid::SimulatorItem*, cnoid::SubSimulatorItem*)>& ev);

    bool removeSimulationStepEvent(const QUuid& id);
    
    void onSimulationStartEnd(bool flg, double time, cnoid::SimulatorItem* simItem, cnoid::SubSimulatorItem* subSimItem);
    void onSimulationStep(double time, cnoid::SimulatorItem* simItem, cnoid::SubSimulatorItem* subSimItem);


protected:

    EventManager();

    ~EventManager();

    template<typename T> static QUuid addEvent(std::function<T>& ev, std::map<QUuid, std::function<T> >& evMap){
        QUuid id = QUuid::createUuid();
        evMap[id] = ev;
        return id;
    }

    template<typename T> static bool removeEvent(const QUuid& id, std::map<QUuid, T>& evMap){
        auto ret = evMap.find(id);
        if( ret != end(evMap) ){
            evMap.erase(ret);
            return true;
        }
        else{
            return false;
        }
    }

private:

    void onCurrentLinkChanged();
    void onCurrentBodyItemChanged(cnoid::BodyItem* bodyItem);
    void onLinkItemSelected();
    void onAllItemChanged();

    void onItemAdd(cnoid::Item* item);
    void onItemRemove(cnoid::Item* item);
    void onSubTreeRemoved(cnoid::Item* item, bool isMoving);
    
        
    void onSimulationStarted(cnoid::SimulatorItem* simItem);

    std::map<QUuid, std::function<void(const cnoid::Body*, const cnoid::Link*)>> _curLinkChangedEvMap;
    std::map<QUuid, std::function<void(const cnoid::BodyItem*)>> _bodyItemCreateEvMap;
    std::map<QUuid, std::function<void(const cnoid::BodyItem*)>> _bodyItemDeleteEvMap;
    std::map<QUuid, std::function<void(bool,double,cnoid::SimulatorItem*,cnoid::SubSimulatorItem*)>> _simStartEndEvMap;
    std::map<QUuid, std::function<void(double, cnoid::SimulatorItem*,cnoid::SubSimulatorItem*)>> _simStepEvMap;

    std::map<QUuid, std::function<void(cnoid::SimulatorItem*)>> _simStartedEvMap;
    
    std::map<cnoid::SimulatorItem*, cnoid::Connection> _simItemConMap;
    
    cnoid::Connection _lnkSelChangedCon;
    cnoid::Connection _curBodyChangedCon;

    cnoid::Connection _itemAddCon;
    cnoid::Connection _itemRemoveCon;
    
    cnoid::Connection _viewCreatedCon;
    cnoid::Connection _viewDeleteCon;

    cnoid::BodyItemPtr _curBodyItem;
    cnoid::Link* _curLink = nullptr;
    bool _isInitialized = false;
    bool _isFinalized = false;

    static EventManager* _inst;

};
}
