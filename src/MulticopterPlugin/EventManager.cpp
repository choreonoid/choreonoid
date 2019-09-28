/**
   @author Japan Atomic Energy Agency
*/

#include "MulticopterPluginHeader.h"

using namespace std;
using namespace cnoid;
using namespace Multicopter;


EventManager* EventManager::_inst = nullptr;

EventManager*
EventManager::instance()
{
    if( _inst == nullptr ){
        _inst = new EventManager();
    }
    return _inst;
}

bool
EventManager::initialize()
{
    if( _curBodyChangedCon.connected() == false ){
        _curBodyChangedCon = BodyBar::instance()->sigCurrentBodyItemChanged().connect(
            std::bind(&EventManager::onCurrentBodyItemChanged, this, std::placeholders::_1));
    }

    if( _lnkSelChangedCon.connected() == false ){
        LinkSelectionView* lnkSelView = LinkSelectionView::mainInstance();
        if( lnkSelView ){            
            _lnkSelChangedCon =
                lnkSelView->sigSelectionChanged(_curBodyItem).connect(
                    std::bind(&EventManager::onLinkItemSelected, this));
        }
    }
    
    RootItem* rootItem = RootItem::instance();
    _itemAddCon.disconnect();
    _itemAddCon = rootItem->sigItemAdded().connect(
        std::bind(&EventManager::onItemAdd, this, std::placeholders::_1));

    _itemRemoveCon.disconnect();
    _itemRemoveCon = rootItem->sigSubTreeRemoved().connect(
        std::bind(&EventManager::onSubTreeRemoved, this, std::placeholders::_1, std::placeholders::_2));
           
    _isInitialized = true;

    return true;
}

void
EventManager::finalize()
{
    if( _isFinalized == true ){
        return;
    }

    if( _curBodyChangedCon.connected() == true ){
        _curBodyChangedCon.disconnect();
    }

    if( _lnkSelChangedCon.connected() == true ){
        _lnkSelChangedCon.disconnect();
    }
    _curLink = nullptr;
    _isFinalized = true;
}

EventManager::EventManager()
{
}

EventManager::~EventManager()
{

}

QUuid
EventManager::addCurrentLinkChangedEvent(std::function<void(const cnoid::Body*, const cnoid::Link*)>& ev)
{
    return addEvent(ev, _curLinkChangedEvMap);    
}


bool
EventManager::removeCurrentLinkChangedEvent(const QUuid& id)
{
    return removeEvent(id, _curLinkChangedEvMap);
}

QUuid
EventManager::addBodyItemCreateEvent(std::function<void(const cnoid::BodyItem*)>& ev)
{
    return addEvent(ev, _bodyItemCreateEvMap);
}

bool
EventManager::removeBodyItemCreateEvent(const QUuid& id)
{
    return removeEvent(id, _bodyItemCreateEvMap);
}

QUuid
EventManager::addBodyItemDeleteEvent(std::function<void(const cnoid::BodyItem*)>& ev)
{
    return addEvent(ev, _bodyItemDeleteEvMap);
}

bool
EventManager::removeBodyItemDeleteEvent(const QUuid& id)
{
    return removeEvent(id, _bodyItemDeleteEvMap);
}

QUuid
EventManager::addSimulationStartedEvent(std::function<void(cnoid::SimulatorItem*)>& ev)
{
    return addEvent(ev, _simStartedEvMap);
}

bool
EventManager::removeSimulationStartedEvent(const QUuid& id)
{
    return removeEvent(id, _simStartedEvMap);
}

QUuid
EventManager::addSimulationStartEndEvent(function<void(bool,double,SimulatorItem*,SubSimulatorItem*)>& ev)
{
    return addEvent(ev, _simStartEndEvMap);
}

bool
EventManager::removeSimulationStartEndEvent(const QUuid& id)
{
    return removeEvent(id, _simStartEndEvMap);
}

QUuid
EventManager::addSimulationStepEvent(function<void(double, SimulatorItem*, SubSimulatorItem*)>& ev)
{
    return addEvent(ev, _simStepEvMap);
}

bool
EventManager::removeSimulationStepEvent(const QUuid& id)
{
    return removeEvent(id, _simStepEvMap);
}

void
EventManager::onCurrentBodyItemChanged(BodyItem* bodyItem)
{
    if( bodyItem == nullptr ){
        _curBodyItem = nullptr;
        _curLink = nullptr;
        _lnkSelChangedCon.disconnect();
        LinkManager::instance()->setCurrentBodyLink(nullptr, nullptr);
        for(auto it = begin(_curLinkChangedEvMap) ; it != end(_curLinkChangedEvMap) ; ++it){
            it->second(nullptr, nullptr);
        }
        return;
    }

    if( bodyItem != _curBodyItem ){
        _curBodyItem = bodyItem;
        _curLink = _curBodyItem->body()->rootLink();
    }

    if( _lnkSelChangedCon.connected() == true ){
      _lnkSelChangedCon.disconnect();
    }
    if( _lnkSelChangedCon.connected() == false ){
        LinkSelectionView* lnkSelView = LinkSelectionView::mainInstance();
        if( lnkSelView ){
            _lnkSelChangedCon = lnkSelView->sigSelectionChanged(_curBodyItem).connect(std::bind(&EventManager::onLinkItemSelected, this));
        }
    }
    LinkManager::instance()->setCurrentBodyLink(_curBodyItem->body(), _curLink);

    for(auto it = begin(_curLinkChangedEvMap) ; it != end(_curLinkChangedEvMap) ; ++it){
        it->second(_curBodyItem->body(), _curLink);
    }
}

void
EventManager::onLinkItemSelected()
{
    const vector<int>& selectedLinkIndices = LinkSelectionView::mainInstance()->selectedLinkIndices(_curBodyItem);
    if(selectedLinkIndices.empty()){
        _curLink = _curBodyItem->body()->rootLink();
    } else {
	  _curLink = _curBodyItem->body()->link(selectedLinkIndices.front());
    }

    LinkManager::instance()->setCurrentBodyLink(_curBodyItem->body(), _curLink);

    for(auto it = begin(_curLinkChangedEvMap) ; it != end(_curLinkChangedEvMap) ; ++it){
        it->second(_curBodyItem->body(), _curLink);
    }
}

void
EventManager::onItemAdd(Item* item)
{
    BodyItem * bodyItem = dynamic_cast<BodyItem*>(item);
    if( bodyItem != nullptr ){
        for(auto it = begin(_bodyItemCreateEvMap) ; it != end(_bodyItemCreateEvMap) ; ++it){
            it->second(bodyItem);
        }
    }
    SimulatorItem* simItem = dynamic_cast<SimulatorItem*>(item);
    if( simItem != nullptr ){
        auto con = simItem->sigSimulationStarted().connect(std::bind(&EventManager::onSimulationStarted, this, simItem));
        _simItemConMap[simItem] = con;
    }
}

void
EventManager::onSubTreeRemoved(Item* item, bool isMoving)
{
    if( isMoving == false ){
        list<BodyItem*> bodyItemAry;
        LinkManager::instance()->childItemArray(item, bodyItemAry, true);
        for(auto it = begin(_bodyItemDeleteEvMap) ; it != end(_bodyItemDeleteEvMap) ; ++it){
            for(auto ite = begin(bodyItemAry) ; ite != end(bodyItemAry) ; ++ite){
                it->second(*ite);
            }
        }
        SimulatorItem* simItem = dynamic_cast<SimulatorItem*>(item);
        if( simItem != nullptr ){
            auto simCon = _simItemConMap.find(simItem);
            if( simCon != end(_simItemConMap) ){
                simCon->second.disconnect();
                _simItemConMap.erase(simItem);
            }
        }
    }
}

void
EventManager::onSimulationStarted(cnoid::SimulatorItem* simItem)
{
    for(auto& ev : _simStartedEvMap){
        ev.second(simItem);
    }
}

void
EventManager::onSimulationStartEnd(bool flg, double time, cnoid::SimulatorItem* simItem, cnoid::SubSimulatorItem* subSimItem)
{
    for(auto it = begin(_simStartEndEvMap) ; it != end(_simStartEndEvMap) ; ++it){
        it->second(flg, time, simItem, subSimItem);
    }
}

void
EventManager::onSimulationStep(double time, cnoid::SimulatorItem* simItem, cnoid::SubSimulatorItem* subSimItem)
{
    for(auto it = begin(_simStepEvMap) ; it != end(_simStepEvMap) ; ++it){
        it->second(time, simItem, subSimItem);
    }    
}
