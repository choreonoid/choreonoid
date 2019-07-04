/**
   @author Japan Atomic Energy Agency
*/

#include "MulticopterPluginHeader.h"
#include <fmt/format.h>

using namespace std;
using namespace cnoid;
using namespace Multicopter;
using fmt::format;

LinkManager* LinkManager::_inst = 0;

LinkManager*
LinkManager::instance()
{
    if( _inst == 0 ){
        _inst = new LinkManager();
    }
    return _inst;
}

bool
LinkManager::initialize()
{
    if( _isInitialized == true ){
        return true;
    }

    std::function<void(const cnoid::BodyItem*)> createEv = std::bind(&LinkManager::onBodyItemCreate, this, std::placeholders::_1);
    _bodyItemCreateEvId = EventManager::instance()->addBodyItemCreateEvent(createEv);

    std::function<void(const cnoid::BodyItem*)> deleteEv = std::bind(&LinkManager::onBodyItemDelete, this, std::placeholders::_1);
    _bodyItemDeleteEvId = EventManager::instance()->addBodyItemDeleteEvent(deleteEv);

    std::function<void(cnoid::SimulatorItem*)> simStartedEv = std::bind(&LinkManager::onSimulationStarted, this, std::placeholders::_1);
    _bodyItemDeleteEvId = EventManager::instance()->addSimulationStartedEvent(simStartedEv);


    for(int i=0 ; i<VISIBLE_ROOT_NODE_SIZE ; ++i){
        if( !_visRootNodeAry[i] ){
            _visRootNodeAry[i] = new SgSwitch();
            _visRootNodeAry[i]->setTurnedOn(true);
            SceneView::instance()->scene()->addChild(_visRootNodeAry[i]);
        }
    }


    if( !_nonVisRootNode ){
        _nonVisRootNode = new SgSwitch();
        _nonVisRootNode->setTurnedOn(true);
        SceneView::instance()->scene()->addChild(_nonVisRootNode);
    }

    _isInitialized = true;
    return true;
}

void
LinkManager::finalize()
{
    EventManager::instance()->removeBodyItemCreateEvent(_bodyItemCreateEvId);
    EventManager::instance()->removeBodyItemDeleteEvent(_bodyItemDeleteEvId);
    _isInitialized = false;
}

bool
LinkManager::initializeSimulation()
{
    if( _isSimInit == true ){
        return true;
    }
    bool ret;


    _isSimInit = true;
    return ret;
}

void
LinkManager::finalizeSimulation()
{
    _isSimInit = false;
}

Body*
LinkManager::currentBody() const
{
    return _curBody;
}

Link*
LinkManager::currentLink() const
{
    return _curLink;
}

cnoid::Body*
LinkManager::body(const cnoid::Link* link) const
{
    auto ret = _linkBodyMap.find(const_cast<Link*>(link));
    if( ret == _linkBodyMap.end() ){
        return nullptr;
    }

    return ret->second;
}

void
LinkManager::linkArray(const cnoid::Body* body, std::vector<cnoid::Link*>& linkAry)
{
    linkAry.clear();
    int linkNum = body->numLinks();
    linkAry.reserve(linkNum);
    for(int i=0 ; i<linkNum ; ++i){
        linkAry.push_back(body->link(i));
    }
}

bool
LinkManager::findBodyLink(const string& bodyName, const string& linkName, tuple<Body*, Link*>& ret)
{

    for(auto& bodyLink : _bodyLinkMap){
        auto& body = bodyLink.first;
        auto& linkAry = bodyLink.second;
        for(auto& link : linkAry){
            if( bodyName == body->name() && linkName == link->name() ){
                ret = make_tuple(body,link);
                return true;
            }
        }
    }
    ret = make_tuple(nullptr,nullptr);
    return false;
}

void
LinkManager::mesh(const Link* link, vector<Triangle3d>& triAry)
{    
    SgNode* shape = link->collisionShape();

    if( shape == nullptr )
        return;

    MeshExtractor* meshExtractor = new MeshExtractor();

    const int VERTEX_NUMBER_RESERVE_SIZE = 10000;
    const int TRIANGLE_NUMBER_RESERVE_SIZE = 3000;

    vector<Eigen::Vector3d> vtxAry;
    vector<Eigen::Vector3i> triIdxAry;

    vtxAry.reserve(VERTEX_NUMBER_RESERVE_SIZE);
    triIdxAry.reserve(TRIANGLE_NUMBER_RESERVE_SIZE);

    bool ret = meshExtractor->extract(shape, [&](){ onMeshFound(meshExtractor, link, &vtxAry, &triIdxAry); });

    delete meshExtractor;

    if( ret == false ){
        return;
    }

    triAry.clear();
    triAry.reserve(triIdxAry.size()*3);

    for(auto it = begin(triIdxAry) ; it != end(triIdxAry) ; ++it){
        Eigen::Vector3d p0 = vtxAry[(*it)[0]];
        Eigen::Vector3d p1 = vtxAry[(*it)[1]];
        Eigen::Vector3d p2 = vtxAry[(*it)[2]];
        triAry.push_back(Triangle3d(p0,p1,p2));
    }
}

SgSwitchPtr
LinkManager::visibleRootNode(int idx)
{
    return _visRootNodeAry[idx];
}

SgSwitchPtr
LinkManager::nonVisibleRootNode()
{
    return _nonVisRootNode;
}

void
LinkManager::showVisibleNode(bool flg, int idx)
{
    if( _visRootNodeAry[idx] ){
        _visRootNodeAry[idx]->setTurnedOn(flg);
        _visRootNodeAry[idx]->notifyUpdate();
    }
}

void
LinkManager::showNonVisibleNode(bool flg)
{
    if( _nonVisRootNode ){
        _nonVisRootNode->setTurnedOn(flg);
        _nonVisRootNode->notifyUpdate();
    }
}

QUuid
LinkManager::addBodyCreateEvent(std::function<void(const QUuid&, const cnoid::Body*, const std::vector<cnoid::Link*>&, const std::vector<cnoid::Device*>&)>& ev)
{
    return EventManager::addEvent(ev, _bodyCreateEvMap);
}

bool
LinkManager::removeBodyCreateEvent(const QUuid& id)
{
    return EventManager::removeEvent(id, _bodyCreateEvMap);
}

QUuid
LinkManager::addBodyDeleteEvent(std::function<void(const QUuid&, const cnoid::Body*, const std::vector<cnoid::Link*>&, const std::vector<cnoid::Device*>&)>& ev)
{
    return EventManager::addEvent(ev, _bodyDeleteEvMap);
}

bool
LinkManager::removeBodyDeleteEvent(const QUuid& id)
{
    return EventManager::removeEvent(id, _bodyDeleteEvMap);
}

QUuid
LinkManager::addDeviceStateChagedEvent(std::function<void(const QUuid&, const cnoid::Body*, const cnoid::Device*)>& ev)
{
    return EventManager::addEvent(ev, _devStateChangedEvMap);
}

bool
LinkManager::removeDeviceStateChagedEvent(const QUuid& id)
{
    return EventManager::removeEvent(id, _devStateChangedEvMap);
}


QUuid
LinkManager::addBodyKinemaStateChangedEvent(std::function<void(const QUuid&, const cnoid::Body*)>& ev)
{
    return EventManager::addEvent(ev, _bodyKinemaStateChangedEvMap);
}


bool
LinkManager::removeBodyKinemaStateChangedEvent(const QUuid& id)
{
    return EventManager::removeEvent(id, _bodyKinemaStateChangedEvMap);
}

void
LinkManager::onMeshFound(MeshExtractor* extractor, const Link* link, vector<Eigen::Vector3d>* vtxAry, vector<Eigen::Vector3i>* triIdxAry)
{
    SgMesh* mesh = extractor->currentMesh();

    const int curVtxSize = static_cast<int>(vtxAry->size());

    const Eigen::Affine3d& T = extractor->currentTransform();

    const SgVertexArray& orgVtxAry = *(mesh->vertices());

    for(auto it = orgVtxAry.begin() ; it != orgVtxAry.end() ; ++it){
        const Eigen::Vector3d& v = T * (*it).cast<double>();
        vtxAry->push_back(v);
    }

    const int triSize = mesh->numTriangles();
    for(int i=0 ; i<triSize ; ++i){
        SgMesh::TriangleRef src = mesh->triangle(i);
        Eigen::Vector3i triIdx;
        for(int j=0 ; j<3 ; ++j){
            triIdx[j] = curVtxSize + src[j];
        }
        triIdxAry->push_back(triIdx);
    }
}

void
LinkManager::onSimulationStarted(cnoid::SimulatorItem* simItem)
{
}

LinkManager::LinkManager()
{
    _curBody = nullptr;
    _curLink = nullptr;
}

LinkManager::~LinkManager()
{

}

void
LinkManager::setCurrentBodyLink(cnoid::Body* body, cnoid::Link* link)
{
    _curBody = body;
    _curLink = link;
}

void
LinkManager::saveBody(const cnoid::Body* body, const string& fileName)
{
    ofstream out;
    out.open(fileName.data(), ios::out);

    if( !out ){
        return;
    }

    map<Link*, vector<Triangle3d>> lnkTriMap;
    int linkSize = body->numLinks();
    for(int i=0 ; i<linkSize ; ++i){
        Link* lnk = body->link(i);        
        Eigen::Affine3d mtx = lnk->T();

        vector<Triangle3d> triAry;        
        mesh(lnk, triAry);               
        
        for(auto it = begin(triAry) ; it != end(triAry) ; ++it){
            (*it).multiMatrix(mtx);
        }
                
        lnkTriMap[lnk] = triAry;
    }

    int vtxIdx = 0;
    for(auto it = begin(lnkTriMap) ; it != end(lnkTriMap) ; ++it){
        if( it->second.empty() == true ){
            continue;
        }
        out << string("g ");
        out << it->first->name() << endl;
        const vector<Triangle3d>& triAry = it->second;
        int triArySize = triAry.size();
        for(int i=0 ; i<triArySize ; ++i){
            Triangle3d tri = triAry[i];
            for(int j=0 ; j<3 ; ++j){
                out << format("v {0:f} {1:f} {2:f}", tri[j].x(), tri[j].y(), tri[j].z()) << endl;
            }
        }

        for(int i=0 ; i<triArySize ; ++i){
            out << format("f {0:d} {1:d} {2:d}", (vtxIdx+1), (vtxIdx+2), (vtxIdx+3)) << endl;
            vtxIdx +=3;
        }
    }
}


void
LinkManager::onBodyItemCreate(const cnoid::BodyItem* bodyItem)
{
    Body* body = bodyItem->body();    
    vector<Link*> linkAry;
    linkArray(body, linkAry);
    _bodyLinkMap[const_cast<Body*>(body)] = linkAry;

    for(auto it = begin(linkAry) ; it != end(linkAry) ; ++it){
        _linkBodyMap[*it] = body;
    }
    DeviceList<Device> devAry = body->devices();
    vector<Device*> devVecAry;
    devVecAry.reserve(devAry.size());
    for(auto& dev : devAry){
        devVecAry.emplace_back(dev);
    }
    _bodyDevMap[body] = devVecAry;


    cnoid::Connection con;
    con = const_cast<BodyItem*>(bodyItem)->sigKinematicStateChanged().connect(std::bind(&LinkManager::onBodyKinemaStateChanged, this, body));
    _bodyKinemaStateChangedConMap[body] = con;

    for(auto& ev : _bodyCreateEvMap){
        ev.second(ev.first, body, linkAry, devVecAry);
    }
}

void
LinkManager::onBodyItemDelete(const BodyItem* bodyItem)
{
    Body* body = bodyItem->body();
    vector<Link*> linkAry;
    linkArray(body, linkAry);

    for(auto& ev : _bodyDeleteEvMap){
        ev.second(ev.first, body, linkAry, _bodyDevMap[body]);
    }
    
    for(auto it = begin(linkAry) ; it != end(linkAry) ; ++it){
        _linkBodyMap.erase(*it);
    }

    _bodyLinkMap.erase(const_cast<Body*>(body));
    
    auto kinCon = _bodyKinemaStateChangedConMap.find(const_cast<Body*>(body));
    if( kinCon != end(_bodyKinemaStateChangedConMap) ){
        kinCon->second.disconnect();
        _bodyKinemaStateChangedConMap.erase(kinCon);
    }
    
    auto bodyDevIt = _bodyDevMap.find(body);
    if( bodyDevIt != end(_bodyDevMap) ){
        vector<Device*>& devAry = (*bodyDevIt).second;
        for(auto& devStat : devAry){
            auto ret = _devStateChangedConMap.find(devStat);
            if( ret != end(_devStateChangedConMap) ){
                ret->second.disconnect();
                _devStateChangedConMap.erase(ret);
            }
        }
        _bodyDevMap.erase(const_cast<Body*>(body));
    }
}

void
LinkManager::onBodyKinemaStateChanged(cnoid::Body* body)
{
    for(auto& ev : _bodyKinemaStateChangedEvMap){
        ev.second(ev.first, body);
    }
}

void
LinkManager::onBodyDeviceStateChanged(cnoid::Body* body, cnoid::Device* dev)
{
    for(auto& ev : _devStateChangedEvMap){
        ev.second(ev.first, body, dev);
    }
}

