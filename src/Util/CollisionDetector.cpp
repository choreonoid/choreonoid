/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "CollisionDetector.h"
#include <map>

using namespace std;
using namespace cnoid;

namespace {
struct FactoryInfo
{
    string name;
    boost::function<CollisionDetectorPtr()> factory;
    FactoryInfo(const string& name, const boost::function<CollisionDetectorPtr()>& factory)
        : name(name), factory(factory) { }
};
    
vector<FactoryInfo> factories;

typedef map<string, int> FactoryMap;
FactoryMap factoryMap;


/**
   The collision detector which does nothing.
   This detector is registered as the first (zero indexed) detector.
*/
class NullCollisionDetector : public CollisionDetector
{
    int numGeometries_;
public:
    NullCollisionDetector() { numGeometries_ = 0; }
    virtual const char* name() const { return "NullCollisionDetector"; }
    virtual CollisionDetectorPtr clone() const { return std::make_shared<NullCollisionDetector>(); }
    virtual bool enableGeometryCache(bool on) { return true; }
    virtual void clearGeometryCache(SgNodePtr geometry) { }
    virtual void clearAllGeometryCaches() { }
    virtual void clearGeometries() { numGeometries_ = 0; }
    virtual int numGeometries() const { return numGeometries_; }
    virtual int addGeometry(SgNodePtr geometry) {
        const int id = numGeometries_++;
        return id;
    }
    virtual void setGeometryStatic(int geometryId, bool isStatic = true) { }
    virtual void setNonInterfarenceGeometyrPair(int geometryId1, int geometryId2) { }
    virtual bool makeReady() { return true; }
    virtual void updatePosition(int geometryId, const Position& position) { }
    virtual void detectCollisions(boost::function<void(const CollisionPair&)> callback) { }
};

CollisionDetectorPtr factory()
{
    return std::make_shared<NullCollisionDetector>();
}

struct FactoryRegistration
{
    FactoryRegistration(){
        CollisionDetector::registerFactory("NullCollisionDetector", factory);
    }
} factoryRegistration;
}


bool CollisionDetector::registerFactory(const std::string& name, boost::function<CollisionDetectorPtr()> factory)
{
    if(!name.empty()){
        int index = factories.size();
        pair<FactoryMap::iterator, bool> result = factoryMap.insert(make_pair(name, index));
        if(result.second){
            factories.push_back(FactoryInfo(name, factory));
            return true;
        }
    }
    return false;
}


int CollisionDetector::numFactories()
{
    return factories.size();
}


std::string CollisionDetector::factoryName(int factoryIndex)
{
    if(factoryIndex >= 0 && factoryIndex < factories.size()){
        return factories[factoryIndex].name;
    }
    return string();
}


int CollisionDetector::factoryIndex(const std::string& name)
{
    FactoryMap::iterator p = factoryMap.find(name);
    if(p != factoryMap.end()){
        return p->second;
    }
    return -1;
}


CollisionDetectorPtr CollisionDetector::create(int factoryIndex)
{
    if(factoryIndex >= 0 && factoryIndex < factories.size()){
        return factories[factoryIndex].factory();
    }
    return CollisionDetectorPtr();
}


CollisionDetector::~CollisionDetector()
{

}
