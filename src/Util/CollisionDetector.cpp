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
    std::function<CollisionDetectorPtr()> factory;
    FactoryInfo(const string& name, const std::function<CollisionDetectorPtr()>& factory)
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
    virtual const char* name() const override { return "NullCollisionDetector"; }
    virtual CollisionDetectorPtr clone() const override { return std::make_shared<NullCollisionDetector>(); }
    virtual bool enableGeometryCache(bool) override { return true; }
    virtual void clearGeometryCache(SgNodePtr) override { }
    virtual void clearAllGeometryCaches() override { }
    virtual void clearGeometries() override { numGeometries_ = 0; }
    virtual int numGeometries() const override { return numGeometries_; }
    virtual int addGeometry(SgNodePtr) override {
        const int id = numGeometries_++;
        return id;
    }
    virtual void setGeometryStatic(int /* geometryId */, bool isStatic = true) override { }
    virtual void setNonInterfarenceGeometyrPair(int /* geometryId1 */, int /* geometryId2 */) override { }
    virtual bool makeReady() override { return true; }
    virtual void updatePosition(int /* geometryId */, const Position& /* position */) override { }
    virtual void detectCollisions(std::function<void(const CollisionPair&)> /* callback */) override { }
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


bool CollisionDetector::registerFactory(const std::string& name, std::function<CollisionDetectorPtr()> factory)
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
    if(factoryIndex >= 0 && factoryIndex < static_cast<int>(factories.size())){
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
    if(factoryIndex >= 0 && factoryIndex < static_cast<int>(factories.size())){
        return factories[factoryIndex].factory();
    }
    return CollisionDetectorPtr();
}


CollisionDetector::~CollisionDetector()
{

}
