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
    std::function<CollisionDetector*()> factory;
    FactoryInfo(const string& name, const std::function<CollisionDetector*()>& factory)
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
    NullCollisionDetector()
    {
        numGeometries_ = 0;
    }

    virtual const char* name() const override
    {
        return "NullCollisionDetector";
    }

    virtual CollisionDetector* clone() const override
    {
        return new NullCollisionDetector;
    }

    virtual void clearGeometries() override
    {
        numGeometries_ = 0;
    }

    virtual int numGeometries() const override
    {
        return numGeometries_;
    }

    virtual stdx::optional<GeometryHandle> addGeometry(SgNode*) override
    {
        GeometryHandle handle;
        handle = numGeometries_++;
        return handle;
    }

    virtual void setCustomObject(GeometryHandle geometry, Referenced* object) { }

    virtual void setGeometryStatic(GeometryHandle, bool isStatic = true) override { }

    virtual void setNonInterfarenceGeometyrPair(GeometryHandle, GeometryHandle) override { }

    virtual bool makeReady() override
    {
        return true;
    }

    virtual void updatePosition(GeometryHandle, const Position& /* position */) override { }

    virtual void updatePositions(
        std::function<void(Referenced* object, Position*& out_position)> positionQuery) override { }

    virtual void detectCollisions(std::function<void(const CollisionPair& collisionPair)> /* callback */) override { }
};

CollisionDetector* factory()
{
    return new NullCollisionDetector;
}

struct FactoryRegistration
{
    FactoryRegistration(){
        CollisionDetector::registerFactory("NullCollisionDetector", factory);
    }
} factoryRegistration;

}


bool CollisionDetector::registerFactory(const std::string& name, std::function<CollisionDetector*()> factory)
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


CollisionDetector* CollisionDetector::create(int factoryIndex)
{
    if(factoryIndex >= 0 && factoryIndex < static_cast<int>(factories.size())){
        return factories[factoryIndex].factory();
    }
    return nullptr;
}


CollisionDetector::~CollisionDetector()
{

}
