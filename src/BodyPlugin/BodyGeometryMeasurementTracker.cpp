#include "BodyGeometryMeasurementTracker.h"
#include "BodyItemKinematicsKit.h"
#include "OperableSceneBody.h"
#include <cnoid/CoordinateFrameList>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;


static GeometryMeasurementTracker* createBodyGeometryMeasurementTracker(Item* item)
{
    if(auto bodyItem = dynamic_cast<BodyItem*>(item)){
        return new BodyGeometryMeasurementTracker(bodyItem);
    }
    return nullptr;
}


void BodyGeometryMeasurementTracker::initializeClass()
{
    GeometryMeasurementTracker::registerItemTrackerFactory(
        createBodyGeometryMeasurementTracker);
}


BodyGeometryMeasurementTracker::BodyGeometryMeasurementTracker(BodyItem* bodyItem)
    : bodyItem(bodyItem),
      body(bodyItem->body())
{
    resetMeasurementPoint();

    currentSubEntryIndex = 0;
    needToUpdateSubEntries = true;

    connections.add(
        bodyItem->sigKinematicStateChanged().connect([this](){ sigGeometryChanged_(); }));
    connections.add(
        bodyItem->sigModelUpdated().connect(
            [this](int flags){
                if(flags & (BodyItem::LinkSetUpdate | BodyItem::LinkSpecUpdate | BodyItem::ShapeUpdate)){
                    sigGeometryChanged_();
                }
            }));
}


void BodyGeometryMeasurementTracker::resetMeasurementPoint()
{
    resetMeasurementPoint_(body->rootLink());
}


void BodyGeometryMeasurementTracker::resetMeasurementPoint_(Link* link)
{
    setMeasurementPoint_(link, Vector3::Zero());
}


void BodyGeometryMeasurementTracker::setMeasurementPoint_(Link* link, const Vector3& localPosition)
{
    linkOfPoint = link;
    this->localPosition = localPosition;
}


Vector3 BodyGeometryMeasurementTracker::getMeasurementPoint()
{
    if(linkOfPoint){
        return linkOfPoint->T() * localPosition;
    } else {
        return body->rootLink()->translation();
    }
    return Vector3::Zero();
}


bool BodyGeometryMeasurementTracker::setMeasurementPoint(const SgNodePath& path, const Vector3& point)
{
    bool updated = false;
    auto index = findSubEntryForSceneNode(path);
    if(index >= 0){
        currentSubEntryIndex = index;
        auto link = subEntries[index].link;
        setMeasurementPoint_(link, link->T().inverse() * point);
        updated = true;
    }
    return updated;
}


int BodyGeometryMeasurementTracker::findSubEntryForSceneNode(const SgNodePath& path)
{
    int subEntryIndex = -1;

    Link* foundLink = nullptr;
    for(auto& node : path){
        if(auto sceneLink = dynamic_cast<OperableSceneLink*>(node.get())){
            auto link = sceneLink->link();
            if(link->body() == body){
                foundLink = link;
                break;
            }
        }
    }
    if(foundLink){
        if(needToUpdateSubEntries){
            updateSubEntries();
        }
        for(size_t i=0; i < subEntries.size(); ++i){
            auto& entry = subEntries[i];
            if(!entry.offsetFrame){
                if(entry.link == foundLink){
                    subEntryIndex = i;
                    break;
                }
            }
        }
    }

    return subEntryIndex;
}
    

bool BodyGeometryMeasurementTracker::checkTrackable(const SgNodePath& path)
{
    return findSubEntryForSceneNode(path) >= 0;
}


void BodyGeometryMeasurementTracker::updateSubEntries()
{
    subEntries.clear();
    subEntryConnections.disconnect();
    
    for(auto& link : body->links()){
        subEntries.emplace_back(link->name(), link, nullptr);
        if(link->isRoot() && link->child()){
            continue;
        }
        if(auto kinematicsKit = bodyItem->getCurrentKinematicsKit(link)){
            if(auto frames = kinematicsKit->offsetFrames()){
                int n = frames->numFrames();
                int frameIndex = frames->hasFirstElementAsDefaultFrame() ? 1: 0;
                while(frameIndex < n){
                    auto frame = frames->frameAt(frameIndex);
                    auto& note =frame->note();
                    string name;
                    if(note.empty()){
                        name = format(_("Offset {0}"), frame->id().label());
                    } else {
                        name = format(_("Offset {0} ( {1} )"), frame->id().label(), note);
                    }
                    subEntries.emplace_back(name, link, frame);
                    ++frameIndex;
                }
            }
            subEntryConnections.add(
                kinematicsKit->sigFrameSetChanged().connect(
                    [this](){ needToUpdateSubEntries = true; }));
        }
    }

    needToUpdateSubEntries = false;
}


int BodyGeometryMeasurementTracker::getNumSubEntries()
{
    if(needToUpdateSubEntries){
        updateSubEntries();
    }
    return subEntries.size();
}


std::string BodyGeometryMeasurementTracker::getSubEntryName(int index)
{
    if(needToUpdateSubEntries){
        updateSubEntries();
    }
    if(index < static_cast<int>(subEntries.size())){
        return subEntries[index].name;
    }
    return std::string();
}


int BodyGeometryMeasurementTracker::findSubEntryIndex(const std::string& name)
{
    if(needToUpdateSubEntries){
        updateSubEntries();
    }    
    int index = -1;
    for(size_t i=0; i < subEntries.size(); ++i){
        if(name == subEntries[i].name){
            index = i;
            break;
        }
    }
    return index;
}


int BodyGeometryMeasurementTracker::getCurrentSubEntryIndex()
{
    return currentSubEntryIndex;
}


bool BodyGeometryMeasurementTracker::setCurrentSubEntry(int index)
{
    if(needToUpdateSubEntries){
        updateSubEntries();
    }
    if(index < static_cast<int>(subEntries.size())){
        currentSubEntryIndex = index;
        auto& subEntry = subEntries[index];
        if(subEntry.offsetFrame){
            setMeasurementPoint_(subEntry.link, subEntry.offsetFrame->position().translation());
        } else {
            resetMeasurementPoint_(subEntry.link);
        }
        return true;
    }
    return false;
}


int BodyGeometryMeasurementTracker::getNumShapes()
{
    auto shape = linkOfPoint->shape();
    return shape->empty() ? 0 : 1;
}


SgNode* BodyGeometryMeasurementTracker::getShape(int index)
{
    auto shape = linkOfPoint->shape();
    if(!shape->empty() && index == 0){
        return shape;
    }
    return nullptr;
}


Isometry3 BodyGeometryMeasurementTracker::getShapePosition(int index)
{
    if(index == 0){
        return linkOfPoint->position();
    }
    return Isometry3::Identity();
}


SignalProxy<void()> BodyGeometryMeasurementTracker::sigGeometryChanged()
{
    return sigGeometryChanged_;
}
