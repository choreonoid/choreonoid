/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_SENSOR_VISUALIZER_ITEM_H
#define CNOID_BODY_PLUGIN_SENSOR_VISUALIZER_ITEM_H

#include <cnoid/Item>
#include <cnoid/BodyItem>
#include <cnoid/SceneProvider>
#include <cnoid/PointSetItem>
#include <cnoid/RangeCamera>
#include <cnoid/RangeSensor>
#include "exportdecl.h"

namespace cnoid {

class SensorVisualizerItemImpl;

class CNOID_EXPORT SensorVisualizerItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    SensorVisualizerItem();
    SensorVisualizerItem(const SensorVisualizerItem& org);
    virtual ~SensorVisualizerItem();

protected:
    virtual Item* doDuplicate() const;
    virtual void onPositionChanged();
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);


private:
    SensorVisualizerItemImpl* impl;
};
        
typedef ref_ptr<SensorVisualizerItem> SensorVisualizerItemPtr;


class ForceSensorVisualizerItemImpl;

class CNOID_EXPORT ForceSensorVisualizerItem : public Item, public SceneProvider
{
public :
    static void initializeClass(ExtensionManager* ext);

    ForceSensorVisualizerItem();
    virtual ~ForceSensorVisualizerItem();

    virtual SgNode* getScene();

    void setBodyItem(BodyItem* bodyItem);
    void setVisualRatio(double visualRatio);

protected:
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);

private :
    ForceSensorVisualizerItemImpl* impl;
};

typedef ref_ptr<ForceSensorVisualizerItem> ForceSensorVisualizerItemPtr;


class PointCloudVisualizerItemImpl;

class CNOID_EXPORT PointCloudVisualizerItem : public PointSetItem
{
public :
    static void initializeClass(ExtensionManager* ext);

    PointCloudVisualizerItem();
    virtual ~PointCloudVisualizerItem();

    void setBodyItem(BodyItem* bodyItem, RangeCamera* rangeCamera);

protected:
    //virtual void doPutProperties(PutPropertyFunction& putProperty);
    //virtual bool store(Archive& archive);

private :
    PointCloudVisualizerItemImpl* impl;
};

typedef ref_ptr<PointCloudVisualizerItem> PointCloudVisualizerItemPtr;

class RangeSensorVisualizerItemImpl;

class CNOID_EXPORT RangeSensorVisualizerItem : public PointSetItem
{
public :
    static void initializeClass(ExtensionManager* ext);

    RangeSensorVisualizerItem();
    virtual ~RangeSensorVisualizerItem();

    void setBodyItem(BodyItem* bodyItem, RangeSensor* rangeSensor);

protected:
    //virtual void doPutProperties(PutPropertyFunction& putProperty);
    //virtual bool store(Archive& archive);

private :
    RangeSensorVisualizerItemImpl* impl;
};

typedef ref_ptr<RangeSensorVisualizerItem> RangeSensorVisualizerItemPtr;

}
#endif
