#ifndef CNOID_MOCAP_PLUGIN_MARKER_MOTION_ITEM_H
#define CNOID_MOCAP_PLUGIN_MARKER_MOTION_ITEM_H

#include "MarkerMotion.h"
#include "MocapMapping.h"
#include <cnoid/MultiVector3SeqItem>
#include <cnoid/ConnectionSet>
#include <cnoid/SceneProvider>
#include "exportdecl.h"

namespace cnoid {

class SceneMarkerMotion;
typedef ref_ptr<SceneMarkerMotion> SceneMarkerMotionPtr;

class CNOID_EXPORT MarkerMotionItem : public MultiVector3SeqItem, public SceneProvider
{
public:

    static void initialize(ExtensionManager* ext);

    MarkerMotionItem();

    MarkerMotionPtr motion() { return motion_; }
    MocapMapping* mocapMapping() { return mocapMapping_.get(); }

    int currentFrame() const { return currentFrame_; }

    const Vector3& currentMarkerPosition(int markerIndex) const {
        return currentMarkerPositions[markerIndex];
    }
    Vector3& currentMarkerPosition(int markerIndex) {
        return currentMarkerPositions[markerIndex];
    }

    SignalProxy<void(int markerIndex, const Vector3& newPosition)>
    sigMarkerPositionChangeRequest() {
        return sigMarkerPositionChangeRequest_;
    }

    void requestMarkerPositionChange(int markerIndex, const Vector3& newPosition);

    SignalProxy<void()> sigCurrentMarkerPositionsChanged() {
        return sigCurrentMarkerPositionsChanged_;
    }
        
    void notifyCurrentMarkerPositionsChanged();

    virtual void notifyUpdate() override;

    bool loadStdYamlFormat(const std::string& filename, std::ostream& os = mvout());
    bool saveAsStdYamlFormat(const std::string& filename, std::ostream& os = mvout());
    bool loadVPM(const std::string& filename, std::ostream& os = mvout());
    bool loadTRC(const std::string& filename, std::ostream& os = mvout());

    bool isEdgeRenderingEnabled() const { return isEdgeRenderingEnabled_; }
    const Vector3& color() const { return color_; }
    double edgeWidth() const { return edgeWidth_; }

    virtual SgNode* getScene() override;

protected:

    MarkerMotionItem(const MarkerMotionItem& org);

    virtual void onConnectedToRoot() override;
    virtual void onDisconnectedFromRoot() override;
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;
    virtual void onTreePathChanged() override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    MarkerMotionPtr motion_;
    MocapMappingPtr mocapMapping_;
    SceneMarkerMotionPtr scene_;
    bool isEdgeRenderingEnabled_;
    Vector3 color_;
    double edgeWidth_;
    int currentFrame_;
    std::vector<Vector3> currentMarkerPositions;
    bool needToUpdateCurrentMarkerPositions;
    Signal<void(int markerIndex, const Vector3& newPosition)> sigMarkerPositionChangeRequest_;
    Signal<void()> sigCurrentMarkerPositionsChanged_;
    ScopedConnection timeBarConnection;;
    ConnectionSet sceneConnections;

    virtual ~MarkerMotionItem();
    void initInstance();
    void onItemCheckToggled();
    bool onTimeChanged(double time);
    bool onPositionOffsetChanged(const std::string& value);
    bool onScaleChanged(double scale);

    void onSceneConnection(bool on);
};

typedef ref_ptr<MarkerMotionItem> MarkerMotionItemPtr;

}

#endif
