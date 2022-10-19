#ifndef CNOID_BODY_PLUGIN_OPERABLE_SCENE_BODY_H
#define CNOID_BODY_PLUGIN_OPERABLE_SCENE_BODY_H

#include <cnoid/SceneWidgetEventHandler>
#include <cnoid/SceneBody>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class BodyItem;
class OperableSceneBody;

class CNOID_EXPORT OperableSceneLink : public SceneLink
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    OperableSceneLink(OperableSceneBody* sceneBody, Link* link);
    ~OperableSceneLink();

    OperableSceneBody* operableSceneBody();
    const OperableSceneBody* operableSceneBody() const;

    virtual void setVisible(bool on) override;

    void showOrigin(bool on);
    bool isOriginShown() const;
    void showCenterOfMass(bool on);
    bool isCenterOfMassShown() const;
    void enableHighlight(bool on);
    void showMarker(const Vector3f& color, float transparency);
    void hideMarker();
    void setColliding(bool on);

private:
    class Impl;
    Impl* impl;

    friend class OperableSceneBody;
};
typedef ref_ptr<OperableSceneLink> OperableSceneLinkPtr;

    
class CNOID_EXPORT OperableSceneBody : public SceneBody, public SceneWidgetEventHandler
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    static void initializeClass(ExtensionManager* ext);

    OperableSceneBody(BodyItem* bodyItem);

    BodyItem* bodyItem();

    OperableSceneLink* operableSceneLink(int index);
    void setLinkVisibilities(const std::vector<bool>& visibilities);

    virtual void updateSceneModel() override;

    virtual void onSceneModeChanged(SceneWidgetEvent* event) override;
    virtual bool onButtonPressEvent(SceneWidgetEvent* event) override;
    virtual bool onDoubleClickEvent(SceneWidgetEvent* event) override;
    virtual bool onButtonReleaseEvent(SceneWidgetEvent* event) override;
    virtual bool onPointerMoveEvent(SceneWidgetEvent* event) override;
    virtual void onPointerLeaveEvent(SceneWidgetEvent* event) override;
    virtual bool onKeyPressEvent(SceneWidgetEvent* event) override;
    virtual bool onKeyReleaseEvent(SceneWidgetEvent* event) override;
    virtual bool onScrollEvent(SceneWidgetEvent* event) override;
    virtual void onFocusChanged(SceneWidgetEvent* event, bool on) override;
    virtual bool onContextMenuRequest(SceneWidgetEvent* event) override;

    [[deprecated("Use operableSceneLink")]]
    OperableSceneLink* editableSceneLink(int index) { return operableSceneLink(index); };

protected:
    virtual ~OperableSceneBody();

private:
    OperableSceneBody(const OperableSceneBody& org);

    class Impl;
    Impl* impl;
    
    friend class OperableSceneLink;
};

typedef ref_ptr<OperableSceneBody> OperableSceneBodyPtr;

[[deprecated("Use OperableSceneBody")]]
typedef OperableSceneBody EditableSceneBody;

}

#endif
