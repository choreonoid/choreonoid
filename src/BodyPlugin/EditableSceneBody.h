/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_EDITABLE_SCENE_BODY_H
#define CNOID_BODY_PLUGIN_EDITABLE_SCENE_BODY_H

#include <cnoid/SceneWidgetEventHandler>
#include <cnoid/SceneBody>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class BodyItem;
class EditableSceneBody;

class CNOID_EXPORT EditableSceneLink : public SceneLink
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EditableSceneLink(EditableSceneBody* sceneBody, Link* link);
    ~EditableSceneLink();

    void showOrigin(bool on);
    bool isOriginShown() const;
    void enableHighlight(bool on);
    [[deprecated("Use enableHighlight")]]
    void showOutline(bool on);
    void showMarker(const Vector3f& color, float transparency);
    void hideMarker();
    void setColliding(bool on);

private:
    class Impl;
    Impl* impl;

    friend class EditableSceneBody;
};
typedef ref_ptr<EditableSceneLink> EditableSceneLinkPtr;

    
class CNOID_EXPORT EditableSceneBody : public SceneBody, public SceneWidgetEventHandler
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    static void initializeClass(ExtensionManager* ext);

    EditableSceneBody(BodyItem* bodyItem);

    BodyItem* bodyItem();

    EditableSceneLink* editableSceneLink(int index);
    void setLinkVisibilities(const std::vector<bool>& visibilities);

    virtual void updateModel() override;

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

protected:
    virtual ~EditableSceneBody();

private:
    EditableSceneBody(const EditableSceneBody& org);

    class Impl;
    Impl* impl;
    
    friend class EditableSceneLink;
};
            
typedef ref_ptr<EditableSceneBody> EditableSceneBodyPtr;

}
    
#endif
