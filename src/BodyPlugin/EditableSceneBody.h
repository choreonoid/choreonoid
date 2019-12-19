/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_EDITABLE_SCENE_BODY_H
#define CNOID_BODY_PLUGIN_EDITABLE_SCENE_BODY_H

#include <cnoid/SceneWidgetEditable>
#include <cnoid/SceneBody>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;

class BodyItem;
typedef ref_ptr<BodyItem> BodyItemPtr;

class CNOID_EXPORT EditableSceneLink : public SceneLink
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EditableSceneLink(Link* link);
    ~EditableSceneLink();

    void showOutline(bool on);
    void showMarker(const Vector3f& color, float transparency);
    void hideMarker();
    void setColliding(bool on);

private:
    class Impl;
    Impl* impl;
    
};
typedef ref_ptr<EditableSceneLink> EditableSceneLinkPtr;

    
class CNOID_EXPORT EditableSceneBody : public SceneBody, public SceneWidgetEditable
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    static void initializeClass(ExtensionManager* ext);

    EditableSceneBody(BodyItemPtr bodyItem);

    EditableSceneLink* editableSceneLink(int index);
    void setLinkVisibilities(const std::vector<bool>& visibilities);

    bool isDraggable() const;
    void setDraggable(bool on);

    virtual void updateModel() override;

    virtual bool onKeyPressEvent(const SceneWidgetEvent& event) override;
    virtual bool onKeyReleaseEvent(const SceneWidgetEvent& event) override;
    virtual bool onButtonPressEvent(const SceneWidgetEvent& event) override;
    virtual bool onDoubleClickEvent(const SceneWidgetEvent& event) override;
    virtual bool onButtonReleaseEvent(const SceneWidgetEvent& event) override;
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event) override;
    virtual void onPointerLeaveEvent(const SceneWidgetEvent& event) override;
    virtual bool onScrollEvent(const SceneWidgetEvent& event) override;
    virtual void onFocusChanged(const SceneWidgetEvent& event, bool on) override;
    virtual void onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menuManager) override;
    virtual void onSceneModeChanged(const SceneWidgetEvent& event) override;
    virtual bool onUndoRequest() override;
    virtual bool onRedoRequest() override;

protected:
    virtual ~EditableSceneBody();

private:
    EditableSceneBody(const EditableSceneBody& org);

    class Impl;
    Impl* impl;
    
    friend class EditableSceneBodyImpl;
};
            
typedef ref_ptr<EditableSceneBody> EditableSceneBodyPtr;

}
    
#endif
