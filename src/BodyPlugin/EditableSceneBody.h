/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_EDITABLE_SCENE_BODY_H
#define CNOID_BODYPLUGIN_EDITABLE_SCENE_BODY_H

#include <cnoid/SceneWidgetEditable>
#include <cnoid/SceneBody>
#include <cnoid/SceneMarker>
#include <boost/dynamic_bitset.hpp>
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

    void showBoundingBox(bool on);
    void showMarker(const Vector3f& color, float transparency);
    void hideMarker();
    void setColliding(bool on);

private:
    SgLineSetPtr bbLineSet;
    SgOutlineGroupPtr outlineGroup;
    BoundingBoxMarkerPtr bbMarker;
    bool isPointed_;
    bool isColliding_;

    void createBoundingBoxLineSet();
};
typedef ref_ptr<EditableSceneLink> EditableSceneLinkPtr;

    
class EditableSceneBodyImpl;
    
class CNOID_EXPORT EditableSceneBody : public SceneBody, public SceneWidgetEditable
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    static void initializeClass(ExtensionManager* ext);

    EditableSceneBody(BodyItemPtr bodyItem);

    EditableSceneLink* editableSceneLink(int index);
    const boost::dynamic_bitset<>& linkVisibilities() const;
    void setLinkVisibilities(const boost::dynamic_bitset<>& visibilities);

    virtual void updateModel();

    virtual bool onKeyPressEvent(const SceneWidgetEvent& event);
    virtual bool onKeyReleaseEvent(const SceneWidgetEvent& event);
    virtual bool onButtonPressEvent(const SceneWidgetEvent& event);
    virtual bool onDoubleClickEvent(const SceneWidgetEvent& event);
    virtual bool onButtonReleaseEvent(const SceneWidgetEvent& event);
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event);
    virtual void onPointerLeaveEvent(const SceneWidgetEvent& event);
    virtual bool onScrollEvent(const SceneWidgetEvent& event);
    virtual void onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menuManager);
    virtual void onSceneModeChanged(const SceneWidgetEvent& event);
    virtual bool onUndoRequest();
    virtual bool onRedoRequest();

protected:
    virtual ~EditableSceneBody();

private:
    friend class EditableSceneBodyImpl;
    EditableSceneBodyImpl* impl;
    EditableSceneBody(const EditableSceneBody& org);
};
            
typedef ref_ptr<EditableSceneBody> EditableSceneBodyPtr;
}
    
#endif
