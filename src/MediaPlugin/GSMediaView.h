/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_MEDIAPLUGIN_GS_MEDIA_VIEW_H_INCLUDED
#define CNOID_MEDIAPLUGIN_GS_MEDIA_VIEW_H_INCLUDED

#include <cnoid/View>

namespace cnoid {

class GSMediaViewImpl;

class GSMediaView : public View
{
public:
    static bool initializeClass(ExtensionManager* ext);

    GSMediaView();
    ~GSMediaView();

    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);

protected:
    virtual bool event(QEvent* event);
    virtual void resizeEvent(QResizeEvent* event);
    virtual void paintEvent(QPaintEvent* event);
    virtual QPaintEngine* paintEngine () const;
    virtual void onActivated();
    virtual void onDeactivated();

private:
    GSMediaViewImpl* impl;
};
}

#endif
