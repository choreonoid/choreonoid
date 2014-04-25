
#ifndef CNOID_MEDIAPLUGIN_DS_MEDIA_VIEW_H_INCLUDED
#define CNOID_MEDIAPLUGIN_DS_MEDIA_VIEW_H_INCLUDED

#include <cnoid/View>
#include <QEvent>

namespace cnoid {

class DSMediaViewImpl;

class DSMediaView : public View
{
public:
    static bool initialize(ExtensionManager* ext);
    static void finalize();

    DSMediaView();
    ~DSMediaView();
        
    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);

protected:
    //virtual bool event(QEvent* event);
    virtual void resizeEvent(QResizeEvent* event);
    virtual void paintEvent(QPaintEvent* event);
    virtual QPaintEngine* paintEngine () const;
    virtual void onActivated();
    virtual void onDeactivated();

private:
    DSMediaViewImpl* impl;
};
}

#endif
