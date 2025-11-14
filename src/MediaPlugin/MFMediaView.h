#ifndef CNOID_MEDIA_PLUGIN_MF_MEDIA_VIEW_H
#define CNOID_MEDIA_PLUGIN_MF_MEDIA_VIEW_H

#include <cnoid/View>

namespace cnoid {

class MFMediaViewImpl;

class MFMediaView : public View
{
public:
    static bool initialize(ExtensionManager* ext);
    static void finalize();

    MFMediaView();
    virtual ~MFMediaView();

    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

protected:
    virtual bool event(QEvent* event) override;
    virtual void resizeEvent(QResizeEvent* event) override;
    virtual void paintEvent(QPaintEvent* event) override;
    virtual QPaintEngine* paintEngine() const override;
    virtual void onActivated() override;
    virtual void onDeactivated() override;

private:
    MFMediaViewImpl* impl;

    friend class MFMediaViewImpl;
};

}

#endif
