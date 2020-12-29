#ifndef CNOID_BASE_IMAGE_VIEW_H
#define CNOID_BASE_IMAGE_VIEW_H

#include "View.h"
#include "ToolBar.h"
#include "exportdecl.h"

class QImage;

namespace cnoid {

class Image;
class ImageableItem;
    
class CNOID_EXPORT ImageView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static ImageView* instance();
        
    ImageView();
    ~ImageView();

    void setPixmap(const QPixmap& pixmap);
    void setImage(const Image& image);
    void setImage(const QImage& image);

    void setScalingEnabled(bool on);
    bool isScalingEnabled() const;

    ImageableItem* getImageableItem();
    void setImageableItem(ImageableItem* imageable);

    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

protected:
    virtual void onDeactivated() override;
    virtual void onFocusChanged(bool on) override;

private:
    class Impl;
    Impl* impl;

};

class CNOID_EXPORT ImageViewBar : public ToolBar
{
public:
    static void initialize(ExtensionManager* ext);
    static ImageViewBar* instance();

    ImageableItem* getSelectedImageableItem();

private:
    class Impl;
    Impl* impl;

    friend class ImageView;

    ImageViewBar();
    ~ImageViewBar();
};

}

#endif
