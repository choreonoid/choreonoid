#ifndef CNOID_BASE_IMAGE_VIEW_H
#define CNOID_BASE_IMAGE_VIEW_H

#include "View.h"
#include "ToolBar.h"
#include "exportdecl.h"

class QImage;

namespace cnoid {

class Image;
class ImageableItem;
class ImageViewImpl;
class ImageViewBarImpl;
    
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

    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
        
private:
    ImageViewImpl* impl;

};

class CNOID_EXPORT ImageViewBar : public ToolBar
{
public:
    static void initialize(ExtensionManager* ext);
    static ImageViewBar* instance();

    ImageableItem* getSelectedImageableItem();

private:
    ImageViewBarImpl* impl;

    ImageViewBar();
    ~ImageViewBar();

};

}

#endif
