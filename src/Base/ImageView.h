/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_IMAGE_VIEW_H
#define CNOID_BASE_IMAGE_VIEW_H

#include "View.h"
#include "Item.h"
#include "ToolBar.h"
#include <cnoid/ImageProvider>
#include "exportdecl.h"

class QImage;

namespace cnoid {

class Image;
class ImageViewImpl;
    
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

    ImageProvider* getImageProvider();
    void setImageProvider(ImageProvider* imageProvider, Item* item);

    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
        
private:
    ImageViewImpl* impl;

};

class ImageViewBarImpl;
class CNOID_EXPORT ImageViewBar : public ToolBar
{
public:
    static void initialize(ExtensionManager* ext);
    static ImageViewBar* instance();

    ImageProvider* getSelectedImageProvider();
    Item* getImageProviderItem(ImageProvider* imageProvider);

private:
    ImageViewBarImpl* impl;

    ImageViewBar();
    ~ImageViewBar();

};

}

#endif
