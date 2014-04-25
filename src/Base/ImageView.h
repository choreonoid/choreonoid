/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_IMAGE_VIEW_H_INCLUDED
#define CNOID_BASE_IMAGE_VIEW_H_INCLUDED

#include <cnoid/View>
#include "exportdecl.h"

class QImage;

namespace cnoid {

class Image;
class ImageViewImpl;
    
class CNOID_EXPORT ImageView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    ImageView();
    ~ImageView();

    void setPixmap(const QPixmap& pixmap);
    void setImage(const Image& image);
    void setImage(const QImage& image);

    void setScalingEnabled(bool on);
    bool isScalingEnabled() const;
        
private:
    ImageViewImpl* impl;
};
}

#endif
