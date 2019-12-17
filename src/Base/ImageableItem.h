#ifndef CNOID_BASE_IMAGEABLE_ITEM_H
#define CNOID_BASE_IMAGEABLE_ITEM_H

#include <cnoid/Signal>

namespace cnoid {

class Image;

class ImageableItem
{
public:
    virtual const Image* getImage() = 0;
    virtual SignalProxy<void()> sigImageUpdated() = 0;
};

}

#endif
