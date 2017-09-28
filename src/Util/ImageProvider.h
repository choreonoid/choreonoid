/*!
  @file
  @author Shizuko Hattori
*/

#ifndef CNOID_UTIL_IMAGE_PROVIDER_H
#define CNOID_UTIL_IMAGE_PROVIDER_H

#include "Image.h"
#include "Signal.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ImageProvider
{
public:
    virtual ~ImageProvider();

    virtual const Image* getImage() = 0;

    void notifyImageUpdate(){
        sigImageUpdated_();
    };

    SignalProxy<void()> sigImageUpdated() {
        return sigImageUpdated_;
    }

private:
    Signal<void()> sigImageUpdated_;
};

}

#endif
