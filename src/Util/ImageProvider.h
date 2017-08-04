/*!
  @file
  @author Shizuko Hattori
*/

#ifndef CNOID_UTIL_IMAGE_PROVIDER_H
#define CNOID_UTIL_IMAGE_PROVIDER_H

#include <cnoid/Image>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ImageProvider
{
public:

    virtual ~ImageProvider() {};

    virtual const Image* getImage() = 0;

    virtual void notifyUpdate(){
        sigUpdated_();
    };

    SignalProxy<void()> sigUpdated() {
        return sigUpdated_;
    }

private:
    Signal<void()> sigUpdated_;

};

}

#endif
