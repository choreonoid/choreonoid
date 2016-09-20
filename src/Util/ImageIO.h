/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_IMAGE_IO_H
#define CNOID_UTIL_IMAGE_IO_H

#include "Image.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ImageIO
{
public:
    ImageIO();

    void setUpsideDown(bool on) { isUpsideDown_ = on; }

    //! \todo implement this mode.
    void allocateAlphaComponent(bool on);
        
    void load(Image& image, const std::string& filename);
    void save(const Image& image, const std::string& filename);

private:
    bool isUpsideDown_;
};

}

#endif
