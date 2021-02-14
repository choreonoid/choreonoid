#ifndef CNOID_UTIL_IMAGE_IO_H
#define CNOID_UTIL_IMAGE_IO_H

#include "Image.h"
#include "NullOut.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ImageIO
{
public:
    ImageIO();

    void setUpsideDown(bool on) { isUpsideDown_ = on; }

    //! \todo implement this mode.
    void allocateAlphaComponent(bool on);
        
    bool load(Image& image, const std::string& filename, std::ostream& os = nullout());
    bool save(const Image& image, const std::string& filename, std::ostream& os = nullout());

private:
    bool isUpsideDown_;
};

}

#endif
