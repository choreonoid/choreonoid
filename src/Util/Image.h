/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_IMAGE_H_INCLUDED
#define CNOID_UTIL_IMAGE_H_INCLUDED

#include <string>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Image
{
public:
    Image();
    Image(const Image& org);
    virtual ~Image();

    Image& operator=(const Image& rhs);

    void reset();
    bool empty() const { return pixels_.empty(); }

    unsigned char* pixels() { return &pixels_.front(); }
    const unsigned char* pixels() const { return &pixels_.front(); }

    int width() const { return width_; }
    int height() const { return height_; }
    int numComponents() const { return numComponents_; }
    bool hasAlphaComponent() const { return (numComponents() % 2) == 0; }

    void setSize(int width, int height, int nComponents);
    void setSize(int width, int height);

    void clear();
    void applyVerticalFlip();

    void load(const std::string& filename);

private:
    std::vector<unsigned char> pixels_;
    int width_;
    int height_;
    int numComponents_;
};
}

#endif
