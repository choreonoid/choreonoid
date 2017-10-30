/*!
  @file ImageConverter.h
  @brief Header file of Image Converter class
  @author K.FUKUDA
*/

#ifndef CNOID_UTIL_IMAGECONVERTER_H
#define CNOID_UTIL_IMAGECONVERTER_H

#include "VRML.h"
#include "exportdecl.h"

namespace cnoid {

class ImageConverter
{
private:
    bool initializeSFImage();
    bool loadPNG(const std::string & filePath);
    bool loadJPEG(const std::string & filePath);

public:
    SFImage* image;
    ImageConverter(void);
    ~ImageConverter(void);
    std::string message;

    CNOID_EXPORT SFImage* convert(const std::string& url);
};

}

#endif
