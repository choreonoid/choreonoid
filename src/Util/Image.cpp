/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "Image.h"
#include "ImageIO.h"
#include "Exception.h"
#include <fmt/format.h>

using namespace std;
using namespace cnoid;


Image::Image()
{
    width_ = 0;
    height_ = 0;
    numComponents_ = 3;
}


Image::Image(const Image& org)
    : pixels_(org.pixels_)
{
    width_ = org.width_;
    height_ = org.height_;
    numComponents_ = org.numComponents_;
}


Image::~Image()
{
    
}


Image& Image::operator=(const Image& rhs)
{
    pixels_ = rhs.pixels_;
    width_ = rhs.width_;
    height_ = rhs.height_;
    numComponents_ = rhs.numComponents_;
    return *this;
}


void Image::reset()
{
    pixels_.clear();
    width_ = 0;
    height_ = 0;
}


void Image::setSize(int width, int height, int nComponents)
{
    if(nComponents > 0 && nComponents <= 4){
        numComponents_ = nComponents;
    } else {
        exception_base exception;
        exception << error_info_message(
            fmt::format("Invalid number ({}) of image components", nComponents));
        BOOST_THROW_EXCEPTION(exception);
    }
    setSize(width, height);
}


void Image::setSize(int width, int height)
{
    width_ = width;
    height_ = height;
    pixels_.resize(numComponents_ * width_ * height_);
}


void Image::clear()
{
    std::fill(pixels_.begin(), pixels_.end(), 0);
}


void Image::applyVerticalFlip()
{
    const int heightHalf = height_ / 2;
    const int lineSize = width_ * numComponents_;
    unsigned char* upperLine = pixels();
    unsigned char* lowerLine = pixels() + lineSize * (height_ - 1);
    for(int y = 0; y < heightHalf; ++y){
        for(int x=0; x < lineSize; ++x){
            std::swap(upperLine[x], lowerLine[x]);
        }
        upperLine += lineSize;
        lowerLine -= lineSize;
    }
}


    
void Image::load(const std::string& filename)
{
    ImageIO iio;
    iio.load(*this, filename);
}


void Image::save(const std::string& filename) const
{
    ImageIO iio;
    iio.save(*this, filename);
}
