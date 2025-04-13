#ifndef CNOID_GL_CAMERA_EFFECT_PLUGIN_IMAGE_FILTER_H
#define CNOID_GL_CAMERA_EFFECT_PLUGIN_IMAGE_FILTER_H

#include <cnoid/EigenTypes>
#include <cnoid/Image>
#include <cnoid/ValueTree>
#include <memory>
#include <random>
#include <QImage>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ImageProcessor
{
public:
    ImageProcessor();

    void initialize(int width, int height);

    void red(Image* image);
    void green(Image* image);
    void blue(Image* image);
    void white(Image* image);
    void black(Image* image);

    void salt(Image* image, const double& salt_amount);
    void random_salt(Image* image,
                     const double& salt_amount,
                     const double& salt_chance);
    void pepper(Image* image, const double& pepper_amount);
    void random_pepper(Image* image,
                       const double& pepper_amount,
                       const double& pepper_chance);
    void salt_pepper(Image* image,
                     const double& salt_amount,
                     const double& pepper_amount);
    void rgb(Image* image,
             const double& red,
             const double& green,
             const double& blue);
    void hsv(Image* image,
             const double& hue,
             const double& saturation,
             const double& value);
    void gaussian_noise(Image* image, const double& std_dev);
    void barrel_distortion(Image* image,
                           const double& coef_b,
                           const double& coef_d);
    void mosaic(Image* image, int kernel = 16);
    void random_mosaic(Image* image, const double& rate, int kernel = 16);

private:
    int width_;
    int height_;
    std::random_device seed_gen_;
    std::default_random_engine engine_;
    std::normal_distribution<> dist_;
};

class CNOID_EXPORT ImageFilter
{
public:
    ImageFilter();

    Vector3 hsv() const { return hsv_; }
    Vector3 rgb() const { return rgb_; }
    double std_dev() const { return std_dev_; }
    double salt_amount() const { return salt_amount_; }
    double salt_chance() const { return salt_chance_; }
    double pepper_amount() const { return pepper_amount_; }
    double pepper_chance() const { return pepper_chance_; }
    double coef_b() const { return coef_b_; }
    double coef_d() const { return coef_d_; }
    double mosaic_chance() const { return mosaic_chance_; }
    int kernel() const { return kernel_; }

    bool readCameraInfo(const Mapping* info);
    void updateImage(Image* image);

private:
    Vector3 hsv_;
    Vector3 rgb_;
    double std_dev_;
    double salt_amount_;
    double salt_chance_;
    double pepper_amount_;
    double pepper_chance_;
    double coef_b_;
    double coef_d_;
    double mosaic_chance_;
    int kernel_;

    ImageProcessor processor;
};

void toCnoidImage(Image* image, QImage q_image);
QImage toQImage(Image* image);

}  // namespace cnoid

#endif  // CNOID_GL_CAMERA_EFFECT_PLUGIN_IMAGE_FILTER_H
