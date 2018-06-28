/**
   \author Shizuko Hattori
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_FISHEYE_LENS_CONVERTER_H
#define CNOID_BODYPLUGIN_FISHEYE_LENS_CONVERTER_H

#include <cnoid/Image>
#include <vector>
#include <memory>

namespace cnoid {

class FisheyeLensConverter
{
public:
    enum ScreenId {
        NO_SCREEN = -1, FRONT_SCREEN, LEFT_SCREEN, RIGHT_SCREEN, TOP_SCREEN, BOTTOM_SCREEN, BACK_SCREEN
    };

    FisheyeLensConverter();
    void initialize(int width, int height, double fov, int screenWidth);
    void addScreenImage(std::shared_ptr<Image> image);
    void setImageRotationEnabled(bool on);
    void setAntiAliasingEnabled(bool on);
    void convertImage(Image* image);

private:
    int width;
    int height;
    double fov;
    int screenWidth;
    std::vector<std::shared_ptr<Image>> screenImages;
    
    bool isImageRotationEnabled;
    bool isAntiAliasingEnabled;

    struct ScreenIndex{
        int screenId;
        int ix;
        int iy;
    };
    std::vector<std::vector<ScreenIndex>> fisheyeLensMap;

    // for Interpolation
    int screenId[4];
    int npx[4],npy[4];
    struct ScreenIndex4 {
        ScreenIndex screenIndex[4];
        double bias[4];
    };
    std::vector<std::vector<ScreenIndex4>> fisheyeLensInterpolationMap;

    enum Corner {
        FRONT_UR,  FRONT_UL,  FRONT_DR,  FRONT_DL,
        LEFT_UR,   LEFT_UL,   LEFT_DR,   LEFT_DL,
        RIGHT_UR,  RIGHT_UL,  RIGHT_DR,  RIGHT_DL,
        BOTTOM_UR, BOTTOM_UL, BOTTOM_DR, BOTTOM_DL,
        TOP_UR,    TOP_UL,    TOP_DR,    TOP_DL,
        BACK_UR,   BACK_UL,   BACK_DR,   BACK_DL
    };
    
    void setCornerPoint(int i, Corner corner);
    void setCubeCorner(Corner corner0, Corner corner1, Corner corner2, Corner corner3);
    void setCenter(int id, double sx, double sy);
    void setVerticalBorder(int id0, int id1, double sy);
    void setHorizontalBorder(int id0, int id1, double sx);
    void convertImageWithoutAntiAliasing(Image* image);
    void convertImageWithAntiAliasing(Image* image);
};

}

#endif
