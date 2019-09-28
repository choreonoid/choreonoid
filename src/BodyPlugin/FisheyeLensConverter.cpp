/**
   \author Shizuko Hattori
   \author Shin'ichiro Nakaoka
*/

#include "FisheyeLensConverter.h"
#include <cmath>
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

static const bool DEBUG_MESSAGE2 = false;

int clamp(int i, int min, int max)
{
    return i < min ? min : i < max ? i : max - 1;
}

}


FisheyeLensConverter::FisheyeLensConverter()
{
    isImageRotationEnabled = false;
    isAntiAliasingEnabled = false;
}



void FisheyeLensConverter::initialize(int width_, int height_, double fov_, int screenWidth_)
{
    width = width_;
    height = height_;
    fov = fov_;
    screenWidth = screenWidth_;
    fisheyeLensMap.clear();
    fisheyeLensInterpolationMap.clear();

    screenImages.clear();
}


void FisheyeLensConverter::addScreenImage(std::shared_ptr<Image> image)
{
    screenImages.push_back(image);
}


void FisheyeLensConverter::setImageRotationEnabled(bool on)
{
    if(on != isImageRotationEnabled){
        fisheyeLensMap.clear();
        fisheyeLensInterpolationMap.clear();
        isImageRotationEnabled = on;
    }
}


void FisheyeLensConverter::setAntiAliasingEnabled(bool on)
{
    isAntiAliasingEnabled = on;
}


void FisheyeLensConverter::setCornerPoint(int i, Corner corner)
{
    switch(corner){

    case FRONT_UR:
    case FRONT_UL:
    case FRONT_DR:
    case FRONT_DL:
        screenId[i] = FRONT_SCREEN;
        break;

    case LEFT_UR:
    case LEFT_UL:
    case LEFT_DR:
    case LEFT_DL:
        screenId[i] = LEFT_SCREEN;
        break;

    case RIGHT_UR:
    case RIGHT_UL:
    case RIGHT_DR:
    case RIGHT_DL:
        screenId[i] = RIGHT_SCREEN;
        break;

    case BOTTOM_UR:
    case BOTTOM_UL:
    case BOTTOM_DR:
    case BOTTOM_DL:
        screenId[i] = BOTTOM_SCREEN;
        break;

    case TOP_UR:
    case TOP_UL:
    case TOP_DR:
    case TOP_DL:
        screenId[i] = TOP_SCREEN;
        break;

    case BACK_UR:
    case BACK_UL:
    case BACK_DR:
    case BACK_DL:
        screenId[i] = BACK_SCREEN;
        break;
    }

    switch(corner){

    case FRONT_UL:
    case LEFT_UL:
    case RIGHT_UL:
    case BOTTOM_UL:
    case TOP_UL:
    case BACK_UL:
        npx[i] = npy[i] = 0;
        break;
        
    case FRONT_UR:
    case LEFT_UR:
    case RIGHT_UR:
    case BOTTOM_UR:
    case TOP_UR:
    case BACK_UR:
        npx[i] = screenWidth - 1;
        npy[i] = 0;
        break;

    case FRONT_DL:
    case LEFT_DL:
    case RIGHT_DL:
    case BOTTOM_DL:
    case TOP_DL:
    case BACK_DL:
        npx[i] = 0;
        npy[i] = screenWidth - 1;
        break;

    case FRONT_DR:
    case LEFT_DR:
    case RIGHT_DR:
    case BOTTOM_DR:
    case TOP_DR:
    case BACK_DR:
        npx[i] = screenWidth - 1;
        npy[i] = screenWidth - 1;
        break;
    }
}


void FisheyeLensConverter::setCubeCorner(Corner corner0, Corner corner1, Corner corner2, Corner corner3)
{
    setCornerPoint(0, corner0);
    setCornerPoint(1, corner1);
    setCornerPoint(2, corner2);
    setCornerPoint(3, corner3);
}


void FisheyeLensConverter::setCenter(int id, double sx, double sy)
{
    screenId[0] = screenId[1] = screenId[2] = screenId[3] = id;
    npx[0] = sx;              npy[0] = sy;
    npx[1] = npx[0]+1;        npy[1] = npy[0];
    npx[2] = npx[0];          npy[2] = npy[0]+1;
    npx[3] = npx[1];          npy[3] = npy[2];
}


void FisheyeLensConverter::setVerticalBorder(int id0, int id1, double sy)
{
    screenId[0] = screenId[2] = id0;
    screenId[1] = screenId[3] = id1;
    npx[0] = screenWidth - 1;  npy[0] = sy;
    npx[1] = 0;                npy[1] = npy[0];
    npx[2] = screenWidth - 1;  npy[2] = npy[0]+1;
    npx[3] = 0;                npy[3] = npy[2];
}


void FisheyeLensConverter::setHorizontalBorder(int id0, int id1, double sx)
{
    screenId[0] = screenId[1] = id0;
    screenId[2] = screenId[3] = id1;
    npx[0] = sx;              npy[0] = screenWidth - 1;
    npx[1] = npx[0]+1;        npy[1] = screenWidth - 1;
    npx[2] = npx[0];          npy[2] = 0;
    npx[3] = npx[1];          npy[3] = 0;
}


void  FisheyeLensConverter::convertImage(Image* image)
{
    if(!isAntiAliasingEnabled){
        convertImageWithoutAntiAliasing(image);
    } else {
        convertImageWithAntiAliasing(image);
    }
}


void FisheyeLensConverter::convertImageWithoutAntiAliasing(Image* image)
{
    image->setSize(width, height, 3);
    unsigned char* pixels = image->pixels();

    if(fisheyeLensMap.empty()){
        fisheyeLensMap.resize(height);
        for(int i=0; i<height; i++){
            fisheyeLensMap[i].resize(width);
        }

        double height2 = height/2.0;
        double screenWidth2 = screenWidth / 2.0;
        double sw22 = screenWidth2 * screenWidth2;
        double r = fov / height;

        for(int j=0; j<height; j++){
            double y = j - height2 + 0.5;
            for(int i=0; i<width; i++){
                bool picked = false;

                int screenId;
                int ii,jj;
                if(i<height){
                    double x = i - height2 + 0.5;;
                    double l = sqrt(x*x+y*y);

                    if(l<=height2){
                        double tanTheta;
                        if(l==0){
                            tanTheta = 0.0;
                        } else {
                            tanTheta = screenWidth2 / l * tan(l*r);
                        }
                        double xx = x*tanTheta;
                        double yy = y*tanTheta;
                        ii = nearbyint(xx + screenWidth2-0.5);
                        jj = nearbyint(yy + screenWidth2-0.5);
                        if(0<=ii && ii<screenWidth && 0<=jj && jj<screenWidth){
                            screenId = FRONT_SCREEN;
                            picked = true;
                        }else if(ii >= screenWidth){  //right
                            double xx_ = sw22 / xx;
                            double yy_ = screenWidth2 * yy / xx;
                            int iir = nearbyint(-xx_ + screenWidth2-0.5);
                            int jjr = nearbyint(yy_ + screenWidth2-0.5);
                            if( 0 <= jjr && jjr < screenWidth){
                                screenId = RIGHT_SCREEN;
                                ii = clamp(iir, 0, screenWidth);
                                jj = jjr;
                                picked = true;
                            }
                        }else if(ii < 0){    //left
                            double xx_ = sw22 / -xx;
                            double yy_ = screenWidth2 * yy / -xx;
                            int iil = nearbyint(xx_ +screenWidth2-0.5);
                            int jjl = nearbyint(yy_ + screenWidth2-0.5);
                            if( 0 <= jjl && jjl < screenWidth){
                                screenId = LEFT_SCREEN;
                                ii = clamp(iil, 0, screenWidth);
                                jj = jjl;
                                picked = true;
                            }
                        }
                        if(!picked && jj >= screenWidth){    //bottom
                            double xx_ = screenWidth2 * xx / yy;
                            double yy_ = sw22 / yy;
                            int iib = nearbyint(xx_ + screenWidth2-0.5);
                            int jjb = nearbyint(-yy_ + screenWidth2-0.5);
                            screenId = BOTTOM_SCREEN;
                            ii = clamp(iib, 0, screenWidth);
                            jj = clamp(jjb, 0, screenWidth);
                            picked = true;
                        }else if(!picked && jj < 0){    //top
                            double xx_ = screenWidth2 * xx / -yy;
                            double yy_ = sw22 / -yy;
                            int iit = nearbyint(xx_ + screenWidth2-0.5);
                            int jjt = nearbyint(yy_ + screenWidth2-0.5);
                            screenId = TOP_SCREEN;
                            ii = clamp(iit, 0, screenWidth);
                            jj = clamp(jjt, 0, screenWidth);
                            picked = true;
                        }
                        if(DEBUG_MESSAGE2 && !picked){
                            cout << "Could not pick it up. " << i << " " << j << endl;
                        }
                    }
                }else{
                    double x = i - height - height2 +0.5;
                    double l = sqrt(x*x+y*y);
                    if(l<=height2){
                        double tanTheta;
                        if(l==0){
                            tanTheta = 0.0;
                        } else {
                            tanTheta = screenWidth2 / l * tan(l*r);
                        }
                        double xx = x*tanTheta;
                        double yy = y*tanTheta;
                        ii = nearbyint(xx + screenWidth2-0.5);
                        jj = nearbyint(yy + screenWidth2-0.5);
                        if(0<=ii && ii<screenWidth && 0<=jj && jj<screenWidth){
                            screenId = BACK_SCREEN;
                            picked = true;
                        }else if(ii >= screenWidth){
                            double xx_ = sw22 / xx;
                            double yy_ = screenWidth2 * yy / xx;
                            int iir = nearbyint(-xx_ + screenWidth2-0.5);
                            int jjr = nearbyint(yy_ + screenWidth2-0.5);
                            if( 0 <= jjr && jjr < screenWidth){
                                screenId = LEFT_SCREEN;
                                ii = clamp(iir, 0, screenWidth);
                                jj = jjr;
                                picked = true;
                            }
                        }else if(ii < 0){
                            double xx_ = sw22 / -xx;
                            double yy_ = screenWidth2 * yy / -xx;
                            int iil = nearbyint(xx_ +screenWidth2-0.5);
                            int jjl = nearbyint(yy_ + screenWidth2-0.5);
                            if( 0 <= jjl && jjl < screenWidth){
                                screenId = RIGHT_SCREEN;
                                ii = clamp(iil, 0, screenWidth);
                                jj = jjl;
                                picked = true;
                            }
                        }
                        if(!picked && jj >= screenWidth){
                            double xx_ = screenWidth2 * xx / yy;
                            double yy_ = sw22 / yy;
                            int iib = nearbyint(-xx_ + screenWidth2-0.5);
                            int jjb = nearbyint(yy_ + screenWidth2-0.5);
                            screenId = BOTTOM_SCREEN;
                            ii = clamp(iib, 0, screenWidth);
                            jj = clamp(jjb, 0, screenWidth);
                            picked = true;
                        }else if(!picked && jj < 0){
                            double xx_ = screenWidth2 * xx / -yy;
                            double yy_ = sw22 / -yy;
                            int iit = nearbyint(-xx_ + screenWidth2-0.5);
                            int jjt = nearbyint(-yy_ + screenWidth2-0.5);
                            screenId = TOP_SCREEN;
                            ii = clamp(iit, 0, screenWidth);
                            jj = clamp(jjt, 0, screenWidth);
                            picked = true;
                        }
                        if(DEBUG_MESSAGE2 && !picked){
                            cout << "Could not pick it up. " << i << " " << j << endl;
                        }
                    }
                }

                int i_, j_;
                if(!isImageRotationEnabled){
                    i_ = i;
                    j_ = j;
                }else{
                    if(i<height){
                        i_ = j;
                        j_ = height - 1 - i;
                    }else{
                        i_ = height - 1 - j + height;
                        j_ = i - height;
                    }
                }
                unsigned char* pix = &pixels[(i_+j_*width)*3];
                if(picked){
                    unsigned char* tempPixels = screenImages[screenId]->pixels();
                    unsigned char* tempPix = &tempPixels[(int)((ii + jj * screenWidth) * 3)];
                    pix[0] = tempPix[0];
                    pix[1] = tempPix[1];
                    pix[2] = tempPix[2];
                    fisheyeLensMap[j_][i_].screenId = screenId;
                    fisheyeLensMap[j_][i_].ix = ii;
                    fisheyeLensMap[j_][i_].iy = jj;
                }else{
                    pix[0] = pix[1] = pix[2] = 0;
                    fisheyeLensMap[j_][i_].screenId = NO_SCREEN;
                }
            }
        }
    }else{
        for(int j=0; j<height; j++){
            for(int i=0; i<width; i++){
                unsigned char* pix = &pixels[(i+j*width)*3];
                ScreenIndex& screenIndex = fisheyeLensMap[j][i];
                if(screenIndex.screenId != NO_SCREEN){
                    unsigned char* tempPixels = screenImages[screenIndex.screenId]->pixels();
                    unsigned char* tempPix = &tempPixels[(int)((screenIndex.ix + screenIndex.iy * screenWidth) * 3)];
                    pix[0] = tempPix[0];
                    pix[1] = tempPix[1];
                    pix[2] = tempPix[2];
                }else{
                    pix[0] = pix[1] = pix[2] = 0;
                }
            }
        }
    }
}


void  FisheyeLensConverter::convertImageWithAntiAliasing(Image* image)
{
    image->setSize(width, height, 3);
    unsigned char* pixels = image->pixels();

    if(fisheyeLensInterpolationMap.empty()){
        fisheyeLensInterpolationMap.resize(height);
        for(int i=0; i<height; i++){
            fisheyeLensInterpolationMap[i].resize(width);
        }

        double height2 = height/2.0;
        double screenWidth2 = screenWidth / 2.0;
        double sw22 = screenWidth2 * screenWidth2;
        double r = fov / height;

        for(int j=0; j<height; j++){
            double y = j - height2 +0.5;
            for(int i=0; i<width; i++){
                bool picked = false;
                double sx,sy;
                int ii,jj;
                if(i<height){  //front
                    double x = i - height2+0.5;
                    double l = sqrt(x*x+y*y);

                    if(l<=height2){
                        double tanTheta;
                        if(l==0){
                            tanTheta = 0.0;
                        } else {
                            tanTheta = screenWidth2 / l * tan(l*r);
                        }
                        double xx = x*tanTheta;
                        double yy = y*tanTheta;
                        ii = nearbyint(xx + screenWidth2-0.5);
                        jj = nearbyint(yy + screenWidth2-0.5);
                        if(0<=ii && ii<screenWidth && 0<=jj && jj<screenWidth){  //center
                            sx = xx + screenWidth2-0.5;
                            sy = yy + screenWidth2-0.5;
                            if(sx<0){
                                if(sy<0){
                                    setCubeCorner(TOP_DL, TOP_DL, LEFT_UR, FRONT_UL);
                                }else if(sy>=screenWidth-1){
                                    setCubeCorner(LEFT_DR, FRONT_DL, BOTTOM_UL, BOTTOM_UL);
                                }else{
                                    setVerticalBorder(LEFT_SCREEN, FRONT_SCREEN, sy);
                                }
                            }else if(sx>=screenWidth-1){
                                if(sy<0){
                                    setCubeCorner(TOP_DR, TOP_DR, FRONT_UR, RIGHT_UL);
                                }else if(sy>=screenWidth-1){
                                    setCubeCorner(FRONT_DR, RIGHT_DL, BOTTOM_UR, BOTTOM_UR);
                                }else{
                                    setVerticalBorder(FRONT_SCREEN, RIGHT_SCREEN, sy);
                                  }
                            }else{
                                if(sy<0){
                                    setHorizontalBorder(TOP_SCREEN, FRONT_SCREEN, sx);
                                }else if(sy>=screenWidth-1){
                                    setHorizontalBorder(FRONT_SCREEN, BOTTOM_SCREEN, sx);
                                }else{
                                    setCenter(FRONT_SCREEN, sx, sy);
                                }
                            }
                            picked = true;
                        }else if(ii >= screenWidth){  //right
                            double xx_ = sw22 / xx;
                            double yy_ = screenWidth2 * yy / xx;
                            int iir = nearbyint(-xx_ + screenWidth2-0.5);
                            int jjr = nearbyint(yy_ + screenWidth2-0.5);
                            if( 0 <= jjr && jjr < screenWidth){
                                sx = -xx_ + screenWidth2-0.5;
                                sy = yy_ + screenWidth2-0.5;
                                if(sx<0){
                                    if(sy<0){
                                        setCubeCorner(TOP_DR, TOP_DR, FRONT_UR, RIGHT_UL);
                                    }else if(sy>=screenWidth-1){
                                        setCubeCorner(FRONT_DR, RIGHT_DL, BOTTOM_UR, BOTTOM_UR);
                                    }else{
                                        setVerticalBorder(FRONT_SCREEN, RIGHT_SCREEN, sy);
                                    }
                                }else{
                                    if(sy<0){
                                        screenId[0] = screenId[1] = TOP_SCREEN;
                                        screenId[2] = screenId[3] = RIGHT_SCREEN;
                                        npx[0] = screenWidth - 1;    npy[0] = screenWidth - 1 - (int)sx;
                                        npx[1] = screenWidth - 1;    npy[1] = npy[0] - 1;
                                        npx[2] = sx;                 npy[2] = 0;
                                        npx[3] = npx[2]+1;           npy[3] = 0;
                                    }else if(sy>=screenWidth-1){
                                        screenId[0] = screenId[1] = RIGHT_SCREEN;
                                        screenId[2] = screenId[3] = BOTTOM_SCREEN;
                                        npx[0] = sx;                 npy[0] = screenWidth - 1;
                                        npx[1] = npx[0]+1;           npy[1] = screenWidth - 1;
                                        npx[2] = screenWidth - 1;    npy[2] = sx;
                                        npx[3] = screenWidth - 1;    npy[3] = npy[2] + 1;
                                    }else{
                                        setCenter(RIGHT_SCREEN, sx, sy);
                                    }
                                }
                                picked = true;
                            }
                        }else if(ii < 0){    //left
                            double xx_ = sw22 / -xx;
                            double yy_ = screenWidth2 * yy / -xx;
                            int iil = nearbyint(xx_ +screenWidth2-0.5);
                            int jjl = nearbyint(yy_ + screenWidth2-0.5);
                            if( 0 <= jjl && jjl < screenWidth){
                                sx = xx_ + screenWidth2-0.5;
                                sy = yy_ + screenWidth2-0.5;
                                if(sx>=screenWidth-1){
                                    if(sy<0){
                                        setCubeCorner(TOP_DL, TOP_DL, LEFT_UR, FRONT_UL);
                                    }else if(sy>=screenWidth-1){
                                        setCubeCorner(LEFT_DR, FRONT_DL, BOTTOM_UL, BOTTOM_UL);
                                    }else{
                                        setVerticalBorder(LEFT_SCREEN, FRONT_SCREEN, sy);
                                    }
                                }else{
                                    if(sy<0){
                                        screenId[0] = screenId[1] = TOP_SCREEN;
                                        screenId[2] = screenId[3] = LEFT_SCREEN;
                                        npx[0] = 0;    npy[0] = sx;
                                        npx[1] = 0;    npy[1] = npy[0] + 1;
                                        npx[2] = sx;                 npy[2] = 0;
                                        npx[3] = npx[2]+1;           npy[3] = 0;
                                    }else if(sy>=screenWidth-1){
                                        screenId[0] = screenId[1] = LEFT_SCREEN;
                                        screenId[2] = screenId[3] = BOTTOM_SCREEN;
                                        npx[0] = sx;                 npy[0] = screenWidth - 1;
                                        npx[1] = npx[0]+1;           npy[1] = screenWidth - 1;
                                        npx[2] = 0;                  npy[2] = screenWidth - 1 - (int)sx;
                                        npx[3] = 0;                  npy[3] = npy[2] - 1;
                                    }else{
                                        setCenter(LEFT_SCREEN, sx, sy);
                                    }
                                }
                                picked = true;
                            }
                        }
                        if(!picked && jj >= screenWidth){    //bottom
                            double xx_ = screenWidth2 * xx / yy;
                            double yy_ = sw22 / yy;
                            sx = xx_ + screenWidth2-0.5;
                            sy = -yy_ + screenWidth2-0.5;
                            if(sy<0){
                                if(sx<0){
                                    setCubeCorner(FRONT_DL, FRONT_DL, LEFT_DR, BOTTOM_UL);
                                }else if(sx>=screenWidth-1){
                                    setCubeCorner(FRONT_DR, FRONT_DR, BOTTOM_UR, RIGHT_DL);
                                }else{
                                    setHorizontalBorder(FRONT_SCREEN, BOTTOM_SCREEN, sx);
                                }
                            }else{
                                if(sx<0){
                                    screenId[0] = screenId[2] = LEFT_SCREEN;
                                    screenId[1] = screenId[3] = BOTTOM_SCREEN;
                                    npx[0] = screenWidth - 1 -(int)sy;   npy[0] = screenWidth - 1;
                                    npx[1] = 0;                          npy[1] = sy;
                                    npx[2] = npx[0] - 1;                 npy[2] = screenWidth - 1;
                                    npx[3] = 0;                          npy[3] = npy[1]+1;
                                }else if(sx>=screenWidth-1){
                                    screenId[0] = screenId[2] = BOTTOM_SCREEN;
                                    screenId[1] = screenId[3] = RIGHT_SCREEN;
                                    npx[0] = screenWidth-1;     npy[0] = sy;
                                    npx[1] = sy;                npy[1] = screenWidth - 1;
                                    npx[2] = screenWidth - 1;   npy[2] = npy[0] + 1;
                                    npx[3] = npx[1] + 1;        npy[3] = screenWidth - 1;
                                }else{
                                    setCenter(BOTTOM_SCREEN, sx, sy);
                                }
                            }
                            picked = true;
                        }
                        if(!picked && jj < 0){    //top
                            double xx_ = screenWidth2 * xx / -yy;
                            double yy_ = sw22 / -yy;
                            sx = xx_ + screenWidth2-0.5;
                            sy = yy_ + screenWidth2-0.5;
                            if(sy>=screenWidth-1){
                                if(sx<0){
                                    setCubeCorner(LEFT_UR, TOP_DL, FRONT_UL, FRONT_UL);
                                }else if(sx>=screenWidth-1){
                                    setCubeCorner(TOP_DR, RIGHT_UL, FRONT_UR, FRONT_UR);
                                }else{
                                    setHorizontalBorder(TOP_SCREEN, FRONT_SCREEN, sx);
                                }
                            }else{
                                if(sx<0){
                                    screenId[0] = screenId[2] = LEFT_SCREEN;
                                    screenId[1] = screenId[3] = TOP_SCREEN;
                                    npx[0] = sy;           npy[0] = 0;
                                    npx[1] = 0;            npy[1] = sy;
                                    npx[2] = npx[0] + 1;   npy[2] = 0;
                                    npx[3] = 0;            npy[3] = npy[1] + 1;
                                }else if(sx>=screenWidth-1){
                                    screenId[0] = screenId[2] = TOP_SCREEN;
                                    screenId[1] = screenId[3] = RIGHT_SCREEN;
                                    npx[0] = screenWidth - 1;            npy[0] = sy;
                                    npx[1] = screenWidth - 1 - (int)sy;  npy[1] = 0;
                                    npx[2] = screenWidth - 1;            npy[2] = npy[0] + 1;
                                    npx[3] = npx[1] - 1;                 npy[3] = 0;
                                }else{
                                    setCenter(TOP_SCREEN, sx, sy);
                                }
                            }
                            picked = true;
                        }
                        if(DEBUG_MESSAGE2 && !picked){
                            cout << "Could not pick it up. " << i << " " << j << endl;
                        }
                    }
                }else{  //back
                    double x = i - height - height2 + 0.5;
                    double l = sqrt(x*x+y*y);
                    if(l<=height2){
                        double tanTheta;
                        if(l==0){
                            tanTheta = 0.0;
                        } else {
                            tanTheta = screenWidth2 / l * tan(l*r);
                        }
                        double xx = x*tanTheta;
                        double yy = y*tanTheta;
                        ii = nearbyint(xx + screenWidth2-0.5);
                        jj = nearbyint(yy + screenWidth2-0.5);
                        if(0<=ii && ii<screenWidth && 0<=jj && jj<screenWidth){  // center
                            sx = xx + screenWidth2-0.5;
                            sy = yy + screenWidth2-0.5;
                            if(sx<0){
                                if(sy<0){
                                    setCubeCorner(TOP_UR, TOP_UR, RIGHT_UR, BACK_UL);
                                }else if(sy>=screenWidth-1){
                                    setCubeCorner(RIGHT_DR, BACK_DL, BOTTOM_DR, BOTTOM_DR);
                                }else{
                                    setVerticalBorder(RIGHT_SCREEN, BACK_SCREEN, sy);
                                }
                            }else if(sx>=screenWidth-1){
                                if(sy<0){
                                    setCubeCorner(TOP_UL, TOP_UL, BACK_UR, LEFT_UL);
                                }else if(sy>=screenWidth-1){
                                    setCubeCorner(BACK_DR, LEFT_DL, BOTTOM_DL, BOTTOM_DL);
                                }else{
                                    setVerticalBorder(BACK_SCREEN, LEFT_SCREEN, sy);
                                }
                            }else{
                                if(sy<0){
                                    screenId[0] = screenId[1] = TOP_SCREEN;
                                    screenId[2] = screenId[3] = BACK_SCREEN;
                                    npx[0] = screenWidth - 1 -(int)sx;    npy[0] = 0;
                                    npx[1] = npx[0] - 1;                  npy[1] = 0;
                                    npx[2] = sx;                          npy[2] = 0;
                                    npx[3] = npx[2] + 1;                  npy[3] = 0;
                                }else if(sy>=screenWidth-1){
                                    screenId[0] = screenId[1] = BACK_SCREEN;
                                    screenId[2] = screenId[3] = BOTTOM_SCREEN;
                                    npx[0] = sx;                          npy[0] = screenWidth - 1;
                                    npx[1] = npx[0] + 1;                  npy[1] = screenWidth - 1;
                                    npx[2] = screenWidth - 1 -(int)sx;;   npy[2] = screenWidth - 1;
                                    npx[3] = npx[2] - 1;                  npy[3] = screenWidth - 1;
                                }else{
                                    setCenter(BACK_SCREEN, sx, sy);
                                }
                            }
                            picked = true;
                        }else if(ii >= screenWidth){  //right
                            double xx_ = sw22 / xx;
                            double yy_ = screenWidth2 * yy / xx;
                            int iir = nearbyint(-xx_ + screenWidth2-0.5);
                            int jjr = nearbyint(yy_ + screenWidth2-0.5);
                            if( 0 <= jjr && jjr < screenWidth){
                                sx = -xx_ + screenWidth2-0.5;
                                sy = yy_ + screenWidth2-0.5;
                                if(sx<0){
                                    if(sy<0){
                                        setCubeCorner(TOP_UL, TOP_UL, BACK_UR, LEFT_UL);
                                    }else if(sy>=screenWidth-1){
                                        setCubeCorner(BACK_DR, LEFT_DL, BOTTOM_DL, BOTTOM_DL);
                                    }else{
                                        setVerticalBorder(BACK_SCREEN, LEFT_SCREEN, sy);
                                    }
                                }else{
                                    if(sy<0){
                                        screenId[0] = screenId[1] = TOP_SCREEN;
                                        screenId[2] = screenId[3] = LEFT_SCREEN;
                                        npx[0] = 0;                  npy[0] = sx;
                                        npx[1] = 0;                  npy[1] = npy[0] + 1;
                                        npx[2] = sx;                 npy[2] = 0;
                                        npx[3] = npx[2] + 1;         npy[3] = 0;
                                    }else if(sy>=screenWidth-1){
                                        screenId[0] = screenId[1] = LEFT_SCREEN;
                                        screenId[2] = screenId[3] = BOTTOM_SCREEN;
                                        npx[0] = sx;                 npy[0] = screenWidth - 1;
                                        npx[1] = npx[0]+1;           npy[1] = screenWidth - 1;
                                        npx[2] = 0;                  npy[2] = screenWidth - 1 - (int)sx;
                                        npx[3] = 0;                  npy[3] = npy[2] - 1;
                                    }else{
                                        setCenter(LEFT_SCREEN, sx, sy);
                                    }
                                }
                                picked = true;
                            }
                        }else if(ii < 0){   //left
                            double xx_ = sw22 / -xx;
                            double yy_ = screenWidth2 * yy / -xx;
                            int iil = nearbyint(xx_ + screenWidth2-0.5);
                            int jjl = nearbyint(yy_ + screenWidth2-0.5);
                            if( 0 <= jjl && jjl < screenWidth){
                                sx = xx_ + screenWidth2-0.5;
                                sy = yy_ + screenWidth2-0.5;
                                if(sx>=screenWidth-1){
                                    if(sy<0){
                                        setCubeCorner(TOP_UR, TOP_UR, RIGHT_UR, BACK_UL);
                                    }else if(sy>=screenWidth-1){
                                        setCubeCorner(RIGHT_DR, BACK_DL, BOTTOM_DR, BOTTOM_DR);
                                    }else{
                                        setVerticalBorder(RIGHT_SCREEN, BACK_SCREEN, sy);
                                    }
                                }else{
                                    if(sy<0){
                                        screenId[0] = screenId[1] = TOP_SCREEN;
                                        screenId[2] = screenId[3] = RIGHT_SCREEN;
                                        npx[0] = screenWidth - 1;    npy[0] = screenWidth - 1 - (int)sx;
                                        npx[1] = screenWidth - 1;    npy[1] = npy[0] - 1;
                                        npx[2] = sx;                 npy[2] = 0;
                                        npx[3] = npx[2]+1;           npy[3] = 0;
                                    }else if(sy>=screenWidth-1){
                                        screenId[0] = screenId[1] = RIGHT_SCREEN;
                                        screenId[2] = screenId[3] = BOTTOM_SCREEN;
                                        npx[0] = sx;                 npy[0] = screenWidth - 1;
                                        npx[1] = npx[0]+1;           npy[1] = screenWidth - 1;
                                        npx[2] = screenWidth - 1;    npy[2] = sx;
                                        npx[3] = screenWidth - 1;    npy[3] = npy[2] + 1;
                                    }else{
                                        setCenter(RIGHT_SCREEN, sx, sy);
                                    }
                                }
                                picked = true;
                            }
                        }
                        if(!picked && jj >= screenWidth){    //bottom
                            double xx_ = screenWidth2 * xx / yy;
                            double yy_ = sw22 / yy;
                            sx = -xx_ + screenWidth2-0.5;
                            sy = yy_ + screenWidth2-0.5;
                            if(sy>=screenWidth-1){
                                if(sx<0){
                                    setCubeCorner(LEFT_DL, BOTTOM_DL, BACK_DR, BACK_DR);
                                }else if(sx>=screenWidth-1){
                                    setCubeCorner(BOTTOM_DR, RIGHT_DR, BACK_DL, BACK_DL);
                                }else{
                                    screenId[0] = screenId[1] = BOTTOM_SCREEN;
                                    screenId[2] = screenId[3] = BACK_SCREEN;
                                    npx[0] = sx;                         npy[0] = screenWidth - 1;
                                    npx[1] = npx[0]+1;                   npy[1] = screenWidth - 1;
                                    npx[2] = screenWidth - 1 - (int)sx;  npy[2] = screenWidth - 1;
                                    npx[3] = npx[2] - 1;                 npy[3] = screenWidth - 1;
                                }
                            }else{
                                if(sx<0){
                                    screenId[0] = screenId[2] = LEFT_SCREEN;
                                    screenId[1] = screenId[3] = BOTTOM_SCREEN;
                                    npx[0] = screenWidth - 1 -(int)sy;   npy[0] = screenWidth - 1;
                                    npx[1] = 0;                          npy[1] = sy;
                                    npx[2] = npx[0] - 1;                 npy[2] = screenWidth - 1;
                                    npx[3] = 0;                          npy[3] = npy[1]+1;
                                }else if(sx>=screenWidth-1){
                                    screenId[0] = screenId[2] = BOTTOM_SCREEN;
                                    screenId[1] = screenId[3] = RIGHT_SCREEN;
                                    npx[0] = screenWidth-1;     npy[0] = sy;
                                    npx[1] = sy;                npy[1] = screenWidth - 1;
                                    npx[2] = screenWidth - 1;   npy[2] = npy[0] + 1;
                                    npx[3] = npx[1] + 1;        npy[3] = screenWidth - 1;
                                }else{
                                    setCenter(BOTTOM_SCREEN, sx, sy);
                                }
                            }
                            picked = true;
                        }else if(!picked && jj < 0){   //top
                            double xx_ = screenWidth2 * xx / -yy;
                            double yy_ = sw22 / -yy;
                            sx = -xx_ + screenWidth2-0.5;
                            sy = -yy_ + screenWidth2-0.5;
                            if(sy<0){
                                if(sx<0){
                                    setCubeCorner(BACK_UR, BACK_UR, LEFT_UL, TOP_UL);
                                }else if(sx>=screenWidth-1){
                                    setCubeCorner(BACK_UL, BACK_UL, TOP_UR, TOP_UR);
                                }else{
                                    screenId[0] = screenId[1] = BACK_SCREEN;
                                    screenId[2] = screenId[3] = TOP_SCREEN;
                                    npx[0] = screenWidth - 1 - (int)sx;     npy[0] = 0;
                                    npx[1] = npx[0] - 1;                    npy[1] = 0;
                                    npx[2] = sx;                            npy[2] = 0;
                                    npx[3] = npx[2] + 1;                    npy[3] = 0;
                                }
                            }else{
                                if(sx<0){
                                    screenId[0] = screenId[2] = LEFT_SCREEN;
                                    screenId[1] = screenId[3] = TOP_SCREEN;
                                    npx[0] = sy;           npy[0] = 0;
                                    npx[1] = 0;            npy[1] = sy;
                                    npx[2] = npx[0] + 1;   npy[2] = 0;
                                    npx[3] = 0;            npy[3] = npy[1] + 1;
                                }else if(sx>=screenWidth-1){
                                    screenId[0] = screenId[2] = TOP_SCREEN;
                                    screenId[1] = screenId[3] = RIGHT_SCREEN;
                                    npx[0] = screenWidth - 1;            npy[0] = sy;
                                    npx[1] = screenWidth - 1 - (int)sy;  npy[1] = 0;
                                    npx[2] = screenWidth - 1;            npy[2] = npy[0] + 1;
                                    npx[3] = npx[1]-1;                   npy[3] = 0;
                                }else{
                                    setCenter(TOP_SCREEN, sx, sy);
                                }
                            }
                            picked = true;
                        }
                        if(DEBUG_MESSAGE2 && !picked){
                            cout << "Could not pick it up. " << i << " " << j << endl;
                        }
                    }
                }

                int i_, j_;
                if(!isImageRotationEnabled){
                    i_ = i;
                    j_ = j;
                }else{
                    if(i<height){
                        i_ = j;
                        j_ = height - 1 - i;
                    }else{
                        i_ = height - 1 - j + height;
                        j_ = i - height;
                    }
                }
                unsigned char* pix = &pixels[(i_+j_*width)*3];
                ScreenIndex4& map = fisheyeLensInterpolationMap[j_][i_];
                if(picked){
                    double dx, dy;
                    if(sx<0){
                        dx = sx + 1;
                    }else{
                        dx = sx - (int)sx;
                    }
                    if(sy<0){
                        dy = sy + 1;
                    }else{
                        dy = sy - (int)sy;
                    }
                    double bias[4];
                    bias[0] = (1.0-dx)*(1.0-dy);
                    bias[1] = dx*(1.0-dy);
                    bias[2] = (1.0-dx)*dy;
                    bias[3] = dx*dy;
                    double pixd[3] = {0.0,0.0,0.0};
                    for(int k=0; k<4; k++){
                        unsigned char* tempPixels = screenImages[screenId[k]]->pixels();
                        unsigned char* tempPix = &tempPixels[(int)((npx[k] + npy[k] * screenWidth) * 3)];
                        for(int kk=0; kk<3; kk++){
                            pixd[kk] += bias[k] * tempPix[kk];
                        }
                        map.screenIndex[k].screenId = screenId[k];
                        map.screenIndex[k].ix = npx[k];
                        map.screenIndex[k].iy = npy[k];
                        map.bias[k] = bias[k];
                    }
                    for(int kk=0; kk<3; kk++){
                        pix[kk] = nearbyint(pixd[kk]);
                    }
                }else{
                    pix[0] = pix[1] = pix[2] = 0;
                    map.screenIndex[0].screenId = NO_SCREEN;
                }
            }
        }
    }else{
        for(int j=0; j<height; j++){
            for(int i=0; i<width; i++){
                unsigned char* pix = &pixels[(i+j*width)*3];
                ScreenIndex4& map = fisheyeLensInterpolationMap[j][i];
                if(map.screenIndex[0].screenId != NO_SCREEN){
                    double pixd[3] = {0.0,0.0,0.0};
                    for(int k=0; k<4; k++){
                        unsigned char* tempPixels = screenImages[map.screenIndex[k].screenId]->pixels();
                        unsigned char* tempPix = &tempPixels[(int)((map.screenIndex[k].ix + map.screenIndex[k].iy * screenWidth) * 3)];
                        for(int kk=0; kk<3; kk++){
                            pixd[kk] += map.bias[k] * tempPix[kk];
                        }
                    }
                    for(int kk=0; kk<3; kk++){
                        pix[kk] = nearbyint(pixd[kk]);
                    }
                }else{
                    pix[0] = pix[1] = pix[2] = 0;
                }
            }
        }
    }
}
