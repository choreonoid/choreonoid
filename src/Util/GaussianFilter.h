/*! @file
  @brief Header file of gaussian filter functions
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_GAUSSIAN_FILTER_H_INCLUDED
#define CNOID_UTIL_GAUSSIAN_FILTER_H_INCLUDED

#include <cmath>
#include <vector>

namespace cnoid {

template <class T>
void setGaussWindow(T sigma, int range, std::vector<T>& out_window)
{
    const double mu = 0.0;
            
    out_window.resize(range * 2 + 1);

    T x = 0.0;
    for(int i = 0; i <= range; ++i){
        out_window[i + range] = exp(- (x - mu) * (x - mu) / (2.0 * sigma * sigma));
    }
    // normalization
    T area_half = out_window[range] / 2.0;
    for(int i=1; i <= range; ++i){
        area_half += out_window[i + range];
    }
    const T area = area_half * 2.0;

    out_window[range] /= area;
    for(int i=1; i <= range; ++i){
        T& v = out_window[i + range];
        v /= area;
        out_window[range - i] = v;
    }
}

        
template <class RESULTVECTOR, class SRCVECTOR, class ELEMENT, class T>
void applyGaussianFilter(RESULTVECTOR& result, const SRCVECTOR& src, std::vector<T>& gwin, ELEMENT zero)
{
    const int range = (gwin.size() - 1) / 2;
    const int size = src.size();
            
    // head
    for(int i=0; i < range; i++){
        ELEMENT v = zero; T ave = 0.0;
        for(int j = - i; j <= +range; j++){
            v += src[i+j] * gwin[j+range];
            ave += gwin[j+range];
        }
        result[i] = v / ave;
    }
            
    // body
    for(int i=range; i < size - range; i++){
        ELEMENT v = zero;
        for(int j=-range; j <= +range; j++){
            v += src[i+j] * gwin[j+range];
        }
        result[i] = v;
    }
            
    // tail
    for(int i = size - range; i < size; i++){
        ELEMENT v = zero; T ave = 0.0;
        for(int j=-range; j < size - i; j++){
            v += src[i+j] * gwin[j+range];
            ave += gwin[j+range];
        }
        result[i] = v / ave;
    }
}


template <class RESULTVECTOR, class SRCVECTOR, class ELEMENT, class T>
void applyGaussianFilter(RESULTVECTOR& result, const SRCVECTOR& src, T sigma, int range, ELEMENT zero)
{
    std::vector<T> gwin;
    setGaussWindow(sigma, range, gwin);
    applyGaussianFilter(result, src, gwin, zero);
}
}

#endif
