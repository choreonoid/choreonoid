/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_RANGE_LIMITER_H_INCLUDED
#define CNOID_UTIL_RANGE_LIMITER_H_INCLUDED

#include <cmath>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class RangeLimiter
{
public:
    template <class TArray>
    bool apply(TArray& valueSeq, double upperLimit, double lowerLimit, double limitGrad, double edgeGradRatio = 0.5);
            
private:
    double uL;
    double lL;
    double r;
            
    struct TSegment {
        int begin;
        int end;
        double upper;
        double lower;
        double uc;
        double lc;
        bool isOver;
    };
            
    std::vector<TSegment> segments;

    CNOID_EXPORT void calcCrossPoint(TSegment* seg);
    CNOID_EXPORT TSegment* mergeSegment(TSegment* seg, TSegment* prevSeg);
            
    template <class TArray>
    TSegment* finalizeSegment(TSegment* seg, int endFrame, TArray& valueSeq);
            
};

template <class TArray>
RangeLimiter::TSegment* RangeLimiter::finalizeSegment
(RangeLimiter::TSegment* seg, int endFrame, TArray& valueSeq)
{
    seg->end = endFrame;
    int backFrame = seg->begin - 1;
    TSegment* prevSeg = &segments[segments.size() - 2];
    while(backFrame >= 0){
        if(backFrame < prevSeg->end){
            return mergeSegment(seg, prevSeg);
        }
        double v = valueSeq[backFrame];
        backFrame--;
        if((v < seg->uc) && (v > seg->lc)){
            break;
        }
    }
            
    backFrame++;
    seg->begin = backFrame;
            
    return seg;
}
        
template <class TArray>
bool RangeLimiter::apply
(TArray& valueSeq, double upperLimit, double lowerLimit, double limitGrad, double edgeGradRatio)
{
    r = limitGrad;
    uL = upperLimit;
    lL = lowerLimit;
            
    segments.clear();
            
    TSegment dummy;
    dummy.isOver = false;
    dummy.end = -1;
    segments.push_back(dummy);
            
    segments.push_back(TSegment());
            
    TSegment* seg = &segments.back();
            
    seg->begin = 0;
    seg->upper = uL;
    seg->lower = lL;
    seg->isOver = false;
    calcCrossPoint(seg);
            
    int frame = 0;
    int n = valueSeq.size();
            
    while(frame < n){
        double v = valueSeq[frame];
                
        bool isOver = false;
        if(v > seg->upper){
            isOver = true;
            seg->upper = v;
        } else if(v < seg->lower){
            isOver = true;
            seg->lower = v;
        }
                
        if(isOver){
            calcCrossPoint(seg);
            if(!seg->isOver){
                seg->isOver = true;
                seg->begin = frame;
            }
        } else if(seg->isOver) {
            if((v < seg->uc) && (v > seg->lc)){
                TSegment* finalized = finalizeSegment(seg, frame, valueSeq);
                        
                if(finalized != seg){
                    seg = finalized;
                    continue;
                } else {
                    TSegment nextSeg;
                    nextSeg.begin = finalized->end;
                    nextSeg.upper = uL;
                    nextSeg.lower = lL;
                    nextSeg.isOver = false;
                    calcCrossPoint(&nextSeg);
                    segments.push_back(nextSeg);
                    seg = &segments.back();
                }
            }
        }
        frame++;
    }
            
    if(seg->isOver){
        while(true) {
            TSegment* finalized = finalizeSegment(seg, frame, valueSeq);
            if(finalized == seg){
                break;
            }
            seg = finalized;
        }
    } else {
        segments.pop_back();
    }
            
            
    for(std::size_t i=1; i < segments.size(); i++){
        TSegment& seg = segments[i];
                
        if(seg.isOver){
                    
            double ua, ub;
            if(seg.upper == uL){
                ua = 0.0;
                ub = 0.0;
            } else {
                double k = ((uL - seg.uc) / (seg.upper - seg.uc)) * edgeGradRatio;
                double u = seg.upper - seg.uc;
                double l = uL - seg.uc;
                ub = ((1.0 - k) * u - 3.0 * (u - l)) / (u * u);
                ua = (-2.0 * u * ub - 1.0 + k) / (3.0 * u * u);
            }
                    
            double la, lb;
            if(seg.lower == lL){
                la = 0.0;
                lb = 0.0;
            } else {
                double k = ((lL - seg.lc) / (seg.lower - seg.lc)) * edgeGradRatio;
                double u = seg.lower - seg.lc;
                double l = lL - seg.lc;
                lb = ((1.0 - k) * u - 3.0 * (u - l)) / (u * u);
                la = (-2.0 * u * lb - 1.0 + k) / (3.0 * u * u);
            }
                    
                    
            for(int frame=seg.begin; frame < seg.end; frame++){
                double v = valueSeq[frame];
                if(v >= seg.uc){
                    double x = v - seg.uc;
                    valueSeq[frame] = ua * x*x*x + ub * x*x + x + seg.uc;
                } else if(v <= seg.lc){
                    double x = v - seg.lc;
                    valueSeq[frame] = la * x*x*x + lb * x*x + x + seg.lc;
                }
            }
        }
    }
            
    return true;
}
}

#endif
