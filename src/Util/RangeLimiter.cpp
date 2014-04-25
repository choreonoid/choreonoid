/*! @file
  @author Shin'ichiro Nakaoka
*/

#include "RangeLimiter.h"

using namespace cnoid;

void RangeLimiter::calcCrossPoint(RangeLimiter::TSegment* seg)
{
    seg->uc = (uL - r * seg->upper) / (1.0 - r);
    seg->lc = (lL - r * seg->lower) / (1.0 - r);
    
    if(seg->lc > seg->uc){
        seg->uc = seg->lc = (seg->upper * lL - seg->lower * uL) / (seg->upper + lL - uL - seg->lower);
    }
}


RangeLimiter::TSegment* RangeLimiter::mergeSegment(RangeLimiter::TSegment* seg, RangeLimiter::TSegment* prevSeg)
{
    if(seg->upper > prevSeg->upper){
        prevSeg->upper = seg->upper;
    }
    if(seg->lower < prevSeg->lower){
        prevSeg->lower = seg->lower;
    }
    
    calcCrossPoint(prevSeg);
    
    segments.pop_back();
    
    return &segments.back();
}

