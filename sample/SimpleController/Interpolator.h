/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_INTERPOLATOR_H_INCLUDED
#define CNOID_UTIL_INTERPOLATOR_H_INCLUDED

#include <vector>
#include <limits>
#include <cmath>

namespace cnoid {

template<class VectorType> class Interpolator
{
    enum SegmentType { UNDETERMINED, CUBIC_SPLINE, POLYNOMINAL };

    struct Sample
    {
        double x;
        VectorType y;
        VectorType yp;
                
        // coefficients
        VectorType a;
        VectorType a_end;
        VectorType b;
        VectorType c;

        bool isEdge;
        bool isNaturalEdge;
        SegmentType segmentType;

        Sample(int size)
            : y(size),
              yp(size),
              a(size),
              a_end(size),
              b(size),
              c(size) {

        }
    };

    std::vector<Sample> samples;
    mutable int prevReferredSegments;

public:
    void clear() {
        samples.clear();
        prevReferredSegments = -1;
    }

    int numSamples() const {
        return samples.size();
    }

    double domainLower() const {
        return samples.empty() ? 0.0 : samples.front().x;
    }

    double domainUpper() const {
        return samples.empty() ? 0.0 : samples.back().x;
    }
        
    int appendSample(double x, const VectorType& y) {
        if(!samples.empty()){
            Sample& prev = samples.back();
            if(fabs(prev.x - x) < std::numeric_limits<double>::epsilon()){
                samples.pop_back();
            }
        }

        int index = samples.size();
        int size = y.size();
        samples.push_back(Sample(size));

        Sample& s = samples.back();
        s.x = x;
        s.y = y;
        s.yp = VectorType::Zero(size);
        s.isEdge = false;
        s.isNaturalEdge = false;
        s.segmentType = UNDETERMINED;

        return index;
    }

    void setEndPoint(int sampleIndex, bool isNatural = false) {
        Sample& sample = samples[sampleIndex];
        sample.isEdge = true;
        sample.isNaturalEdge = isNatural;
        sample.yp = VectorType::Zero(sample.yp.size()); // first-order derivative (velocity)
    }

    bool update() {
        int n = samples.size();
        int s = 0;
        while(true){
            const int m = n - s;
            if(m < 2){
                break;
            }
            if(m >= 3 && !samples[s + 1].isEdge){
                s = updateCubicSplineSegment(s);
            } else {
                s = updateSegmentBetweenTwoSamples(s);
            }
        }
        return (n >= 2);
    }

    VectorType interpolate(double x) const {
        int lower;
        int upper;
    
        const int n = samples.size();
        int k;
        if(prevReferredSegments >= 0 && prevReferredSegments < (n - 1)){
            k = prevReferredSegments;
            if(x >= samples[k].x && x < samples[k+1].x){
                goto calc;
            }
        }
            
        lower = 0;
        upper = n - 1;
            
        if(x < samples[0].x){
            return samples[0].y;
        } else if(x >= samples[upper].x){
            return samples[upper].y;
        }
            
        while(upper - lower > 1){
            k = (upper + lower) / 2;
            if(samples[k].x > x){
                upper = k;
            } else {
                lower = k;
            }
        }
            
        k = lower;
            
calc:
            
        prevReferredSegments = k;
        const Sample& s0 = samples[k];
        const Sample& s1 = samples[k+1];
            
        if(s0.segmentType == CUBIC_SPLINE){
            const VectorType& a_end = (s1.isEdge ? s1.a_end : s1.a);
            const double h = s1.x - s0.x;
            const double A = (s1.x - x) / h;
            const double B = (x - s0.x) / h;
            return A * s0.y + B * s1.y + ((A*A*A - A) * s0.a + (B*B*B - B) * a_end) * (h*h) / 6.0;
                
        } else if(s0.segmentType == POLYNOMINAL){
            const VectorType& a0 = s0.y;
            const VectorType& a1 = s0.a;
            const VectorType& a2 = s0.b;
            const VectorType& a3 = s0.c;
            const double h = x - s0.x;
            const double h2 = h * h;
            const double h3 = h2 * h;
            return (a0 + a1 * h + a2 * h2 + a3 * h3);
        }
            
        return VectorType();
    }
            
private:

    int updateCubicSplineSegment(int begin) {

        Sample& s0 = samples[begin];
        s0.segmentType = CUBIC_SPLINE;
        s0.isEdge = true;

        const int size = s0.y.size();

        if(s0.isNaturalEdge){
            s0.a = VectorType::Zero(size);
            s0.b = VectorType::Zero(size);
        } else {
            Sample& s1 = samples[begin + 1];
            s0.a = VectorType::Constant(size, -0.5);
            s0.b = (3.0 / (s1.x - s0.x)) * ((s1.y - s0.y) / (s1.x - s0.x) - s0.yp);
        }
            
        const int n = samples.size();
        int i = (begin + 1);
        while(true) {
            Sample& s0 = samples[i-1];
            Sample& s1 = samples[i];
            Sample& s2 = samples[i+1];
                
            s1.segmentType = CUBIC_SPLINE;

            const VectorType b = (s2.y - s1.y) / (s2.x - s1.x) - (s1.y - s0.y) / (s1.x - s0.x);
            const double sig = (s1.x - s0.x) / (s2.x - s0.x);
            for(int j=0; j < size; ++j){
                const double p = sig * s0.a[j] + 2.0;
                s1.a[j] = (sig - 1.0) / p;
                s1.b[j] = (6.0 * b[j] / (s2.x - s0.x) - sig * s0.b[j]) / p;
            }
                
            if(s2.isEdge || i == (n - 2)){
                break;
            }
                
            ++i;
        }

        double qf;
        VectorType bf;
        Sample& sf0 = samples[i];
        Sample& sf = samples[i+1];
        const int next = i + 1;
            
        sf.isEdge = true;
            
        if(sf.isNaturalEdge){
            qf = 0.0;
            bf = VectorType::Zero(size);
        } else {
            qf = 0.5;
            bf = (3.0 / (sf.x - sf0.x)) * (sf.yp - (sf.y - sf0.y) / (sf.x - sf0.x));
        }

        const VectorType a_save = sf.a;

        for(int i=0; i < sf0.a.size(); ++i){
            sf.a[i] = (bf[i] - qf * sf0.b[i]) / (qf * sf0.a[i] + 1.0);
        }

        while(i >= begin){
            Sample& s0 = samples[i];
            Sample& s1 = samples[i+1];
            s0.a = s0.a.cwiseProduct(s1.a) + s0.b;
            --i;
        }
            
        sf.a_end = sf.a;
        sf.a = a_save;
            
        return next;
    }
            
    int updateSegmentBetweenTwoSamples(int begin) {
        Sample& s0 = samples[begin];
        s0.segmentType = POLYNOMINAL;
        s0.isEdge = true;
        Sample& s1 = samples[begin+1];
        s1.isEdge = true;
            
        const double h = (s1.x - s0.x);
        const double h2 = h * h;
        const double h3 = h2 * h;
            
        const int size = s0.yp.size();
        const VectorType d0 = s0.isEdge ? s0.yp : VectorType::Zero(size);
        const VectorType d1 = s1.isEdge ? s1.yp : VectorType::Zero(size);
            
        s0.a = d0;
        s0.b = 3.0 * (s1.y - s0.y) / h2 - (2.0 * d0 - d1) / h;
        s0.c = (d0 + d1) / h2 + 2.0 * (s0.y - s1.y) / h3;
            
        return begin + 1;
    }
};

}

#endif
