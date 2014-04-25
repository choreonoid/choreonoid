/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_UNIFORM_CUBIC_BSPLINE_H_INCLUDED
#define CNOID_UTIL_UNIFORM_CUBIC_BSPLINE_H_INCLUDED

#include <Eigen/Core>

namespace cnoid {

template <class ElementType>
class UniformCubicBSplineBase
{
    int numSamples;
    double dtinv;

protected:
    Eigen::Matrix<ElementType, 4, 4> M;
        
public:
    UniformCubicBSplineBase(int numSamples, double dtinv)
        : numSamples(numSamples),
          dtinv(dtinv) {
        M <<
            -1.0,  3.0, -3.0, 1.0,
            3.0, -6.0,  3.0, 0.0,
            -3.0,  0.0,  3.0, 0.0,
            1.0,  4.0,  1.0, 0.0;
    }

    void findKnotPosition(ElementType t, int* out_knot, Eigen::Matrix<ElementType, 1, 4>& out_u) {

        out_knot[1] = t * dtinv;
        out_knot[0] = out_knot[1] - 1;
        out_knot[2] = out_knot[1] + 1;
        out_knot[3] = out_knot[1] + 2;

        if(out_knot[0] < 0){
            out_knot[0] = 0;
            if(out_knot[1] < 0){
                out_knot[1] = 0;
                t = 0.0;
            }
                
        } else if(out_knot[3] >= numSamples){
            out_knot[3] = numSamples - 1;
            if(out_knot[2] >= numSamples){
                out_knot[2] = numSamples - 1;
                if(out_knot[1] >= numSamples){
                    out_knot[1] = numSamples - 1;
                    t = 1.0;
                }
            }
        }
        t = fmod(t * dtinv, 1.0);

        ElementType t2 = t * t;
        out_u << (t2 * t), t2, t, 1.0;
    }
};


template <class ElementType>
class UniformCubicBSpline : public UniformCubicBSplineBase<ElementType>
{
    typedef UniformCubicBSplineBase<ElementType> SuperClass;
    ElementType* x;

public:
    UniformCubicBSpline(ElementType* x, int numSamples, double dtinv)
        : UniformCubicBSplineBase<ElementType>(numSamples, dtinv) {
    }

    double calc(double t) {

        int knot[4];
        Eigen::Matrix<ElementType, 1, 4> u;
        SuperClass::findKnotPosition(t, knot, u);

        Eigen::Matrix<ElementType, 4, 1> p;
        p << x[knot[0]], x[knot[1]], x[knot[2]], x[knot[3]];

        return (u.dot(UniformCubicBSplineBase<ElementType>::M * p)) / 6.0;
    }
};


template <class ContainerType, class VectorType, class ElementType>
class UniformCubicBSplineVector : public UniformCubicBSplineBase<ElementType>
{
    typedef UniformCubicBSplineBase<ElementType> SuperClass;
    const ContainerType& container;
    int vectorSize;
        
public:

    typedef Eigen::Matrix<ElementType, Eigen::Dynamic, 1> SolutionType;

    UniformCubicBSplineVector(const ContainerType& container, int numSamples, int vectorSize, double dtinv)
        : UniformCubicBSplineBase<ElementType>(numSamples, dtinv),
          container(container),
          vectorSize(vectorSize) {
    }

    void calc(double t, SolutionType& out_solution) {

        int knot[4];
        Eigen::Matrix<ElementType, 1, 4> u;
        SuperClass::findKnotPosition(t, knot, u);

        typedef Eigen::Matrix<ElementType, 4, Eigen::Dynamic> ValueMatrix;
        ValueMatrix P(4, vectorSize);

        for(int i=0; i < 4; ++i){
            VectorType value = container[knot[i]];
            typename ValueMatrix::RowXpr p = P.row(i);
            for(int j=0; j < vectorSize; ++j){
                p(j) = value[j];
            }
        }

        out_solution = u * (UniformCubicBSplineBase<ElementType>::M * P) / 6.0;
    }
};
}

#endif
