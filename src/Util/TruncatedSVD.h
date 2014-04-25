/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_TRUNCATED_SVD_SOLVER_H_INCLUDED
#define CNOID_UTIL_TRUNCATED_SVD_SOLVER_H_INCLUDED

#include <Eigen/Core>

namespace cnoid {

template <class MatrixType>
class TruncatedSVD
{
    typedef typename MatrixType::Scalar Scalar;
    Eigen::JacobiSVD<MatrixType> svd;
    Eigen::DiagonalMatrix<Scalar, Eigen::Dynamic> sinv;
    Scalar truncateRatio;
    int numTruncated;

public:

    static double defaultTruncateRatio() { return 2.0e2; }

    TruncatedSVD() {
        truncateRatio = defaultTruncateRatio();
        numTruncated = 0;
    }

    void setTruncateRatio(Scalar r){
        if(r <= 0.0){
            truncateRatio = std::numeric_limits<double>::infinity();
        } else {
            truncateRatio = r;
        }
    }

    TruncatedSVD& compute(const MatrixType& matrix){

        numTruncated = 0;
            
        svd.compute(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
        sinv.diagonal() = svd.singularValues();

        int lastNonZeroSingularValues = svd.nonzeroSingularValues() - 1;
        if(lastNonZeroSingularValues > 0){
            Scalar& s0 = sinv.diagonal()(0);
            Scalar& s_last = sinv.diagonal()(lastNonZeroSingularValues);
            if((s0 / s_last) > truncateRatio){
                s_last = Scalar(0.0);
                ++numTruncated;
                int j;
                for(j = lastNonZeroSingularValues - 1; j > 0; --j){
                    Scalar& s = sinv.diagonal()(j);
                    if((s0 / s) > truncateRatio){
                        s = Scalar(0.0);
                        ++numTruncated;
                    } else {
                        break;
                    }
                }
                while(j >= 0){
                    Scalar& s = sinv.diagonal()(j);
                    s = Scalar(1.0) / s;
                    --j;
                }
            }
        }
        return *this;
    }

    int numTruncatedSingularValues() const {
        return numTruncated;
    }

    const typename Eigen::JacobiSVD<MatrixType>::SingularValuesType& singularValues() const {
        return svd.singularValues();
    }
        
    template <class VectorType1, class VectorType2>
    void solve(const Eigen::MatrixBase<VectorType1>& b, Eigen::MatrixBase<VectorType2>& out_x) const {
        if(numTruncated > 0){
            out_x = svd.matrixV() * sinv * svd.matrixU().transpose() * b;
        } else {
            out_x = svd.solve(b);
        }
    }
};
}

#endif
