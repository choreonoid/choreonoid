/**
   @author Japan Atomic Energy Agency
*/

#pragma once

namespace Multicopter{

template<typename T> class Triangle3
{
public:

    Triangle3(){}

    Triangle3(const Eigen::Matrix<T, 3, 1>& p0, const Eigen::Matrix<T, 3, 1>& p1, const Eigen::Matrix<T, 3, 1>& p2){
        _pos[0] = p0;
        _pos[1] = p1;
        _pos[2] = p2;
    }

    Eigen::Matrix<T, 3, 1> operator[](int idx) const{
        return _pos[idx];
    }

    Eigen::Matrix<T, 3, 1>& operator[](int idx){
        return _pos[idx];
    }

	Eigen::Matrix<T, 3, 1> barycenter() const{
		return (_pos[0]+_pos[1]+_pos[2])/static_cast<T>(3.0);
	}

	Eigen::Matrix<T, 3, 1> normal() const{
        return (_pos[1]-_pos[0]).cross(_pos[2]-_pos[0]).normalized();
	}

    void multiMatrix(const Eigen::Transform<T, 3, Eigen::Affine>& mtx){
      _pos[0] = mtx*_pos[0];
      _pos[1] = mtx*_pos[1];
      _pos[2] = mtx*_pos[2];
    }

    const Eigen::Matrix<T, 3, 1>* data() const{
        return _pos;
    }

protected:
    Eigen::Matrix<T, 3, 1> _pos[3];
};
using Triangle3f = Triangle3<float> ;
using Triangle3d = Triangle3<double>;
}

