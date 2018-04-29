/**
   @author Japan Atomic Energy Agency
*/

#pragma once

namespace Multicopter{

template<typename T> class Box{
public:
    Box(){
        null();
    }

    Box(const Eigen::Matrix<T,3,1>& min, const Eigen::Matrix<T,3,1>& max){
        _min = min;
        _max = max;
    }

    template<typename S> Box(const Box<S>& b) : _min(b.min()), _max(b.max()){}

    Eigen::Matrix<T,3,1> min() const { return _min; }

    Eigen::Matrix<T,3,1> max() const { return _max; }

    T	x() const { return _max.x() - _min.x(); }

    T	y() const { return _max.y() - _min.y(); }

    T	z() const { return _max.z() - _min.z(); }

    Eigen::Matrix<T,3,1> center() const {
        return Eigen::Matrix<T,3,1>((_min.x()+_max.x())/static_cast<T>(2.0), (_min.y()+_max.y())/static_cast<T>(2.0), (_min.z()+_max.z())/static_cast<T>(2.0));
    }

    void setMin(const Eigen::Matrix<T,3,1>& min){ _min = min; }

    void setMax(const Eigen::Matrix<T,3,1>& max){ _max = max; }

    bool contain(const Eigen::Matrix<T,3,1>& pos) const{
        if( _min.x() <= pos.x() && pos.x() <= _max.x() &&
            _min.y() <= pos.y() && pos.y() <= _max.y() &&
            _min.z() <= pos.z() && pos.z() <= _max.z() )
            return true;
        else
            return false;
    }

    void calcCircumscribingSphere(T& r, Eigen::Matrix<T,3,1>& cen) const{
        T x = x();
        T y = y();
        T z = z();
        r = sqrt(x*x+y*y+z*z)/static_cast<T>(2.0);
        cen = center();
    }

    bool isNull() const{
        return ((_min.x() >= _max.x()) || (_min.y() >= _max.y()) || (_min.z() >= _max.z()));
    }

    void null(){
        _min = Eigen::Matrix<T,3,1>(1.0,0,0);
        _max = Eigen::Matrix<T,3,1>(0.0,0,0);
    }

    Box& operator=(const Box& rhs){
        _min = rhs._min;
        _max = rhs._max;
        return *this;
    }

    Box& operator|=(const Box& b){
        _min.x() = _min.x() > b._min.x() ? b._min.x() : _min.x();
        _min.y() = _min.y() > b._min.y() ? b._min.y() : _min.y();
        _min.z() = _min.z() > b._min.z() ? b._min.z() : _min.z();
        _max.x() = _max.x() < b._max.x() ? b._max.x() : _max.x();
        _max.y() = _max.y() < b._max.y() ? b._max.y() : _max.y();
        _max.z() = _max.z() < b._max.z() ? b._max.z() : _max.z();
        return *this;
    }

    Box& operator|=(const Eigen::Matrix<T,3,1>& v){
        _min.x() = _min.x() > v.x ? v.x : _min.x();
        _min.y() = _min.y() > v.y ? v.y : _min.y();
        _min.z() = _min.z() > v.z ? v.z : _min.z();
        _max.x() = _max.x() < v.x ? v.x : _max.x();
        _max.y() = _max.y() < v.y ? v.y : _max.y();
        _max.z() = _max.z() < v.z ? v.z : _max.z();
        return *this;
    }

    friend Box operator|(const Box& a, const Box& b){
        return Box(
            Eigen::Matrix<T,3,1>(
                a._min.x() < b._min.x() ? a._min.x() : b._min.x(),
                a._min.y() < b._min.y() ? a._min.y() : b._min.y(),
                a._min.z() < b._min.z() ? a._min.z() : b._min.z()
            ),
            Eigen::Matrix<T,3,1>(
                a._max.x() > b._max.x() ? a._max.x() : b._max.x(),
                a._max.y() > b._max.y() ? a._max.y() : b._max.y(),
                a._max.z() > b._max.z() ? a._max.z() : b._max.z()
            )
        );
    }

    friend Box operator&(const Box& a, const Box& b){
        return Box(
            Eigen::Matrix<T,3,1>(
                a._min.x() > b._min.x() ? a._min.x() : b._min.x(),
                a._min.y() > b._min.y() ? a._min.y() : b._min.y(),
                a._min.z() > b._min.z() ? a._min.z() : b._min.z()
            ),
            Eigen::Matrix<T,3,1>(
                a._max.x() < b._max.x() ? a._max.x() : b._max.x(),
                a._max.y() < b._max.y() ? a._max.y() : b._max.y(),
                a._max.z() < b._max.z() ? a._max.z() : b._max.z()
            )
        );
    }

    friend bool operator==(const Box& a, const Box& b){
        return (a._min == b._min && a._max == b._max);
    }

    friend bool operator!=(const Box& a, const Box& b){
        return !(a == b);
    }

    static Box create(const std::vector<Eigen::Matrix<T,3,1>>& posAry){
        Box bnds;
        for(const auto it = begin(posAry) ; it != end(posAry) ; ++it){
            bnds._min.x() = ((*it).x<=bnds._min.x()) ? (*it).x : bnds._min.x();
            bnds._max.x() = ((*it).x>=bnds._max.x()) ? (*it).x : bnds._max.x();
            bnds._min.y() = ((*it).y<=bnds._min.y()) ? (*it).y : bnds._min.y();
            bnds._max.y() = ((*it).y>=bnds._max.y()) ? (*it).y : bnds._max.y();
            bnds._min.z() = ((*it).z<=bnds._min.z()) ? (*it).z : bnds._min.z();
            bnds._max.z() = ((*it).z>=bnds._max.z()) ? (*it).z : bnds._max.z();
        }
        return bnds;
    }

private:
    Eigen::Matrix<T,3,1> _min;
    Eigen::Matrix<T,3,1> _max;
};
using  Boxf = Box<float>;
using  Boxd = Box<double>;
}
