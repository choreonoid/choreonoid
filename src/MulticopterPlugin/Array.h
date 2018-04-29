/**
   @author Japan Atomic Energy Agency
*/

#pragma once

namespace Multicopter {

template<class T> class Array2{
public:   
	Array2(){
		_x = _y = 0;
	}

	bool create(unsigned int x, unsigned int y){
		if( x == 0 || y == 0 ){
			return false;
		}
		_ary.resize(x*y);
		_x = x;
		_y = y;
		return true;
	}

	T operator()(unsigned int x, unsigned int y) const{
		return _ary[_x*y+x];
	}

	T& operator()(unsigned int x, unsigned int y){
		return _ary[_x*y+x];
	}
    
	void clear(){
		_x = _y = 0;
		_ary.clear();
	}

    unsigned int xSize() const{
        return _x;
    }

    unsigned int ySize() const{
        return _y;
    }

	const T* data() const{
		return ( _ary.empty() == true ) ? nullptr : &(_ary[0]);
	}
private:
    unsigned int _x;
    unsigned int _y;
    std::vector<T> _ary;
};

template<class T> class Array3{
public:   
	Array3(){
		_x = _y = _z = 0;
	}

	bool create(unsigned int x, unsigned int y, unsigned int z){
		if( x == 0 || y == 0 || z == 0 ){
			return false;
		}
		_ary.resize(x*y*z);
		_x = x;
		_y = y;
		_z = z;
		return true;
	}

	void clear(){
		_x = _y = _z = 0;
		_ary.clear();
	}

	T operator()(unsigned int x, unsigned int y, unsigned int z) const{
		return _ary[_y*_x*z+_x*y+x];
	}

	T& operator()(unsigned int x, unsigned int y, unsigned int z){
		return _ary[_y*_x*z+_x*y+x];
	}

    unsigned int xSize() const{
        return _x;
    }

    unsigned int ySize() const{
        return _y;
    }

    unsigned int zSize() const{
        return _z;
    }

	const T* data() const{
		return ( _ary.empty() == true ) ? nullptr : &(_ary[0]);
	}
private:
    unsigned int _x;
    unsigned int _y;
    unsigned int _z;
    std::vector<T> _ary;
};
}
