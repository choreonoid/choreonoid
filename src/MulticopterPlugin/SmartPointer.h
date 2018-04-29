/**
   @author Japan Atomic Energy Agency
*/

#pragma once

namespace Multicopter{

template<typename T> class PrivateDeleter
{
public:
  void operator()(T* ptr) { delete ptr; }
};

}












