/**
   @author Shin'ichiro Nakaoka
*/

#include "Referenced.h"

using namespace cnoid;

Referenced::~Referenced()
{
    WeakCounter* wc = weakCounter_.load(std::memory_order_acquire);
    if(wc){
        wc->isObjectAlive_ = false;
        wc->release();
    }
}
