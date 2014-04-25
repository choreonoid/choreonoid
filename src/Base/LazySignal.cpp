/**
   @author Shin'ichiro Nakaoka
*/

#include "LazySignal.h"
#include <boost/bind.hpp>

using namespace boost;
using namespace cnoid;


LazySignalBase::LazySignalBase()
    : LazyCaller(bind(&LazySignalBase::doEmit, this))
{

}


LazySignalBase::LazySignalBase(boost::function<void()> emitFunction, int priority)
    : LazyCaller(bind(&LazySignalBase::doEmit, this), priority),
      emitFunction(emitFunction)
{

}


void LazySignalBase::request()
{
    (*this)();
}


bool LazySignalBase::doEmit()
{
    for(size_t i=0; i < connectionsToBlock.size(); ++i){
        connectionsToBlock[i].block();
    }

    if(emitFunction){
        emitFunction();
    } else {
        defaultEmitFunction();
    }
    
    for(size_t i=0; i < connectionsToBlock.size(); ++i){
        connectionsToBlock[i].unblock();
    }
    connectionsToBlock.clear();
    
    return false;
}
