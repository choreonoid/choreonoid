/**
   @author Shin'ichiro Nakaoka
*/

#include "LazySignal.h"

using namespace cnoid;


LazySignalBase::LazySignalBase()
    : LazyCaller(std::bind(&LazySignalBase::doEmit, this))
{

}


LazySignalBase::LazySignalBase(std::function<void()> emitFunction, int priority)
    : LazyCaller(std::bind(&LazySignalBase::doEmit, this), priority),
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
