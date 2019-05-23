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
    for(auto& connection : connectionsToBlock){
        connection.block();
    }

    if(emitFunction){
        emitFunction();
    } else {
        defaultEmitFunction();
    }
    
    for(auto& connection : connectionsToBlock){
        connection.unblock();
    }
    connectionsToBlock.clear();
    
    return false;
}
