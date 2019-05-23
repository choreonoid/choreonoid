/**
   @author Shin'ichiro Nakaoka
*/

#include "LazySignal.h"

using namespace cnoid;


LazySignalBase::LazySignalBase(int priority)
    : LazyCaller([&](){ doEmit(); }, priority)
{

}


LazySignalBase::LazySignalBase(std::function<void()> emitFunction, int priority)
    : LazyCaller([&](){ doEmit(); }, priority),
      emitFunction(emitFunction)
{

}


void LazySignalBase::doEmit()
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
}
