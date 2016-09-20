/**
   @author Shin'ichiro Nakaoka
*/

#include "ConnectionSet.h"

using namespace cnoid;


ConnectionSet::ConnectionSet()
{

}


ConnectionSet::ConnectionSet(const ConnectionSet& org)
{
    add(org);
}


/**
   This operator disconnects existing connections.
*/
ConnectionSet& ConnectionSet::operator=(const ConnectionSet& org)
{
    disconnect();
    add(org);
    return *this;
}
        

/**
   Destructor.
   Note that the connections are *not* disconnected by the destructor.
   This design is employed to  allow a use of the copy constructor and copy operator.
*/
ConnectionSet::~ConnectionSet()
{

}


void ConnectionSet::disconnect()
{
    for(size_t i=0; i < connections.size(); ++i){
        connections[i].disconnect();
    }
    connections.clear();
}
        

void ConnectionSet::add(const Connection& connection)
{
    connections.push_back(connection);
}


void ConnectionSet::add(const ConnectionSet& other)
{
    for(size_t i=0; i < other.connections.size(); ++i){
        connections.push_back(other.connections[i]);
    }
}
        

void ConnectionSet::block()
{
    for(size_t i=0; i < connections.size(); ++i){
        connections[i].block();
    }
}


void ConnectionSet::unblock()
{
    for(size_t i=0; i < connections.size(); ++i){
        connections[i].unblock();
    }
}


void ConnectionSet::block(int index)
{
    connections[index].block();
}
        

void ConnectionSet::unblock(int index)
{
    connections[index].unblock();
}
