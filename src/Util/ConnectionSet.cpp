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

#ifdef CNOID_USE_BOOST_SIGNALS
        GeneralConnection& c = connections[i];
        if(c.which() == 0){
            boost::get<Connection>(c).disconnect();
        } else {
            boost::get<boost::signals::connection>(c).disconnect();
        }
#else
        connections[i].disconnect();
#endif
    }
    connections.clear();
}
        

void ConnectionSet::add(const Connection& connection)
{
    connections.push_back(connection);
}


#ifdef CNOID_USE_BOOST_SIGNALS
void ConnectionSet::add(const boost::signals::connection& connection)
{
    connections.push_back(connection);
}
#endif
        

void ConnectionSet::add(const ConnectionSet& other)
{
    for(size_t i=0; i < other.connections.size(); ++i){
        connections.push_back(other.connections[i]);
    }
}
        

void ConnectionSet::block()
{
    for(size_t i=0; i < connections.size(); ++i){

#ifdef CNOID_USE_BOOST_SIGNALS
        GeneralConnection& c = connections[i];
        if(c.which() == 0){
            boost::get<Connection>(c).block();
        } else {
            boost::get<boost::signals::connection>(c).block();
        }
#else
        connections[i].block();
#endif
    }
}


void ConnectionSet::unblock()
{
    for(size_t i=0; i < connections.size(); ++i){

#ifdef CNOID_USE_BOOST_SIGNALS
        GeneralConnection& c = connections[i];
        if(c.which() == 0){
            boost::get<Connection>(c).unblock();
        } else {
            boost::get<boost::signals::connection>(c).unblock();
        }
#else
        connections[i].unblock();
    }
#endif
}


void ConnectionSet::block(int index)
{
#ifdef CNOID_USE_BOOST_SIGNALS
    GeneralConnection& c = connections[index];
    if(c.which() == 0){
        boost::get<Connection>(c).block();
    } else {
        boost::get<boost::signals::connection>(c).block();
    }
#else
    connections[index].block();
#endif
}
        

void ConnectionSet::unblock(int index)
{
#ifdef CNOID_USE_BOOST_SIGNALS
    GeneralConnection& c = connections[index];
    if(c.which() == 0){
        boost::get<Connection>(c).unblock();
    } else {
        boost::get<boost::signals::connection>(c).unblock();
    }
#else
    connections[index].unblock();
#endif
}
