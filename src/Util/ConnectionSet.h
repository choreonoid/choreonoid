/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_CONNECTION_SET_H
#define CNOID_BASE_CONNECTION_SET_H

//#define CNOID_USE_BOOST_SIGNALS

#include "Signal.h"
#include <vector>
#include <boost/variant.hpp>

#ifdef CNOID_USE_BOOST_SIGNALS
#include <boost/signals/connection.hpp>
#endif

#include "exportdecl.h"

namespace cnoid {
    
class CNOID_EXPORT ConnectionSet
{
public:
    ConnectionSet();
    ConnectionSet(const ConnectionSet& org);
    ConnectionSet& operator=(const ConnectionSet& org);
    ~ConnectionSet();

    bool empty() const {
        return connections.empty();
    }
        
    size_t numConnections() const { return connections.size(); }

#ifdef CNOID_USE_BOOST_SIGNALS
    void add(const boost::signals::connection& connection);
#endif
    void add(const Connection& connection);
    void add(const ConnectionSet& connections);
    void block();
    void block(int index);
    void unblock();
    void unblock(int index);
    void disconnect();
        
private:
#ifdef CNOID_USE_BOOST_SIGNALS
    typedef boost::variant<Connection, boost::signals::connection> GeneralConnection;
    std::vector<GeneralConnection> connections;
#else
    std::vector<Connection> connections;
#endif
};

}
        
#endif

