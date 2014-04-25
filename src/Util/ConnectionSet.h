/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_CONNECTION_SET_H_INCLUDED
#define CNOID_GUIBASE_CONNECTION_SET_H_INCLUDED

#include <vector>
#include <boost/signals/connection.hpp>
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

    void add(const boost::signals::connection& connection);
    void add(const ConnectionSet& connections);
    void block();
    void block(int index) { connections[index].block(); }
    void unblock();
    void unblock(int index) { connections[index].unblock(); }
    void disconnect();
        
private:
    std::vector<boost::signals::connection> connections;
};
}
        
#endif

