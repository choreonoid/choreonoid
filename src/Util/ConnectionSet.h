/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_CONNECTION_SET_H
#define CNOID_BASE_CONNECTION_SET_H

#include "Signal.h"
#include <vector>

namespace cnoid {

class ConnectionSetBase
{
    std::vector<Connection> connections;
    
protected:
    ConnectionSetBase() { }
    ConnectionSetBase(const ConnectionSetBase& org)
        : connections(org.connections) { }
    ConnectionSetBase(ConnectionSetBase&& org)
        : connections(std::move(org.connections)) { }
    void clear() { connections.clear(); }

public:
    bool empty() const { return connections.empty(); }
    int numConnections() const { return connections.size(); }
    Connection& operator[](int index){ return connections[index]; };
    int add(const Connection& connection){
        auto index = connections.size();
        connections.push_back(connection);
        return index;
    }
    void add(const ConnectionSetBase& another){
        for(auto& connection : another.connections){
            connections.push_back(connection);
        }
    }
    void block(){
        for(auto& connection : connections){
            connection.block();
        }
    }
    void unblock(){
        for(auto& connection : connections){
            connection.unblock();
        }
    }
    void block(int index){
        connections[index].block();
    }
    void unblock(int index){
        connections[index].unblock();
    }
    void disconnect(){
        for(auto& connection : connections){
            connection.disconnect();
        }
        connections.clear();
    }
    
    class ScopedBlock {
        ConnectionSetBase* pConnections;
    public:
        ScopedBlock(ConnectionSetBase& connections)
            : pConnections(&connections) {
            connections.block();
        }
        ScopedBlock(ScopedBlock&& org) : pConnections(org.pConnections) {
            org.pConnections = nullptr;
        }
        ScopedBlock(const ScopedBlock&) = delete;
        ScopedBlock& operator=(const ScopedBlock&) = delete;
        ~ScopedBlock(){
            if(pConnections){
                pConnections->unblock();
            }
        }
    };
    ScopedBlock scopedBlock(){ return ScopedBlock(*this); }
};
    

class ConnectionSet : public ConnectionSetBase
{
public:
    ConnectionSet() { }
    ConnectionSet(const ConnectionSet& org) : ConnectionSetBase(org) { }
    ConnectionSet(ConnectionSet&& org) : ConnectionSetBase(std::move(org)) { }
    ConnectionSet& operator=(const ConnectionSet& org){
        clear(); add(org); return *this;
    }
};


class ScopedConnectionSet : public ConnectionSetBase
{
public:
    ScopedConnectionSet() { }
    ScopedConnectionSet(ScopedConnectionSet&& org) : ConnectionSetBase(std::move(org)) { }
    ~ScopedConnectionSet() { disconnect(); }

    ScopedConnectionSet(const ScopedConnectionSet&) = delete;
    ScopedConnectionSet& operator=(const ScopedConnectionSet&) = delete;
};

}
        
#endif
