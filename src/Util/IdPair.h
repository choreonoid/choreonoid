/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_ID_PAIR_H
#define CNOID_UTIL_ID_PAIR_H

#include <functional>
#include <cstdint>

namespace cnoid {

template<class T = int> class IdPair
{
protected:
    T id[2];

public:
    IdPair(){ }

    IdPair(T id0, T id1){
        set(id0, id1);
    }

    IdPair(const T* src){
        if(src[0] <= src[1]){
            id[0] = src[0];
            id[1] = src[1];
        } else {
            id[0] = src[1];
            id[1] = src[0];
        }
    }

    void set(T id0, T id1){
        if(id0 <= id1){
            id[0] = id0;
            id[1] = id1;
        } else {
            id[0] = id1;
            id[1] = id0;
        }
    }        

    T operator()(int which) const { return id[which]; }

    T operator[](int which) const { return id[which]; }
    T& operator[](int which) { return id[which]; }

    bool operator==(const IdPair& pair2) const {
        return (id[0] == pair2.id[0] && id[1] == pair2.id[1]);
    }
    
    bool operator<(const IdPair& pair2) const {
        if(id[0] < pair2.id[0]){
            return true;
        } else if(id[0] == pair2.id[0]){
            return (id[1] < pair2.id[1]);
        } else {
            return false;
        }
    }
};

}

namespace std {

template<class T> struct hash<cnoid::IdPair<T>>
{
    void hash_combine(std::size_t& seed, const T& v) const {
        std::hash<T> hasher;
        seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
    }
    std::size_t operator()(const cnoid::IdPair<T>& idPair) const {
        
        std::size_t seed = 0;
        hash_combine(seed, idPair(0));
        hash_combine(seed, idPair(1));
        return seed;
    }
};

template<> struct hash<cnoid::IdPair<int32_t>>
{
    std::size_t operator()(const cnoid::IdPair<int32_t>& idPair) const{
        return hash<int64_t>()(static_cast<int64_t>(idPair(0)) | static_cast<int64_t>(idPair(1)) << 32);
    }
};

}
    
#endif
