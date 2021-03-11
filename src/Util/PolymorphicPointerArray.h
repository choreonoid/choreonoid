/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_POLYMORPHIC_POINTER_ARRAY_H
#define CNOID_UTIL_POLYMORPHIC_POINTER_ARRAY_H

#include "Referenced.h"
#include <vector>

namespace cnoid {

class PolymorphicPointerArrayBase
{
public:
    virtual ~PolymorphicPointerArrayBase() { }
};


template<class ObjectType, class PointerType>
class PolymorphicPointerArray : public PolymorphicPointerArrayBase
{
    typedef std::vector<PointerType> Container;
    Container elements;

public:
    typedef PolymorphicPointerArrayBase Base;
    typedef typename Container::iterator iterator;
    typedef typename Container::const_iterator const_iterator;
    typedef typename Container::reference reference;
    typedef typename Container::const_reference const_reference;

    PolymorphicPointerArray() { }

    template <class RhsObjectType, class RhsPointerType>
    PolymorphicPointerArray(const PolymorphicPointerArray<RhsObjectType, RhsPointerType>& rhs){
        (*this) << rhs;
    }
        
    virtual ~PolymorphicPointerArray() { }

    template <class RhsObjectType, class RhsPointerType>
    PolymorphicPointerArray& operator<<(const PolymorphicPointerArray<RhsObjectType, RhsPointerType>& rhs){
        for(std::size_t i=0; i < rhs.size(); ++i){
            PointerType p = cnoid::dynamic_pointer_cast<ObjectType>(rhs[i]);
            if(p){
                push_back(p);
            }
        }
        return *this;
    }

    bool operator==(const PolymorphicPointerArray& rhs) const {
        return elements == rhs.elements;
    }

    bool operator!=(const PolymorphicPointerArray& rhs) const {
        return elements != rhs.elements;
    }
        
    bool empty() const {
        return elements.empty();
    }

    void reserve(size_t size) {
        elements.reserve(size);
    }

    void resize(size_t size) {
        elements.resize(size);
    }
        
    std::size_t size() const { 
        return elements.size();
    }

    iterator begin() {
        return elements.begin();
    }

    const_iterator begin() const {
        return elements.begin();
    }

    iterator end() {
        return elements.end();
    }

    const_iterator end() const {
        return elements.end();
    }
        
    PointerType& back() {
        return elements.back();
    }

    const PointerType& back() const {
        return elements.back();
    }
        
    PointerType& front() {
        return elements.front();
    }

    const PointerType& front() const {
        return elements.front();
    }
        
    PointerType& operator[](std::size_t i) {
        return elements[i];
    }

    const PointerType& operator[](std::size_t i) const {
        return elements[i];
    }
        
    void clear(){
        elements.clear();
    }

    void push_back(const PointerType& pointer){
        elements.push_back(pointer);
    }

    void pop_back(){
        elements.pop_back();
    }

    iterator erase(iterator pos){
        return elements.erase(pos);
    }

    void swap(PolymorphicPointerArray& x){
        elements.swap(x.elements);
    }
};

}

#endif
