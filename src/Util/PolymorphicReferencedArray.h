/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_POLYMORPHIC_REFERENCED_ARRAY_H
#define CNOID_UTIL_POLYMORPHIC_REFERENCED_ARRAY_H

#include "Referenced.h"
#include <vector>

namespace cnoid {

template <class BaseReferencedType = Referenced>
class PolymorphicReferencedArrayBase
{
public:
    virtual ~PolymorphicReferencedArrayBase() { }
    virtual bool try_push_back(BaseReferencedType* obj) = 0;
    virtual BaseReferencedType* get_element(size_t i) = 0;
    virtual const BaseReferencedType* get_element(size_t i) const = 0;        
    virtual size_t get_size() const = 0;
    virtual void clear_elements() = 0;
};
    
    
template <class ReferencedType, class BaseReferencedType = Referenced, class PointerType = ReferencedType*>
class PolymorphicReferencedArray : public PolymorphicReferencedArrayBase<BaseReferencedType>
{
    typedef std::vector<PointerType> Container;
    Container elements;

public:
    typedef PolymorphicReferencedArrayBase<BaseReferencedType> ArrayBase;
    typedef typename Container::value_type value_type;
    typedef typename Container::iterator iterator;
    typedef typename Container::const_iterator const_iterator;
    typedef typename Container::reference reference;
    typedef typename Container::const_reference const_reference;

    PolymorphicReferencedArray() { }

    template <class RhsReferencedType, class RhsPointerType>
    PolymorphicReferencedArray(
        const PolymorphicReferencedArray<RhsReferencedType, BaseReferencedType, RhsPointerType>& rhs){
        (*this) << rhs;
    }
        
    virtual ~PolymorphicReferencedArray() { }

    virtual bool try_push_back(BaseReferencedType* obj) {
        if(ReferencedType* p = dynamic_cast<ReferencedType*>(obj)){
            push_back(p);
            return true;
        }
        return false;
    }

    bool operator==(const PolymorphicReferencedArray& rhs) const {
        if(size() == rhs.size()){
            return std::equal(begin(), end(), rhs.begin());
        }
        return false;
    }

    bool operator!=(const PolymorphicReferencedArray& rhs) const {
        return !operator==(rhs);
    }

    template <class RhsReferencedType, class RhsPointerType>
    PolymorphicReferencedArray& assign(
        const PolymorphicReferencedArray<RhsReferencedType, BaseReferencedType, RhsPointerType>& another){
        clear();
        (*this) << another;
        return *this;
    }

    template <class RetReferencedType, class RetPointerType>
    PolymorphicReferencedArray& operator<<(const PolymorphicReferencedArray<RetReferencedType, BaseReferencedType, RetPointerType>& rhs) {
        for(std::size_t i=0; i < rhs.size(); ++i){
            try_push_back(rhs[i]);
        }
        return *this;
    }
    
    template <class RetReferencedType, class RetPointerType>
    bool extractFrom(PolymorphicReferencedArray<RetReferencedType, BaseReferencedType, RetPointerType>& another) {
        size_t orgSize = size();
        typedef PolymorphicReferencedArray<RetReferencedType, BaseReferencedType, RetPointerType> ArgType;
        typename ArgType::iterator p = another.begin();
        while(p != another.end()){
            if(ReferencedType* element = dynamic_cast<ReferencedType*>(p->get())){
                push_back(element);
                p = another.erase(p);
            } else{
                ++p;
            }
        }
        return (size() > orgSize);
    }

    template <class RetReferencedType> PolymorphicReferencedArray extract() {
        PolymorphicReferencedArray extracted;
        iterator p = begin();
        while(p != end()){
            if(RetReferencedType* element = dynamic_cast<RetReferencedType*>(p->get())){
                extracted.push_back(element);
                p = erase(p);
            } else{
                ++p;
            }
        }
        return extracted;
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

    virtual size_t get_size() const {
        return size();
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

    virtual BaseReferencedType* get_element(size_t i) {
        return elements[i];
    }

    virtual const BaseReferencedType* get_element(size_t i) const {
        return elements[i];
    }
        
    void clear(){
        elements.clear();
    }

    virtual void clear_elements() {
        clear();
    }

    void push_back(const PointerType& obj){
        elements.push_back(obj);
    }

    void pop_back(){
        elements.pop_back();
    }

    iterator erase(iterator pos){
        return elements.erase(pos);
    }
};

}

#endif
