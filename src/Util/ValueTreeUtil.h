/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_VALUE_TREE_UTIL_H_INCLUDED
#define CNOID_UTIL_VALUE_TREE_UTIL_H_INCLUDED

#include "ValueTree.h"

namespace cnoid {

template<class Container>
bool writeElements(Mapping& mapping, const std::string& key, const Container& elements, bool isFlowStyle = false)
{
    ListingPtr listing = new Listing();
    if(isFlowStyle){
        listing->setFlowStyle(true);
    }
    for(typename Container::const_iterator p = elements.begin(); p != elements.end(); ++p){
        listing->append(*p);
    }
    if(!listing->empty()){
        mapping.insert(key, listing);
        return true;
    }
    return false;
}

template<class Container>
bool readElements(const Mapping& mapping, const std::string& key, Container& elements)
{
    bool completed = false;
    const Listing& listing = *mapping.findListing(key);
    if(listing.isValid()){
        for(int i=0; i < listing.size(); ++i){
            elements.push_back(listing[i].to<typename Container::value_type>());
        }
    }
    return !elements.empty();
}
}

#endif
