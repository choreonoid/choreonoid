/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SELECTION_H_INCLUDED
#define CNOID_UTIL_SELECTION_H_INCLUDED

#include <vector>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Selection
{
public:
    explicit Selection(const char* domainname = 0)
        : domainname_(domainname) {
        selectedIndex_ = 0;
    }

    explicit Selection(size_t size, const char* domainname = 0)
        : symbols_(size),
        domainname_(domainname) {
            selectedIndex_ = 0;
        }
        
    Selection& operator<<(const std::string& label){
        symbols_.push_back(label);
        return *this;
    }

    int selectedIndex() const {
        return selectedIndex_;
    }

    int which() const {
        return selectedIndex_;
    }

    bool is(int index) const {
        return (index == selectedIndex_);
    }

    bool select(int index);
    bool select(const std::string& symbol);

    int size() const {
        return  symbols_.size();
    }
        
    void resize(int s) {
        symbols_.resize(s);
        if(selectedIndex_ >= s){
            selectedIndex_ = s - 1;
        }
    }

    int index(const std::string& symbol) const;

    const char* label(int index) const;
    const char* selectedLabel() const;

    void setSymbol(int index, const std::string& symbol){
        symbols_[index] = symbol;
    }

    std::string& symbol(int index) {
        return symbols_[index];
    }
        
    const std::string& symbol(int index) const {
        return symbols_[index];
    }
            
    const std::string& selectedSymbol() const {
        return symbols_[selectedIndex_];
    }

private:
    std::vector<std::string> symbols_;
    int selectedIndex_;
    const char* domainname_;
};
}

#endif
