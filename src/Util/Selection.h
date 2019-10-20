/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SELECTION_H
#define CNOID_UTIL_SELECTION_H

#include <vector>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Selection
{
public:
    explicit Selection(const char* domainname = 0);
    explicit Selection(size_t size, const char* domainname = 0);
    Selection(const Selection& org);
        
    int size() const {
        return  static_cast<int>(symbols_.size());
    }

    explicit operator bool() const {
        return selectedIndex_ >= 0;
    }

    operator int() const {
        return selectedIndex_;
    }
        
    void resize(int s);

    void clear();

    void setSymbol(int index, const std::string& symbol);

    Selection& operator<<(const std::string& symbol);

    std::string& symbol(int index) {
        return symbols_[index];
    }
        
    const std::string& symbol(int index) const {
        return symbols_[index];
    }
            
    int index(const std::string& symbol) const;

    const char* label(int index) const;

    bool select(int index);
    bool selectIndex(int index);
    bool select(const std::string& symbol);

    int selectedIndex() const {
        return selectedIndex_;
    }

    int which() const {
        return selectedIndex_;
    }

    bool is(int index) const {
        return (index == selectedIndex_);
    }

    const char* selectedSymbol() const;
    const char* selectedLabel() const;

private:
    std::vector<std::string> symbols_;
    int selectedIndex_;
    const char* domainname_;
};

}

#endif
