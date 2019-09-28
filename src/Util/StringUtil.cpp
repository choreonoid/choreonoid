#include "StringUtil.h"
#include <algorithm> 
#include <cctype>
#include <locale>

namespace cnoid {

void trim(std::string& s)
{
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch){
        return !std::isspace(ch);
    }));
    s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch){
        return !std::isspace(ch);
    }).base(), s.end());
}

std::string trimmed(const std::string& s)
{
    auto left  = std::find_if_not(s.begin(), s.end(), [](int c){ return std::isspace(c); });
    auto right = std::find_if_not(s.rbegin(),s.rend(),[](int c){ return std::isspace(c);}).base();
    return (left >= right ? std::string() : std::string(left, right));
}

}
