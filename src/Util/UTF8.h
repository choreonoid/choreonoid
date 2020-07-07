/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_UTF8_H
#define CNOID_UTIL_UTF8_H

#include <string>
#include "exportdecl.h"

namespace cnoid {
#ifdef _WIN32
CNOID_EXPORT const std::string toUTF8(const std::string& text);
CNOID_EXPORT const std::string fromUTF8(const std::string& text);
CNOID_EXPORT const std::string toUTF8(const char* text);
CNOID_EXPORT const std::string fromUTF8(const char* text);
#else
inline const std::string& toUTF8(const std::string& text) { return text; }
inline const std::string& fromUTF8(const std::string& text) { return text; }
inline const std::string toUTF8(const char* text) { return text; }
inline const std::string fromUTF8(const char* text) { return text; }
#endif
}

#endif
