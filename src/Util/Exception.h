/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_EXCEPTION_H
#define CNOID_UTIL_EXCEPTION_H

#include <boost/exception/all.hpp>
#include <exception>

namespace cnoid {

struct exception_base : virtual std::exception, virtual boost::exception { };

typedef boost::error_info<struct tag_error_info_message, std::string> error_info_message;
    
struct nonexistent_key_error : virtual exception_base { };

typedef boost::error_info<struct tag_error_info_key, std::string> error_info_key;

struct type_mismatch_error : virtual exception_base { };

struct file_read_error : virtual exception_base { };

struct empty_data_error : virtual exception_base { };

}

#endif

