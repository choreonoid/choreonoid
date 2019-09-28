// Replacement for 'strtod()' function in Visual C++
// This is neccessary because the implementation of VC++6.0 uses 'strlen()' in the function,
// so that it becomes too slow for a string buffer which has long length.

namespace cnoid {

#ifndef _MSC_VER

#include <cstdlib>
inline float strtof(const char* nptr, char** endptr){
    return std::strtof(nptr, endptr);
}
inline double strtod(const char* nptr, char** endptr){
    return std::strtod(nptr, endptr);
}

#else

template<typename T> T strtofloat(const char* nptr, char** endptr)    
{
    const char* org = nptr;
    bool valid = false;
    T value = 0.0;
    T sign = +1.0;

    if(*nptr == '+'){
        nptr++;
    } else if(*nptr == '-'){
        sign = -1.0;
        nptr++;
    }
    if(isdigit((unsigned char)*nptr)){
        valid = true;
        do {
            value = value * 10.0 + (*nptr - '0');
            nptr++;
        } while(isdigit((unsigned char)*nptr));
    }
    if(*nptr == '.'){
        //valid = false; // allow values which end with '.'. For example, "0."
        nptr++;
        if(isdigit((unsigned char)*nptr)){
            T small = 0.1;
            valid = true;
            do {
                value += small * (*nptr - '0');
                small *= 0.1;
                nptr++;
            } while(isdigit((unsigned char)*nptr));
        }
    }
    if(valid && (*nptr == 'e' || *nptr == 'E')){
        nptr++;
        valid = false;
        T psign = +1.0;
        if(*nptr == '+'){
            nptr++;
        } else if(*nptr == '-'){
            psign = -1.0;
            nptr++;
        }
        if(isdigit((unsigned char)*nptr)){
            valid = true;
            T p = 0.0;
            do {
                p = p * 10.0 + (*nptr - '0');
                nptr++;
            } while(isdigit((unsigned char)*nptr));
            value *= pow(10.0, (double)(psign * p));
        }
    }
    if(valid){
        *endptr = (char*)nptr;
    } else {
        *endptr = (char*)org;
    }
    return sign * value;
}
inline float strtof(const char* nptr, char** endptr) {
    return strtofloat<float>(nptr, endptr);
}
inline double strtod(const char* nptr, char** endptr) {
    return strtofloat<double>(nptr, endptr);
}
#endif

}
