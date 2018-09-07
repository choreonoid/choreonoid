#ifndef TEACHING_LOGGER_UTIL_H_INCLUDED
#define TEACHING_LOGGER_UTIL_H_INCLUDED

#include <iostream>
#include <string>
#include <stdarg.h>

using namespace std;

namespace cnoid {

enum LogLevel
{
    LOG_NO = 0,
    LOG_ERROR,
    LOG_DEBUG
};

class ILogger
{
public:
    ILogger() {};
    virtual ~ILogger() {};

    void startLog(const string dirName);

    virtual void writeLogError(const string contents) = 0;
    virtual void writeLogWithArgsError(char *contents) = 0;
    virtual void writeLogDebug(const string contents) = 0;
    virtual void writeLogWithArgsDebug(char *contents) = 0;

protected:
    string dirName_;
    string fileName_;

    virtual void writeLog(const string level, const string contents);
    virtual void writeLogWithArgs(const string level, char *contents);
};
/////
class LoggerNo : public ILogger
{
public:
    LoggerNo() {};
    virtual ~LoggerNo() {};

    void writeLogError(const string contents) {};
    void writeLogWithArgsError(char *contents) {};
    void writeLogDebug(const string contents) {};
    void writeLogWithArgsDebug(char *contents) {};
};

/////
class LoggerError : public LoggerNo
{
public:
    LoggerError() {};
    virtual ~LoggerError() {};

    void writeLogError(const string contents)
    {
        writeLog("ERROR", contents);
    };
    void writeLogWithArgsError(char *contents)
    {
        writeLogWithArgs("ERROR", contents);
    };
};
/////
class LoggerDebug : public LoggerError
{
public:
    LoggerDebug() {};
    virtual ~LoggerDebug() {};

    void writeLogDebug(const string contents)
    {
        writeLog("DEBUG", contents);
    };
    void writeLogWithArgsDebug(char *contents)
    {
        writeLogWithArgs("DEBUG", contents);
    };
};
//////////
class LoggerUtil
{
public:
    static void startLog(LogLevel logMode, const string dirName);

    static void logDebug(const string contents)
    {
        instance_->writeLogDebug(contents);
    };
    static void logDebugWithArgs(const char *format, ...)
    {
        va_list argp;
        va_start(argp, format);
        char allocatedBuffer[1024];
        int size = vsprintf(allocatedBuffer, format, argp);
        va_end(argp);

        instance_->writeLogWithArgsDebug(allocatedBuffer);
    };

    static void logError(const string contents)
    {
        instance_->writeLogError(contents);
    };
    static void logErrorWithArgs(const char *format, ...)
    {
        va_list argp;
        va_start(argp, format);
        char allocatedBuffer[1024];
        int size = vsprintf(allocatedBuffer, format, argp);
        va_end(argp);

        instance_->writeLogWithArgsError(allocatedBuffer);
    };

private:
    static ILogger* instance_;

    LoggerUtil();
    ~LoggerUtil();
};

//#define LOG_OUT

#ifdef LOG_OUT
#define DDEBUG(s) { \
LoggerUtil::logDebug(s); \
}
#define DDEBUG_V(...) { \
LoggerUtil::logDebugWithArgs(__VA_ARGS__); \
}
#define DERROR(s) { \
LoggerUtil::logError(s); \
}
#define DERROR_V(...) { \
LoggerUtil::logErrorWithArgs(__VA_ARGS__); \
}

#else
#define DDEBUG(s)
#define DDEBUG_V(...)
#define DERROR(s)
#define DERROR_V(...)
#endif

}
#endif
