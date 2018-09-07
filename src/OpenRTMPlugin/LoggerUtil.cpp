#include "LoggerUtil.h"
#include <cnoid/MessageView>  /* modified by qtconv.rb 0th rule*/  
#include <fstream>

#include <QDir>
#include <QDateTime>

namespace cnoid {

void ILogger::startLog(const string dirName)
{
    dirName_ = dirName;
    QDir dir(QString::fromStdString(dirName_));
    if (dir.exists() == false) {
        QDir().mkdir(QString::fromStdString(dirName_));
    }

    QDateTime dt = QDateTime::currentDateTime();
    QString str = dt.toString("yyyyMMddHHmmss");
    fileName_ = str.toStdString();
}

void ILogger::writeLog(const string level, const string contents)
{
    QDateTime dt = QDateTime::currentDateTime();
    QString str = dt.toString("yyyy/MM/dd HH:mm:ss zzz");

    ofstream logs;

    string fullPath = dirName_ + "/" + fileName_ + ".log";
    logs.open(fullPath.c_str(), ios::out | ios::app);
    logs << level << ":" << str.toStdString() << ":" << contents << "|" << endl;
    logs.close();

    //MessageView::mainInstance()->cout() << level << ":" << str.toStdString() << ":" << contents << endl;
}

void ILogger::writeLogWithArgs(const string level, char *contents)
{
    string strCont = string(contents);
    writeLog(level, strCont);
};
/////
ILogger* LoggerUtil::instance_ = NULL;

LoggerUtil::LoggerUtil() {}

LoggerUtil::~LoggerUtil()
{
    if (instance_ != NULL) {
        delete instance_;
        instance_ = NULL;
    }
}

void LoggerUtil::startLog(LogLevel logMode, const string dirName)
{
    if (instance_ != NULL) delete instance_;

    switch (logMode) {
        case LOG_ERROR:
            instance_ = new LoggerError();
            break;
        case LOG_DEBUG:
            instance_ = new LoggerDebug();
            break;
        default:
            instance_ = new LoggerNo();
            break;
    }
    instance_->startLog(dirName);
}

}