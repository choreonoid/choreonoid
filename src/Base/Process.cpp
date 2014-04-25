/**
   @author Shin'ichiro Nakaoka
*/

#include "Process.h"

#ifdef Q_OS_UNIX
#include "unistd.h"
#endif

using namespace cnoid;

Process::Process(QObject* parent)
    : QProcess(parent)
{
    connect(this, SIGNAL(readyReadStandardOutput()),
            this, SLOT(onReadyReadStandardOutput()));
}


void Process::onReadyReadStandardOutput()
{
    sigReadyReadStandardOutput_();
}


void Process::start(const QString& program, const QStringList& arguments, OpenMode mode)
{
    QProcess::start(program, arguments, mode);
    
#ifdef Q_OS_UNIX
    setpgid(pid(), 0);
#endif
}


void Process::start(const QString& program, OpenMode mode)
{
    QProcess::start(program, mode);
    
#ifdef Q_OS_UNIX
    setpgid(pid(), 0);
#endif
}    
