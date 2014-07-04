/**
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_BASE_PROCESS_H
#define CNOID_BASE_PROCESS_H

#include <cnoid/Signal>
#include <QProcess>
#include "exportdecl.h"

namespace cnoid {

/**
   @note On Linux, a child process inherits the same process group as the parent.
   In this case, some signals such as SIGINT sent to the parent process are also sent to
   the child processes.
   (see "man setpgid" and http://stackoverflow.com/questions/6803395/child-process-receives-parents-sigint.)
   This behavior is not good when the choreonoid main process wants to manage its child processes
   when the main process is terminated by Ctrl+C.
   For a child process invoked by the start methods of this class, its own process group id
   is given and the signals sent to the main process are not sent to it.
*/
class CNOID_EXPORT Process : public QProcess
{
    Q_OBJECT

public:
    Process(QObject* parent = 0);
                               
    SignalProxy<void()> sigReadyReadStandardOutput() {
        return sigReadyReadStandardOutput_;
    }

    void start(const QString& program, const QStringList& arguments, OpenMode mode = ReadWrite);
    void start(const QString& program, OpenMode mode = ReadWrite);

private Q_SLOTS:
    void onReadyReadStandardOutput();

private:
    Signal<void()> sigReadyReadStandardOutput_;
};

}

#endif
