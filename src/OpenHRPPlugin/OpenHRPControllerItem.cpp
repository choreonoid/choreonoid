/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "OpenHRPControllerItem.h"
#include "DynamicsSimulator_impl.h"
#include <cnoid/ControllerIO>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <cnoid/Sleep>
#include <boost/filesystem.hpp>
#include <fmt/format.h>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;
using namespace OpenHRP;
namespace filesystem = boost::filesystem;
using fmt::format;

OpenHRPControllerItem::OpenHRPControllerItem()
{
    signalReadyStandardOutputConnected = false;
    mv = MessageView::instance();
}


OpenHRPControllerItem::OpenHRPControllerItem(const OpenHRPControllerItem& org)
    : ControllerItem(org)
{
    controllerServerName = org.controllerServerName;
    controllerServerCommand = org.controllerServerCommand;
    signalReadyStandardOutputConnected = false;
    mv = MessageView::instance();
}


OpenHRPControllerItem::~OpenHRPControllerItem()
{
    controllerServerProcess.waitForFinished(100);
}


void OpenHRPControllerItem::onDisconnectedFromRoot()
{
    if(controllerServerProcess.state() != QProcess::NotRunning){
        controllerServerProcess.kill();
    }
    ControllerItem::onDisconnectedFromRoot();
}


Item* OpenHRPControllerItem::doDuplicate() const
{
    return new OpenHRPControllerItem(*this);
}


void OpenHRPControllerItem::setControllerServerName(const std::string& name)
{
    controllerServerName = name;
}


void OpenHRPControllerItem::setControllerServerCommand(const std::string& command)
{
    controllerServerCommand = command;
}


bool OpenHRPControllerItem::initialize(ControllerIO* io)
{
    ncHelper = getDefaultNamingContextHelper();
    
    if(!ncHelper->isAlive()){
        return false;
    }

#ifdef OPENHRP_3_0
    typedef OpenHRP::ControllerFactory_var ControllerServer_var;
    typedef OpenHRP::ControllerFactory ControllerServer;
#elif OPENHRP_3_1
    typedef OpenHRP::Controller_var ControllerServer_var;
    typedef OpenHRP::Controller ControllerServer;
#endif
        
    ControllerServer_var server = ncHelper->findObject<ControllerServer>(controllerServerName.c_str());
    
    // do null check here

    bool serverReady = ncHelper->isObjectAlive(server);

    if(!serverReady){

        // invoke the controller command
        if(!controllerServerCommand.empty()){
            if(controllerServerProcess.state() != QProcess::NotRunning){
                controllerServerProcess.kill();
                controllerServerProcess.waitForFinished(100);
            }
            string command(controllerServerCommand);
#ifdef _WIN32
            if(filesystem::path(controllerServerCommand).extension() != ".exe"){
                command += ".exe";
            }
            // quote the command string to support a path including spaces
            controllerServerProcess.start(QString("\"") + command.c_str() + "\"");
#else
            controllerServerProcess.start(command.c_str());
#endif

            if(!controllerServerProcess.waitForStarted()){
                mv->put(format(_("Controller server process \"{}\" cannot be executed."), command));
                if(!filesystem::exists(command)){
                    mv->putln(_(" This file does not exist."));
                } else {
                    mv->putln("");
                }

            } else {
                mv->putln(format(_("Controller server process \"{0}\" has been executed by {1}."),
                                 command, name()));
                
                for(int i=0; i < 20; ++i){
                    controllerServerProcess.waitForReadyRead(10);
                    server = ncHelper->findObject<ControllerServer>(controllerServerName.c_str());
                    serverReady = ncHelper->isObjectAlive(server);
                    if(serverReady){
                        if(!signalReadyStandardOutputConnected){
                            controllerServerProcess.sigReadyReadStandardOutput().connect(
                                std::bind(&OpenHRPControllerItem::onReadyReadServerProcessOutput, this));
                            signalReadyStandardOutputConnected = true;
                        }
                        break;
                    }
                }
            }
        }
    }

    if(!serverReady){
        mv->putln(format(_("Controller server object \"{}\" is not found in the name server."),
                         controllerServerName));
        if(controllerServerProcess.state() != QProcess::NotRunning){
            controllerServerProcess.kill();
        }
        return false;
    }

    Body* body = io->body();

#ifdef OPENHRP_3_0
    controller = server->create(body->name().c_str());
    // do null check here
    mv->putln(format(_("The CORBA object of controller \"{0}\" has been created by the factory \"{1}\"."),
                     name(), controllerServerName));

#elif OPENHRP_3_1
    controller = server;
    controller->setModelName(body->name().c_str());
    controller->initialize();
    mv->putln(format(_("The CORBA object \"{0}\" of controller \"{1}\" has been obtained."),
                     controllerServerName, name()));
#endif

    timeStep_ = io->timeStep();

    dynamicsSimulator.reset(new DynamicsSimulator_impl(body));
    
    controller->setDynamicsSimulator(dynamicsSimulator->_this());
    controller->setTimeStep(timeStep_);
    
    return true;
}

    
bool OpenHRPControllerItem::start()
{
    controller->start();
    return true;
}


double OpenHRPControllerItem::timeStep() const
{
    return timeStep_;
}


void OpenHRPControllerItem::input()
{
    controller->input();
}


bool OpenHRPControllerItem::control()
{
    controller->control();
    return true;
}


void OpenHRPControllerItem::output()
{
    controller->output();
}

    
void OpenHRPControllerItem::stop()
{
    controller->stop();

#ifdef OPENHRP_3_0
    controller->destroy();
#endif

    if(controllerServerProcess.state() != QProcess::NotRunning){
#ifdef OPENHRP_3_1
        controller->destroy();
#endif
        controllerServerProcess.kill();
    }
}


void OpenHRPControllerItem::onReadyReadServerProcessOutput()
{
    MessageView::instance()->put(QString(controllerServerProcess.readAll()));
}


void OpenHRPControllerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    ControllerItem::doPutProperties(putProperty);
    
    putProperty(_("Controller server name"), controllerServerName,
                std::bind(&OpenHRPControllerItem::setControllerServerName, this, _1), true);
    putProperty(_("Controller server command"), controllerServerCommand,
                std::bind(&OpenHRPControllerItem::setControllerServerCommand, this, _1), true);
}


bool OpenHRPControllerItem::store(Archive& archive)
{
    if(!ControllerItem::store(archive)){
        return false;
    }
    archive.write("controllerServerName", controllerServerName);
    archive.writeRelocatablePath("controllerServerCommand", controllerServerCommand);
    return true;
}


bool OpenHRPControllerItem::restore(const Archive& archive)
{
    if(!ControllerItem::restore(archive)){
        return false;
    }
    setControllerServerName(archive.get("controllerServerName", controllerServerName));
    archive.readRelocatablePath("controllerServerCommand", controllerServerCommand);
    return true;
}
