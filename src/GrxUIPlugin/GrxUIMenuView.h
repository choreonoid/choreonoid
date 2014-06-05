/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GRXUI_PLUGIN_GRXUI_MENU_VIEW_H_INCLUDED
#define CNOID_GRXUI_PLUGIN_GRXUI_MENU_VIEW_H_INCLUDED

#include <cnoid/View>
#include <QMessageBox>
#include <boost/python.hpp>
#include "exportdecl.h"

namespace cnoid {

class GrxUIMenuViewImpl;
    
class CNOID_EXPORT GrxUIMenuView : public cnoid::View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static GrxUIMenuView* instance();
    static void setCancelExceptionType(boost::python::object exceptionType);
        
    GrxUIMenuView();
    virtual ~GrxUIMenuView();

    void setMenu(const boost::python::list& menu, bool isLocalSequentialMode, bool doBackgroundExecution);

    static QMessageBox::StandardButton waitInputSelect(const std::string& message);
    static bool waitInputConfirm(const std::string& message);
    static std::string waitInputMessage(const std::string& message);
    
private:
    friend class GrxUIMenuViewImpl;
    GrxUIMenuViewImpl* impl;
};
}

#endif
