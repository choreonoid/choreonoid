/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyQObjectHolder.h"
#include "PyQString.h"
#include "PyQtSignal.h"
#include <QWidget>
#include <QLayout>
#include <QMainWindow>
#include <QPushButton>
#include <QToolButton>
#include <QCheckBox>
#include <QLabel>
#include <QSpinBox>
#include <QDialog>
#include <QFrame>
#include <QAbstractScrollArea>
#include <QMenu>

namespace py = pybind11;

namespace cnoid {

}

using namespace cnoid;

PYBIND11_MODULE(QtGui, m)
{
    m.doc() = "Choreonoid QtGui module";

    py::module::import("cnoid.QtCore");
}
