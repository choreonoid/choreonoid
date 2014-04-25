
# @author Hisashi Ikari

from PySide import QtCore
from PySide import QtGui
import cnoid.Util as util
import cnoid.Base as base

top = util.executableTopDirectory()
icon = QtGui.QIcon(top + '/src/Base/icons/refresh.png')

timebar = base.TimeBar.instance()
button = timebar.addButton(icon, "This is a test by python-script")

def hello():
    print "Hello PySide of Python by the qt4 signal system of C++"

button.clicked(hello)
