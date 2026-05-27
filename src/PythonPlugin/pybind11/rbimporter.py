import sys
import os.path

if sys.version_info[0] == 2:
    import __builtin__
    builtins = __builtin__
else:
    import builtins

rollbackImporter = None

class RollbackImporter:
    def __init__(self, scriptDir):
        self.scriptDir = scriptDir
        self.previousModules = []
        for k in sys.modules.keys():
            self.previousModules.append(k)
            self.realImport = builtins.__import__
            builtins.__import__ = self._import
            self.newModules = {}

    def _import(self, *args):
        n = args[0]
        f = args[3]
        try:
            g = args[1]
        except:
            g = {}
        try:
            l = args[2]
        except:
            l = {}
            result = apply(self.realImport, (n, g, l, f))
        if result:
            if hasattr(result, '__file__'):
                if os.path.dirname(result.__file__) == self.scriptDir:
                    self.newModules[n] = 1
        return result

    def uninstall(self):
        for m in self.newModules.keys():
            if m not in self.previousModules:
                if m in sys.modules:
                    del(sys.modules[m])
                    builtins.__import__ = self.realImport

def refresh(scriptDir):
    global rollbackImporter
    if rollbackImporter:
        rollbackImporter.uninstall()
        rollbackImporter = RollbackImporter(scriptDir)

def uninstall():
    if rollbackImporter:
        rollbackImporter.uninstall()

#refresh("")
