util = require "cnoid.Util"
base = require "cnoid.Base"

function onTimeChanged(time)
   print("time changed: ", time)
   return true
end

timeBar = base.TimeBar.instance()
timeBar:sigTimeChanged():connect(onTimeChanged)


