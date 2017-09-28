cnoid.require "Util"
cnoid.require "Base"

function onTimeChanged(time)
   print("time changed: ", time)
   return true
end

timeBar = cnoid.TimeBar.instance()
timeBar:sigTimeChanged():connect(onTimeChanged)
