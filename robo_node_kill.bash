ps aux | grep gazebo | grep -v grep | awk '{ print "kill -9", $2 }' | sh
