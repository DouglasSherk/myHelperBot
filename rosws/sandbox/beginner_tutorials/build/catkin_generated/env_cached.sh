#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/bitch/myHelperBot/rosws/sandbox/beginner_tutorials/build/catkin_generated', type 'exit' to leave"
  . "/home/bitch/myHelperBot/rosws/sandbox/beginner_tutorials/build/catkin_generated/setup_cached.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/bitch/myHelperBot/rosws/sandbox/beginner_tutorials/build/catkin_generated'"
else
  . "/home/bitch/myHelperBot/rosws/sandbox/beginner_tutorials/build/catkin_generated/setup_cached.sh"
  exec "$@"
fi
