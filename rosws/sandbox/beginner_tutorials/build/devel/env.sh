#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/bitch/myHelperBot/rosws/sandbox/beginner_tutorials/build/devel', type 'exit' to leave"
  . "/home/bitch/myHelperBot/rosws/sandbox/beginner_tutorials/build/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/bitch/myHelperBot/rosws/sandbox/beginner_tutorials/build/devel'"
else
  . "/home/bitch/myHelperBot/rosws/sandbox/beginner_tutorials/build/devel/setup.sh"
  exec "$@"
fi
