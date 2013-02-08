#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/bitch/myHelperBot/rosws/myhelperbot/build/devel', type 'exit' to leave"
  . "/home/bitch/myHelperBot/rosws/myhelperbot/build/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/bitch/myHelperBot/rosws/myhelperbot/build/devel'"
else
  . "/home/bitch/myHelperBot/rosws/myhelperbot/build/devel/setup.sh"
  exec "$@"
fi
