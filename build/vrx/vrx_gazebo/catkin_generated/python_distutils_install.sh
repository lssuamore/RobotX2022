#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/amore/RobotX2022/src/vrx/vrx_gazebo"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/amore/RobotX2022/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/amore/RobotX2022/install/lib/python2.7/dist-packages:/home/amore/RobotX2022/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/amore/RobotX2022/build" \
    "/usr/bin/python2" \
    "/home/amore/RobotX2022/src/vrx/vrx_gazebo/setup.py" \
     \
    build --build-base "/home/amore/RobotX2022/build/vrx/vrx_gazebo" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/amore/RobotX2022/install" --install-scripts="/home/amore/RobotX2022/install/bin"
