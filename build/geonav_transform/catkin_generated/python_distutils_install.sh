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

echo_and_run cd "/home/brad/RobotX2022/src/geonav_transform"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/brad/RobotX2022/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/brad/RobotX2022/install/lib/python3/dist-packages:/home/brad/RobotX2022/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/brad/RobotX2022/build" \
    "/usr/bin/python3" \
    "/home/brad/RobotX2022/src/geonav_transform/setup.py" \
     \
    build --build-base "/home/brad/RobotX2022/build/geonav_transform" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/brad/RobotX2022/install" --install-scripts="/home/brad/RobotX2022/install/bin"
