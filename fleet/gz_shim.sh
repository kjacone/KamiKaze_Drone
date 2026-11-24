#!/bin/sh
# Pass-through placeholder for /usr/bin/gz.
# --headless-rendering (EGL) does not initialise in this image and yields no
# camera frames; rendering goes through the Xvfb GLX path instead, which is
# why px4_entrypoint.sh keeps Xvfb alive.
exec /usr/bin/gz "$@"
