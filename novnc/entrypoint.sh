#!/bin/bash
set -ex

RUN_FLUXBOX=${RUN_FLUXBOX:-yes}
RUN_XTERM=${RUN_XTERM:-yes}

case $RUN_FLUXBOX in
  false|no|n|0)
    rm -f /novnc/conf.d/fluxbox.conf
    ;;
esac

case $RUN_XTERM in
  false|no|n|0)
    rm -f /novnc/conf.d/xterm.conf
    ;;
esac

# mkdir -p ~/.vnc
# x11vnc -storepasswd ${VNC_PW} ~/.vnc/passwd

exec supervisord -c /novnc/supervisord.conf
