FROM dustynv/ros:humble-desktop-pytorch-l4t-r35.4.1

# NOVNC remote desktop Nabbed from https://github.com/theasp/docker-novnc/blob/master/Dockerfile

# Install git, supervisor, VNC, & X11 packages
RUN apt-get update && \
    apt-get install -y \
      bash \
      fluxbox \
      git \
      net-tools \
      novnc \
      supervisor \
      x11vnc \
      xterm \
      xvfb && \
    rm -rf /var/lib/apt/lists/ && \
    ln -s /usr/share/novnc/vnc.html /usr/share/novnc/index.html
# vnc.html or vnc_lite.html

# Setup demo environment variables
ENV HOME=/root \
    DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    LC_ALL=C.UTF-8 \
    DISPLAY=:0.0 \
    DISPLAY_WIDTH=1024 \
    DISPLAY_HEIGHT=768 \
    RUN_XTERM=yes \
    RUN_FLUXBOX=yes \
    PASSWORD=abc123


    
COPY . /novnc
CMD ["/novnc/entrypoint.sh"]
EXPOSE 9002


#@  You can create an x11vnc password file by running:       @#
#@                                                           @#
#@       x11vnc -storepasswd password /path/to/passfile      @#
#@  or   x11vnc -storepasswd /path/to/passfile               @#
#@  or   x11vnc -storepasswd                                 @#
#@                                                           @#
#@  (the last one will use ~/.vnc/passwd)                    @#
#@                                                           @#
#@  and then starting x11vnc via:                            @#
#@                                                           @#
#@      x11vnc -rfbauth /path/to/passfile                    @#

# TODO: 
# RUN mkdir ~/.vnc && \
#     x11vnc -storepasswd ${VNC_PW} ~/.vnc/passwd && \

