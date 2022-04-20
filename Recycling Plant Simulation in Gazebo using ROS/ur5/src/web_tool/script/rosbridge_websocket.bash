#!/bin/bash

ROSBRIDGE_PORT=55001
CERTIFICATE_NAME="certfile.pem"
CERTIFICATE_PATH="$(pwd)/$CERTIFICATE_NAME"

echo .
echo Running ROSBRIDGE
echo PORT: $ROSBRIDGE_PORT
echo CERTIFICATE: $CERTIFICATE_PATH
echo .

roslaunch rosbridge_server rosbridge_websocket.launch ssl:=true certfile:=$CERTIFICATE_PATH keyfile:=$CERTIFICATE_PATH authenticate:=false port:=$ROSBRIDGE_PORT
