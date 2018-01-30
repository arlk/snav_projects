#!/bin/bash

APP="test_app"
REMOTE="thiago@msi:/home/thiago/ws"
LOCAL="/home/linaro/ws"

OPTS=""
if [ "$1" != "" ]; then
  OPTS="$1"
fi

if [ "$2" == "d" ]; then
  set -x
  rsync -auzP --delete --exclude "$REMOTE/.git" "$REMOTE/$APP" $LOCAL && make $OPTS
else
  set -x
  rsync -auzP --exclude "$REMOTE/.git" "$REMOTE/$APP" $LOCAL && make $OPTS
fi
