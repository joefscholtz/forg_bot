#!/usr/bin/env bash

SCRIPT_PATH=$(dirname "$0")
docker build --target forg_bot_dev --tag forg_bot:dev $SCRIPT_PATH
