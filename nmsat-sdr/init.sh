#!/bin/bash

docker run -it -v "$PWD:/bench" --privileged -v /dev/bus/usb:/dev/bus/usb coherence

