#!/usr/bin/sh

sudo mavproxy.py --master=/dev/ttyACM0 --out=udpin:0.0.0.0:14550
