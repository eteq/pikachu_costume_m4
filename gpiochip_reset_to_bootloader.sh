#!/bin/sh

gpioset 1 0=1
gpioset 1 0=0
sleep 0.1
gpioset 1 0=1
sleep 0.1
gpioset 1 0=0
sleep 0.1
gpioset 1 0=1

sleep 0.5
