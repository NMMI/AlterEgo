#!/bin/bash

echo $1 | sudo -S route add -net 224.0.0.0 netmask 224.0.0.0 $2
exit 0