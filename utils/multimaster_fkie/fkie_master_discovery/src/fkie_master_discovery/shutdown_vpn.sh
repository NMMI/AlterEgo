#!/bin/bash

catch_kill() {
  echo "Caught SIGKILL signal!"

  echo $VPN_NETWORK_ID
  echo $VPN_NETWORK_ID
  echo $VPN_NETWORK_ID
  echo $VPN_NETWORK_ID
  echo $VPN_NETWORK_ID
  echo $VPN_NETWORK_ID
#   zerotier-cli leave $VPN_NETWORK_ID
  kill -KILL "$pid" 2>/dev/null
}

catch_term() {
  echo "Caught SIGTERM signal!"
    echo $VPN_NETWORK_ID
  echo $VPN_NETWORK_ID
  echo $VPN_NETWORK_ID
  echo $VPN_NETWORK_ID
  echo $VPN_NETWORK_ID
  echo $VPN_NETWORK_ID
  kill -TERM "$pid" 2>/dev/null
}

catch_quit() {
  echo "Caught SIGTERM signal!"
    echo $VPN_NETWORK_ID
  echo $VPN_NETWORK_ID
  echo $VPN_NETWORK_ID
  echo $VPN_NETWORK_ID
  echo $VPN_NETWORK_ID
  echo $VPN_NETWORK_ID
  kill -QUIT "$pid" 2>/dev/null
}

catch_ctrlc() {
  echo "Caught ctrl+c!"
    echo $VPN_NETWORK_ID
  echo $VPN_NETWORK_ID
  echo $VPN_NETWORK_ID
  echo $VPN_NETWORK_ID
  echo $VPN_NETWORK_ID
  echo $VPN_NETWORK_ID
  kill -KILL "$pid" 2>/dev/null
}

trap catch_term SIGTERM
trap catch_kill SIGKILL
trap catch_quit SIGQUIT
trap catch_ctrlc INT

echo "Script is running! waiting for signals."

pid=$$

sleep infinity