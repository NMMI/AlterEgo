#!/bin/bash

# ------------------------------------------------------------------------- PARAMS
# -- VIDEO --
V_CODEC="libxvid"
V_BITRATE="1500k"
FRAME_RATE="60"
PIX_FMT="yuv410p"
VIDEO_SIZE="1280x720"
#VIDEO_SIZE="3840x1080"
Q_MIN="2"
Q_MAX="30"
GOP_SIZE="20"

# -- AUDIO --
A_CODEC="libopus"
A_BITRATE="44100"
N_CHANNELS="1"
A_SP_MIC=""

# -- CONNECTION --
IP_PILOT="192.168.0.62"
#IP_PILOT="10.240.29.158"

#HAND PARAM
FRAME_RATE_HAND="30"

V_PORT_PILOT="2000"
A_PORT_PILOT="2012"
V_PORT_L_PILOT="2024"
V_PORT_R_PILOT="2026"
V_PORT_HAND_PILOT="2032"

# -- SDP FILES --
AV_PILOT_SDP="AV_pilot.sdp"
A_PILOT_SDP="A_pilot.sdp"
V_PILOT_SDP="V_pilot.sdp"
AV_ROBOT_SDP="AV_robot.sdp"
A_ROBOT_SDP="A_robot.sdp"
V_ROBOT_SDP="V_robot.sdp"
SDP_FOLDER="SDP_folder"

# -- COLORS --
RED="\e[31m"
WHITE="\e[39m"
GREEN="\e[32m"
YELLOW="\e[93m"

i="0"
j="0"
MAX_CY="10"
MODE="$2"
V_DEV_L="/dev/video2"
V_DEV_R="/dev/video3"

#---------------------------------------SEND VIDEO LEFT--------------------------------------#
	gnome-terminal --title send_video -x bash -c "ffmpeg  \
		-f v4l2 \
		-framerate $FRAME_RATE \
		-video_size $VIDEO_SIZE \
		-pixel_format $PIX_FMT \
		-i $V_DEV_L \
		-fflags nobuffer \
		-fflags fastseek \
		-segment_list_flags live \
		-avioflags direct  \
		-g $GOP_SIZE -c:v $V_CODEC  -ssim_acc 4 -me_method zero -qmin $Q_MIN -qmax $Q_MAX -refs 1 -bf 0 -b:v $V_BITRATE -movflags faststart \
		-f mpegts udp://$IP_PILOT:$V_PORT_L_PILOT ;bash"


#---------------------------------------SEND VIDEO LEFT--------------------------------------#


	gnome-terminal --title send_video -x bash -c "ffmpeg  \
		-f v4l2 \
		-framerate $FRAME_RATE \
		-video_size $VIDEO_SIZE \
		-pixel_format $PIX_FMT \
		-i $V_DEV_R \
		-fflags nobuffer \
		-fflags fastseek \
		-segment_list_flags live \
		-avioflags direct  \
		-g $GOP_SIZE -c:v $V_CODEC  -ssim_acc 4 -me_method zero -qmin $Q_MIN -qmax $Q_MAX -refs 1 -bf 0 -b:v $V_BITRATE -movflags faststart \
		 -f mpegts udp://$IP_PILOT:$V_PORT_R_PILOT ;bash"
