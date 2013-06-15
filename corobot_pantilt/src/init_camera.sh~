#init_camera.sh
echo `dirname $0`
export MJPG_ROOT=$(rospack find corobot_pantilt)/src/mjpg-streamer
export LD_LIBRARY_PATH=$MJPG_ROOT/
$MJPG_ROOT/mjpg_streamer -i "input_uvc.so -d /dev/video0" -o "output_http.so -w /usr/local/www -p 8081"&
sleep 1
killall mjpg_streamer
