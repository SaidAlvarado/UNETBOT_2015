# en la pc

# gst-launch-1.0 udpsrc port=5000 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! autovideosink sync=false


# en el raspberry pi

# raspivid -t 999999 -w 1280 -h 720 -fps 25 -vf -hf -b 2000000 -o - | gst-launch-1.0 -e -vvv fdsrc ! h264parse ! rtph264pay pt=96 config-interval=5 ! udpsink host=192.168.1.7  port=5000





