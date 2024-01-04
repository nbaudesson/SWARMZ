pkill -f 'MicroXRCEAgent'
cd ~/ros2_PX4_gz/Micro-XRCE-DDS-Agent
MicroXRCEAgent udp4 -p 8888 &
terminator --layout=SwarmZX -p hold
