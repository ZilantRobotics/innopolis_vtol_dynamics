FIRMWARE_PATH="~/Work/vtol/Firmware"


tmux start-server

sleep 1

tmux new -s vtol_dynamics -d
tmux rename-window -t vtol_dynamics vtol_dynamics


tmux split-window -v -t vtol_dynamics

tmux select-pane -t vtol_dynamics:0.0
tmux split-window -h -t vtol_dynamics

tmux select-pane -t vtol_dynamics:0.0
tmux send-keys "cd $FIRMWARE_PATH
export PX4_HOME_LAT=$START_LAT
export PX4_HOME_LON=$START_LON
export PX4_HOME_ALT=$START_ALT
no_sim=1 make px4_sitl gazebo_standard_vtol" C-m

tmux select-pane -t vtol_dynamics:0.1
tmux send-keys "roslaunch mavros px4.launch fcu_url:=\"udp://:14560@127.0.0.1:14558\"" C-m

tmux attach -t vtol_dynamics