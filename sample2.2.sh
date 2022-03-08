#! /bin/bash

[ -z "$1" ] && echo 'error' && exit 1

filename="$1"

iteration=$(command ls -1 | grep -E "^${filename}_(ms|of)_[1-9]+\.csv\$" | sed -r 's/.*([1-9]+).*/\1/g' | uniq | sort -r | head -n 1)
iteration=$(("$iteration"+1))
echo "iteration $iteration"

of="${filename}_of_${iteration}.csv"
ms="${filename}_ms_${iteration}.csv"
rho="${filename}_rho_${iteration}.csv"

echo "$of"
echo "$ms"
echo "$rho"

function postprocess() {
	echo "cleaning up and fixing header row in csv files"
	kill -9 "$model_states_pid"
	kill -9 "$odometry_filtered_pid"
	kill -9 "$rho_pid"
	sleep 2
	
	sed -ri -e '/^%/d' -e '$d' "$ms"
	sed -ri -e '/^%/d' -e '$d' "$of"
	sed -ri -e '/^%/d' -e '$d' "$rho"
	exit 0
}

trap postprocess SIGINT

echo 'timestamp,x,y' > "$ms"
rostopic echo /gazebo/model_states/pose["$2"]/position -p | cut -f -3 -d , >> "$ms" &
model_states_pid=$!

echo 'timestamp,x,y,z,r,p,y' > "$of"
rostopic echo /odometry/filtered/pose/pose/position -p | cut -f -3 -d , >> "$of" &
odometry_filtered_pid=$!

echo 'timestamp,x,y' > "$of"
rostopic echo /husky_controllers/position_error -p | cut -f -3 -d , >> "$rho" &
rho_pid=$!

while true; do
	echo "sampling..."
	sleep 3
done