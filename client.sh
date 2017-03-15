# if the master node hasn't been set (or its the default value), see if the user tried setting it shorthand
if [ -z "$ROS_MASTER_URI" ] || [ "$ROS_MASTER_URI" == "http://localhost:11311/" ] || [ "$ROS_MASTER_URI" == "http://localhost:11311" ]; then
    # sourcing this writes over ROS_MASTER_URI if not defined
    source /opt/ros/kinetic/setup.bash
    if [ ! -z "${ROS_MASTER_IP}" ]; then
      export ROS_MASTER_URI="http://${ROS_MASTER_IP}:11311/"
    elif [ ! -z "${ROS_MASTER}" ]; then
      export ROS_MASTER_URI="http://${ROS_MASTER}:11311/"
    elif [ ! -z "${MASTER_IP}" ]; then
      export ROS_MASTER_URI="http://${MASTER_IP}:11311/"
    elif [ ! -z "${MASTER}" ]; then
      export ROS_MASTER_URI="http://${MASTER}:11311/"
    fi
else
  source /opt/ros/kinetic/setup.bash
fi

# run the client.py program
python "$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/client.py"
