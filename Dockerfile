FROM ros
WORKDIR /interop-client/
RUN apt-get update && apt-get install -y python-pip \
  ros-kinetic-gazebo-ros-control
COPY . .
RUN ./setup.sh
RUN rm setup.sh
CMD /bin/bash client.sh
