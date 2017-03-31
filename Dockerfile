FROM osrf/ros:kinetic-desktop-full
WORKDIR /interop-client/
RUN apt-get update && apt-get install -y python-pip
COPY . .
RUN ./setup.sh
RUN rm setup.sh
CMD /bin/bash client.sh
