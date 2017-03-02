FROM ros
WORKDIR /interop-client/
COPY . .
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash"
CMD ./client.py
