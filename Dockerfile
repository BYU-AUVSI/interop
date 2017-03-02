FROM ros
WORKDIR /interop-client/
COPY . .
CMD /bin/bash client.sh
