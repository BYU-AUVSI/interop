FROM ros
WORKDIR /interop-client/
RUN apt-get update && apt-get install -y python-pip
RUN pip install requests
RUN pip install --upgrade pip
COPY . .
CMD /bin/bash client.sh
