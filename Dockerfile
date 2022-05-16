FROM ros:noetic
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y git \
	python3-pip \
	python3-catkin-tools \
	python3-opengl \
	pcl-tools \
	python3-pcl \
  	ros-noetic-cv-bridge \
	libpcl-dev \
      	&& rm -rf /var/lib/apt/lists/*

WORKDIR /home
RUN git clone https://github.com/markusltnr/gpd.git


WORKDIR /home/gpd/build
RUN cmake .. && make -j
