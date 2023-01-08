FROM ros:noetic-ros-core
MAINTAINER peterpennrichjochem@gmail.com


# Install thirdparty packages
#RUN pip3 install -r requirements.txt


#RUN apt install ros-melodic-tf2-msgs ros-noetic-rospy
# install pyKDL and copy it into the right spot
# https://answers.ros.org/question/380142/how-to-install-tf2_geometry_msgs-dependency-pykdl/
sudo apt-get install python-sip


# Install developer tools 
RUN apt update && apt install -y vim

CMD ["tail", "-f /dev/null"]
