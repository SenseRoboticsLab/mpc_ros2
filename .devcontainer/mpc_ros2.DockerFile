FROM osrf/ros:humble-desktop-full


# RUN 
RUN DEBIAN_FRONTEND="noninteractive" apt-get update && apt-get -y install tzdata

RUN apt-get update\
&& apt-get install -y ssh \
      build-essential \
      gcc \
      g++ \
      gdb \
      clang \
      make \
      ninja-build \
      cmake \
      autoconf \
      automake \
      locales-all \
      dos2unix \
      rsync \
      tar \
      python3 \
      vim \
      git \
      cmake-gui && apt-get clean



RUN apt-get update && apt-get install -y openssh-server
# RUN mkdir /var/run/sshd
RUN echo 'root:123' | chpasswd


RUN useradd -rm -d /home/user -s /bin/bash -g root -G sudo -u 1001 user
RUN  echo 'user:123' | chpasswd
EXPOSE 22

RUN echo 'user ALL=(ALL) NOPASSWD: ALL' | tee -a /etc/sudoers
ENV DIST=humble
RUN echo "source /opt/ros/${DIST}/setup.bash" >> /home/user/.bashrc
RUN echo "sudo service ssh start" >> /home/user/.bashrc

USER user

# dependency for IPOPT, mpc_ros
RUN sudo apt install -y gfortran python3-pip ros-${DIST}-tf-transformations  && sudo apt clean
# # USER user
# # CMD ["/usr/sbin/sshd", "-D", "-e", "-f", "/etc/ssh/sshd_config_test_clion"]
WORKDIR /home/user/project/ros/bluerov2_ws
CMD [ "bash" ]
#CMD ["/usr/sbin/sshd","-D"]