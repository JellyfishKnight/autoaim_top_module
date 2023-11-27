FROM ros:humble-ros-base

# config apt
RUN apt-get update && apt upgrade -y && \
    sed -i 's@//.*archive.ubuntu.com@//mirrors.ustc.edu.cn@g' /etc/apt/sources.list && \
    apt-get update && apt upgrade -y

# create workspace and config git ssh key
RUN mkdir -p /autoaim_ws
WORKDIR /autoaim_ws/

# config git ssh key
ARG GIT_USERNAME GIT_EMAIL
RUN mkdir -p /root/.ssh/
COPY .ssh/id_rsa /root/.ssh/id_rsa
COPY .ssh/id_rsa.pub /root/.ssh/id_rsa.pub
RUN apt-get install -y openssh-client && \
    chmod 600 /root/.ssh/id_rsa && chmod 600 /root/.ssh/id_rsa.pub && \
    eval $(ssh-agent -s) && ssh-add /root/.ssh/id_rsa && \
    echo "Host e.coding.net\n\tStrictHostKeyChecking no\n" >> ~/.ssh/config

# clone projects and config
RUN git config --global user.name ${GIT_USERNAME} && \
    git config --global user.email ${GIT_EMAIL} && \
    git clone git@e.coding.net:swjtuhelios/cv/autoaim_top_module.git --recursive && \
    mv autoaim_top_module src && cd src && \
    git submodule update --init --recursive

# install dependencies and some tools
RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y && \
    apt-get install ros-humble-foxglove-bridge wget htop vim libceres-dev libpcl-ros-dev -y && \
    wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
    apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
    echo "deb https://apt.repos.intel.com/openvino/2023 ubuntu22 main" | tee /etc/apt/sources.list.d/intel-openvino-2023.list && \
    apt-get update && apt install openvino-2023.2.0 -y && \
    rm -rf /var/lib/apt/lists/* 

# setup zsh
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
    -t jispwoso -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting && \
    chsh -s /bin/zsh && \
    rm -rf /var/lib/apt/lists/*

# build
RUN  touch src/helios_autoaim/autoaim_fire_controller/COLCON_IGNORE && \
     . /opt/ros/humble/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# setup .zshrc
RUN echo 'export TERM=xterm-256color\n\
    source /ros_ws/install/setup.zsh\n\
    eval "$(register-python-argcomplete3 ros2)"\n\
    eval "$(register-python-argcomplete3 colcon)"\n'\
    >> /root/.zshrc


# source entrypoint setup
RUN sed --in-place --expression \
      '$isource "/autoaim_ws/install/setup.bash"' \
      /ros_entrypoint.sh



