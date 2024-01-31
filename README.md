# 兼容传统电控代码的自瞄部署仓库

## 快速部署

### 手动部署(需要自己手动在主机上配置ssh-key)

    mkdir autoaim_ws && cd autoaim_ws && \
    git clone git@e.coding.net:swjtuhelios/cv/autoaim_top_module.git --recursive && \
    mv autoaim_top_module src && cd src && \
    git submodule update --init --recursive

### docker部署
#### 容器构建

注意：如果需要自己构建容器，执行以下命令：
    
    cd ~/autoaim_ws/src && \
    docker build --build-arg ROS_DOAMIN_ID=<ID> -t autoaim_deploy .

#### 直接部署命令

    # 配置凭据
    docker login -u <USERNAME> -p <PASSWORD> swjtuhelios-docker.pkg.coding.net
    # 拉取镜像
    docker pull swjtuhelios-docker.pkg.coding.net/cv/rm_dev_docker/autoaim_deploy:lastest
    # 构建开发容器(调试的时候使用)
    docker run -it --name autoaim_devel \
        --privileged --network host \
        -v /dev:/dev -v $HOME/.ros:/root/.ros \
        swjtuhelios-docker.pkg.coding.net/cv/rm_dev_docker/autoaim_deploy:runtime_v1 \
    # 构建运行容器(部署上场的时候用)
    docker run -it --name autoaim_runtime \
        --privileged --network host --restart always \
        -v /dev:/dev -v $HOME/.ros:/root/.ros \
        swjtuhelios-docker.pkg.coding.net/cv/rm_dev_docker/autoaim_deploy:runtime_v1 \
        ros2 launch autoaim_bring_up autoaim.launch.py

## 快速拉取远程更改
    cd autoaim_ws && \
    sudo chmod +x ./auto_pull.sh && \
    ./auto_pull.sh

## 快速推送本地更改
    cd autoaim_ws && \
    sudo chmod +x ./auto_push.sh && \
    ./auto_push.sh