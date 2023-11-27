# 兼容传统电控代码的自瞄部署仓库

## 快速部署

### 手动部署

    mkdir autoaim_ws && cd autoaim_ws && \
    git clone git@e.coding.net:swjtuhelios/cv/autoaim_top_module.git --recursive && \
    mv autoaim_top_module src && cd src && \
    git submodule update --init --recursive

### docker部署
#### 容器构建

注意：如果需要自己构建容器，需要先将需要部署的电脑上配置git-ssh，然后执行以下命令：
    
    cp ~/.ssh ~/autoaim_ws/ -r && \
    cd ~/autoaim_ws && \
    docker build --build-arg GIT_USERNAME=<USERNAME> --build-arg GIT_EMAIL=<YOUR_EMAIL> -t autoaim_deploy .

#### 直接部署命令

    # 配置凭据
    docker login -u <USERNAME> -p <PASSWORD> swjtuhelios-docker.pkg.coding.net
    # 拉取镜像
    docker pull swjtuhelios-docker.pkg.coding.net/cv/rm_dev_docker/autoaim_deploy:lastest
    # 构建开发容器(调试的时候使用)
    docker run -it --name autoaim_devel \
        --privileged --network host \
        -v /dev:/dev -v $HOME/.ros:/root/.ros \
        swjtuhelios-docker.pkg.coding.net/cv/rm_dev_docker/autoaim_deploy:lastest \
    # 构建运行容器(部署上场的时候用)
    docker run -it --name autoaim_runtime \
        --privileged --network host --restart always \
        -v /dev:/dev -v $HOME/.ros:/root/.ros \
        swjtuhelios-docker.pkg.coding.net/cv/rm_dev_docker/autoaim_deploy:lastest \

## 快速拉取远程更改
    cd autoaim_ws && \
    sudo chmod +x ./auto_pull.sh && \
    ./auto_pull.sh

## 快速推送本地更改
    cd autoaim_ws && \
    sudo chmod +x ./auto_push.sh && \
    ./auto_push.sh