FROM osrf/ros:noetic-desktop-full

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV HOME /home/$USERNAME

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && apt-get install -y sudo \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends \
    curl \
    git \
    python3-catkin-tools \
    clang-format

RUN git clone https://github.com/Huoleit/cmake_clang_tools.git /tmp/clang_tool
RUN cd /tmp/clang_tool && mkdir build && cd build \
    && cmake .. && make && make install

#################
## Install zsh ##
#################
RUN apt-get -y install --no-install-recommends \
    zsh \
    autojump

USER $USERNAME

RUN sh -c "$(curl -fsSL https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"
COPY ./config/docker.zsh-theme $HOME/.oh-my-zsh/themes/
COPY ./config/.zshrc $HOME/
ENV SHELL /bin/zsh

CMD [ "zsh" ]
