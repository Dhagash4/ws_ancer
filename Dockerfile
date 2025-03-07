# Use container registry for the base images
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-perception AS base

ARG UID=1000
ARG GID=1000
ENV UNAME=ros
ENV ROSWS=/home/${UNAME}/ws_anscer
ENV DEBIAN_FRONTEND=noninteractive

# Add normal sudo-user to container, passwordless
RUN addgroup --gid $GID $UNAME \
  && adduser --disabled-password --gecos '' --uid $UID --gid $GID $UNAME \
  && adduser $UNAME sudo \
  && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
  && sed -i 's/required/sufficient/' /etc/pam.d/chsh \
  && touch /home/$UNAME/.sudo_as_admin_successful


# Install some custom packages you need for the dockerfile
RUN apt update && apt upgrade -y


# install dependencies
RUN apt-get update && apt-get install -y \
  sudo \
  zsh \
  git \
  curl \
  unzip \
  bat \
  systemctl \
  clang-tidy \ 
  clang-format \
  python3-pip \
  python3-venv \
  ripgrep \
  build-essential \
  'ros-humble-gazebo-*' \
  'ros-humble-turtlebot3-*' \
  ros-$ROS_DISTRO-desktop-full \
  cmake \
  wget \
  tmux \
  clang \
  clangd \
  htop \ 
  ros-humble-foxglove-bridge \
  && rm -rf /var/lib/apt/lists/*


RUN apt-get update && apt-get install -y locales && \
  locale-gen en_US.UTF-8 && \
  update-locale LANG=en_US.UTF-8

USER ${UNAME}

ENV HOME=/home/${UNAME}


# Install Oh My Zsh
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended


## Set zsh as the default shell
RUN chsh -s $(which zsh)

# persistent zshrc history
USER root
RUN SNIPPET="export PROMPT_COMMAND='history -a' && export HISTFILE=/cmd/.zsh_history" \
  && rm -rf /cmd \
  && mkdir /cmd \
  && touch /cmd/.zsh_history \
  && chown -R ${UNAME} /cmd \
  && echo "$SNIPPET" >> "${HOME}/.zshrc"

USER ${UNAME}


RUN echo "source /opt/ros/humble/setup.zsh" >> "${HOME}/.zshrc" \
  && echo 'eval "$(register-python-argcomplete3 ros2)"' >> "${HOME}/.zshrc" \
  && echo 'eval "$(register-python-argcomplete3 colcon)"' >> "${HOME}/.zshrc"


# Setup workdir
WORKDIR ${HOME}
RUN mkdir -p ${ROSWS}

# some mandatory stuff
RUN sudo chown -R ${UNAME} ${ROSWS} 
COPY entrypoint.sh /usr/local/bin/entrypoint.sh


RUN sudo chmod +x /usr/local/bin/entrypoint.sh
# Set the entrypoint
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD [ "zsh" ]
