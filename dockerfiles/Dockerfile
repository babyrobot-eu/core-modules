FROM tensorflow/tensorflow:1.2.1-gpu

## INSTALL ROS

# setup keys
RUN apt-key adv --keyserver hkp://p80.pool.sks-keyservers.net:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    curl \
    wget \
    git \
    openssh-client \
    python-tk \
    protobuf-compiler \
    tmux \
    && rm -rf /var/lib/apt/lists/*

RUN curl -s https://bootstrap.pypa.io/get-pip.py | python

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-core=1.3.1-0* \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> /root/.bashrc
# install ros packages
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-base=1.3.1-0* \
    && rm -rf /var/lib/apt/lists/*

## GET THE BABYROBOT CODE FROM GITLAB
RUN echo "HEY"
RUN mkdir -p /root/.ssh
COPY ./id_rsa_docker /root/.ssh/id_rsa
RUN chmod 700 /root/.ssh/id_rsa
COPY ./id_rsa_docker.pub /root/.ssh/id_rsa.pub
RUN  echo "    IdentityFile /root/.ssh/id_rsa\n" >> /etc/ssh/ssh_config
RUN echo "Host gitlab.com\n" >> /etc/ssh/ssh_config
RUN echo "    StrictHostKeyChecking no" >> /etc/ssh/ssh_config

RUN git clone --recursive git@gitlab.com:georgepar/babyrobot-integration.git /babyrobot-integration
WORKDIR /babyrobot-integration
RUN git submodule update --remote
RUN git checkout docker-install # TODO: remove when ready to merge
RUN pip install -r ./requirements.txt
WORKDIR /babyrobot-integration/babyrobot
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make && catkin_make install"
RUN echo 'source /babyrobot-integration/babyrobot/devel/setup.bash' >> /root/.bashrc

## SETUP OPENSMILE

RUN apt-get update && apt-get install -y \
    autoconf \
    automake \
    build-essential \
    libtool \
    pkg-config \
    wget \
    && apt-get clean

WORKDIR /opt
RUN wget http://audeering.com/download/1318/ -O opensmile-2.3.0.tar.gz \
    && tar xvf opensmile-2.3.0.tar.gz \
    && rm opensmile-2.3.0.tar.gz

RUN cd opensmile-2.3.0 \
    && sed -i '/chmod/async; sleep 1' buildStandalone.sh \
    && ./buildStandalone.sh -p /usr/local


## SETUP EMOTION RECOGNITION

ENV CLASSPATH /opt/weka3-8/weka

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y  software-properties-common unzip
RUN add-apt-repository ppa:webupd8team/java -y
RUN apt-get update
RUN echo oracle-java7-installer shared/accepted-oracle-license-v1-1 select true | /usr/bin/debconf-set-selections
RUN apt-get install -y oracle-java8-installer
RUN apt-get install -y libsvm-tools python-libsvm
RUN apt-get clean; apt-get autoclean; apt-get autoremove -y

WORKDIR /opt
ADD http://www.cs.waikato.ac.nz/~ml/weka/snapshots/stable-3-8.zip ./weka.zip
RUN unzip ./weka.zip -d ./weka-3-8
RUN rm ./weka.zip


## SETUP TENSORFLOW OBJECT RECOGNITION

RUN git clone https://github.com/tensorflow/models /opt/tensorflow-models

RUN pip install pillow
RUN pip install lxml

WORKDIR /opt/tensorflow-models/research
RUN protoc object_detection/protos/*.proto --python_out=/opt/tensorflow-models/research

# RUN echo 'export PYTHONPATH=$PYTHONPATH:/babyrobot-integration/:/opt/tensorflow-models/research:/tensorflow-models/research/slim/' >> /root/.bashrc
ENV PYTHONPATH=$PYTHONPATH:/babyrobot-integration/:/babyrobot-integration/babyrobot/:/opt/tensorflow-models/research:/tensorflow-models/research/slim/

RUN pip install http://download.pytorch.org/whl/cu80/torch-0.2.0.post3-cp27-cp27mu-manylinux1_x86_64.whl 
RUN pip install torchvision 

RUN cp /babyrobot-integration/dockerfiles/run_services.sh /bin/run_services
RUN chmod +x /bin/run_services

WORKDIR /babyrobot-integration
