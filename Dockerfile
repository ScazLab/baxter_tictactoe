FROM scazlab/human_robot_collaboration:master

RUN cd ~/ros_ws/src \
    && git clone https://github.com/scazlab/baxter_tictactoe.git
RUN cd ~/ros_ws/src \
    && wstool merge -y baxter_tictactoe/dependencies.rosinstall
RUN cd ~/ros_ws/src \
    && wstool up

USER root
RUN  cd ~/ros_ws \
     && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
USER ros
RUN  cd ~/ros_ws && catkin build

CMD ["/bin/bash"]
