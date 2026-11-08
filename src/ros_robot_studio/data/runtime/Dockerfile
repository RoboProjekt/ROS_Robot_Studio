ARG BASE_IMAGE=ros:humble-ros-base-jammy
FROM ${BASE_IMAGE}
ARG DISTRO=humble
RUN test "$ROS_DISTRO" = "$DISTRO" && apt-get update && \
    apt-cache show ros-${DISTRO}-rosbridge-suite ros-${DISTRO}-xacro >/dev/null && \
    apt-get install -y --no-install-recommends ros-${DISTRO}-rosbridge-suite ros-${DISTRO}-xacro ros-${DISTRO}-control-msgs ros-${DISTRO}-trajectory-msgs ros-${DISTRO}-tf2-ros ros-${DISTRO}-tf2-msgs ros-${DISTRO}-diagnostic-msgs ros-${DISTRO}-rmw-fastrtps-cpp python3-numpy python3-defusedxml && \
    dpkg-query -W > /opt/studio-packages.txt && rm -rf /var/lib/apt/lists/*
COPY shared/ /opt/studio/
COPY controller_nodes.py /opt/studio/controller_nodes.py
COPY external_adapter.py /opt/studio/external_adapter.py
COPY fastdds_udp.xml /opt/studio/fastdds_udp.xml
COPY fastdds_localhost.xml /opt/studio/fastdds_localhost.xml
COPY demo_nodes.py /opt/studio/demo_nodes.py
COPY entrypoint.sh /opt/studio/entrypoint.sh
RUN chmod 755 /opt/studio/entrypoint.sh && useradd --create-home --uid 10001 studio
USER studio
ENV HOME=/home/studio
ENV PYTHONPATH=/opt/studio
EXPOSE 9090
HEALTHCHECK --interval=10s --timeout=3s --start-period=30s --retries=3 \
    CMD python3 -c "import os,socket; socket.create_connection(('127.0.0.1',int(os.environ.get('STUDIO_BRIDGE_PORT','9090'))),2).close()"
ENTRYPOINT ["/opt/studio/entrypoint.sh"]
