ARG BASE_IMAGE=ros:humble-ros-base-jammy
FROM ${BASE_IMAGE}
ARG DISTRO=humble
RUN test "$ROS_DISTRO" = "$DISTRO" && apt-get update && \
    apt-cache show ros-${DISTRO}-rosbridge-suite ros-${DISTRO}-xacro >/dev/null && \
    apt-get install -y --no-install-recommends ros-${DISTRO}-rosbridge-suite ros-${DISTRO}-xacro && \
    dpkg-query -W > /opt/studio-packages.txt && rm -rf /var/lib/apt/lists/*
COPY demo_nodes.py /opt/studio/demo_nodes.py
COPY entrypoint.sh /opt/studio/entrypoint.sh
RUN chmod 755 /opt/studio/entrypoint.sh && useradd --create-home --uid 10001 studio
USER studio
ENV HOME=/home/studio
EXPOSE 9090
HEALTHCHECK --interval=10s --timeout=3s --start-period=30s --retries=3 \
    CMD python3 -c "import socket; socket.create_connection(('127.0.0.1',9090),2).close()"
ENTRYPOINT ["/opt/studio/entrypoint.sh"]
