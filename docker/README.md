The single reviewed Docker context is packaged at `src/ros_robot_studio/data/runtime/`.
RuntimeManager supplies a live-verified amd64 digest and DISTRO. No host ROS packages are used.
For manual builds see DEVELOPMENT.md. Packages are verified using apt-cache before install;
`/opt/studio-packages.txt` records exact installed versions. See docs/VERIFICATION.md for evidence.
