## Appended from forg_bot/.zshrc
if [ -f $HOME/ros2_ws/install/local_setup.zsh ]; then
  . $HOME/ros2_ws/install/local_setup.zsh
  eval "$(register-python-argcomplete3 ros2)"
  eval "$(register-python-argcomplete3 colcon)"
fi
