# `Advance Engineering Project`
![alt text](image.png)


# `Overall Design:`



### `ROS Tutorials:`
| `ROS Releases` | `Link` |
| -------------- | ------ |
| `18.0x(eloquent)` | https://docs.ros.org/en/eloquent/Installation/Linux-Install-Debians.html#resources |
| `20.0x(foxy)` | https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html#resources |
| `22.0x(humble)` | https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html#resources |



### `Setup ROS 2 Environment:` for 22.0x(humble)

**1. `Set locale`**
   
Make sure you have a locale which supports `UTF-8`.

``` bash
locale # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 export LANG=en_US.UTF-8

locale # verify setting
```

**2. `Setup sources`**

You will need to add the ROS 2 apt repository to your system.

First ensure that the [Ubuntu Universe repository][Ubuntu] is enabled.

``` bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Now add the ROS 2 GPG key with apt.

``` bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Then add the repository to your sources list.

``` bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```


**3. `Install ROS 2 packages`**

Update your apt repository caches after setting up the repositories.

``` bash
sudo apt update
```

ROS 2 packages are built on frequently updated Ubuntu systems. It is always recommended that you ensure your system is up to date before installing new packages.

``` bash
sudo apt upgrade
```

Desktop Install (`Recommended`): ROS, RViz, demos, tutorials.

``` bash
sudo apt install ros-humble-desktop
```

Development tools: Compilers and other tools to build ROS packages

``` bash
sudo apt install ros-dev-tools
```

**4. `Environment setup`**

Set up your environment by sourcing the following file.

``` bash
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash
```



### `Using colcon to build packages:` for 22.0x(humble)

**1. `Install colcon`**
| Operating System | Command |
| ---------------- | ------- |
| `Linux` | sudo apt install python3-colcon-common-extensions |
| `macOS` | python3 -m pip install colcon-common-extensions |
| `Windows` | pip install -U colcon-common-extensions |



### `Create a workspace:` for 22.0x(humble)




[Ubuntu]: https://help.ubuntu.com/community/Repositories/Ubuntu