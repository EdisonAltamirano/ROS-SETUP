# ROS-SETUP

## Requirements ###

- Docker Engine on Linux or Windows
- VScode

## VScode Extensions ###

- Remote - Containers
- Docker


## Steps in order to start working ###
## WINDOWS ##
First go to the directory where you cloned this repo. Then run the following commands

```
docker build -t tec ./
./create_container_windows.bash

```
## LINUX ##
First go to the directory where you cloned this repo. Then run the following commands

```
sudo docker login
sudo make tec.build 
```

#### Create container

```
chmod +x create_container_linux.bash
./create_container_linux.bash
```
## Windows or Linux

If you get the following error:
> Access control disabled, clients can connect from any host
docker: Error response from daemon: Conflict. The container name "/uuv" is already in use by container [...] You have to remove (or rename) that container to be able to reuse that name.

Just run these lines

```
sudo docker stop /tec
sudo docker rm /tec
```
And try again.


### To turn on the container (Do this step just ONCE)

```
sudo make tec.up
```

### If you want to use the terminal within the container

```
sudo make tec.shell
```

Now you can work accesing the following folder:

```
cd ws/src
```


## Vscode

After installing the extensions if you are having trouble accesing the container just open a terminal and run the following lines

```
sudo usermod -aG docker $USER
newgrp docker
sudo chmod o+rw /var/run/docker.sock
```

Steps to attach to vscode

- Select the container tec
- Right Click
- Select Attach Visual Studio Code

Happy Coding :)