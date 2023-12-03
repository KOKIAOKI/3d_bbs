# How to start 3D_BBS with Docker

Docker makes it easy to configure the configuration required to run 3D_BBS.  
Before you start 3D_BBS with docker, you should install [Docker](https://www.docker.com/) and [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-container-toolkit) in your PC.  
Also, to install CUDA 12.0 from inside docker, the **NVIDIA driver version must be 525.60.11 or higher.**

## 1. Make 3D_BBS Docker image  

You could also make docker image directly with provieded Dockerfile.  

Move the terminal path to `/docker` and execute the following command.  

```
cd docker
```
```
docker build -t 3d_bbs:latest .
```

`3d_bbs:latest` is just example of this docker image, you can replace it with the image name you want.  

After the image is created, you can execute `docker images` command to view the following results from the terminal.

```
REPOSITORY             TAG               IMAGE ID       CREATED             SIZE
3d_bbs                 latest            ece4f57ca14b   48 minutes ago      13.3GB
```

### Docker image information  
||Version|  
|:---:|:---:|  
|Ubuntu|22.04|    
|NVIDIA driver|525.60.11|
|CUDA|12.0|    
|Cudnn|8.0|    
|Cmake|3.27.9| 
|Eigen|3.4.0|      
|PCL|1.12.1| 

## 2. Make 3D_BBS docker container 

When you create a docker container, you need several options to use the GUI and share folders.  

To make this easier to manage, you can use the `container_run.sh` file.  
`container_run.sh` contains various container options and GUI settings.  

You can create your own docker container with the following command.

```
sudo chmod -R 777 container_run.sh
```
```
./container_run.sh <container_name> <image_name:tag>
```

:warning: **You should change {container_name}, {docker image} to suit your environment.**  

For example,  
```
 ./container_run.sh 3d_bbs_container 3d_bbs:latest
```

If you have successfully created the docker container, the terminal output will be similar to the below.  

```
==============3D BBS Docker Env Ready================
root@multirobot1-CILAB:~/workspace#
```  


If the docker volume sharing went well, the `ls` command should result in something like the following.
```
root@multirobot1-CILAB:~/workspace# ls
CMakeLists.txt  LICENSE  README.md  bbs3d  build  docker  figs  test  test_data
```

:warning: **You'll need to set the test_data folder as above to make it work for you. Also, note that the absolute path in the config file will follow the absolute path inside the docker container!**   

For example of `test.yaml`,
```
################[Necessary]################
## Folder Paths
target_clouds: "/root/workspace/test_data/target"
source_clouds: "/root/workspace/test_data/source"
output_folder: "/root/workspace/test_data/output"
```

### Setting up the environment is done! Follow the readme to build 3d_bbs.  

Exit from docker container
``` 
root@multirobot1-CILAB:~/workspace# exit
``` 
Start docker 
``` 
docker start <container_name>
docker attach <container_name>
``` 
For example, 
``` 
docker start 3d_bbs_container
docker attach 3d_bbs_container
```