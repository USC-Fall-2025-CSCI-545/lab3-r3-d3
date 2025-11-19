[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/JB8SDF9g)
# LAB3
Note:

The demo script `adarrt.py` is written for Python 2 and is provided for reference only.

In Lab 3, both Python 2 and Python 3 versions of the script are already included in the Docker images provided (as referenced in the Overleaf file), depending on which version you choose to use.

The Lab 3 simulation supports both Python 2 and Python 3 implementations; however, the real-world lab only supports Python 2.


sudo docker run -it \
    -v /etc/localtime:/etc/localtime:ro \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    --shm-size 8g \
    -e GDK_SCALE \
    -e GDK_DPI_SCALE \
    --network host \
    --ipc=host \
    -v /media:/media \
    -v /data:/data \
    -v /home:/home \
    --user root \
    --name lab3 \
    cs545-lab3-melodic:latest