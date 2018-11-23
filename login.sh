#!/bin/bash


# ssh ros@turtle04 mkdir -p .ssh
# cat ~/.ssh/id_rsa.pub | ssh ros@turtle06 'cat >> .ssh/authorized_keys'

scp -r ./src/* ros@turtle01:~/ros_ws/src/b3m33aro/scripts/
ssh -X ros@turtle01
