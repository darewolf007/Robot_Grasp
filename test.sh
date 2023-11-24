#!/bin/bash
#source activate py27
#source ~/anaconda3/etc/profile.d/conda.sh
source /home/haowen/anaconda3/bin/activate RL
# 打开一个新终端并启动 Conda 环境，保持终端打开
gnome-terminal  --tab --working-directory=/home -- bash -c 'conda run -n RL python; exec bash' &
#xterm &

# 在另一个终端中执行更多命令，保持终端打开
#gnome-terminal --tab -- bash -c 'echo More commands; exec bash' &
