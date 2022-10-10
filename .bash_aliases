### ALIAS
setalias() {
    echo "alias $1='$2'" >> ~/.bash_aliases 
    source ~/.bash_aliases
}

### RENAME
renamepc() {
        sudo sed -i "s/$(hostname)/$1/" /etc/hosts /etc/hostname
        sudo hostname -b $1
        echo "Don't forget to restart to apply!"
}

### CD
alias cd..='cd ..'
alias ..='cd ..'
alias cd.='cd /'
alias x='exit'

### APT
alias aptup='sudo apt update && sudo apt upgrade -y && sudo apt autoremove -y'
alias aptcl='sudo apt autoremove -y && sudo apt clean -y && sudo apt autoclean -y'
alias aptin='sudo apt install -y'
alias aptrem='sudo apt autoremove -y'

### BASH
alias userls='cut -d: -f1 /etc/passwd'
alias hcl='rm -f ~/.bash_history && history -c && reset'
alias cls='rm -f ~/.bash_history && history -c && reset'
alias brc='source ~/.bashrc'
alias bashrc='vim ~/.bashrc && source ~/.bashrc'
alias bashal='vim ~/.bash_aliases && source ~/.bashrc'
alias vimrc='vim ~/.vimrc'
alias prompt='. ~/.bash_prompt'

### LS
alias ls='ls --color=auto'
alias ll='ls -lF'
alias lll='ls -alF'
alias la='ls -A'
alias l='ls -CF'
alias lsdev='ls /dev/*'

### GREP
alias grep='grep --color=auto -E'
alias ngrep='grep --color=auto -nE'

### NET
#alias reset-resolv='echo -e "nameserver 8.8.8.8\nsearch mobilisis.local" | sudo tee /etc/resolv.conf > /dev/null'
getip() {
    if [ -z "$1" ]; then
        ip -4 addr | /usr/bin/grep -oP '(?<=inet\s)\d+(\.\d+){3}'
    else
        ip -4 addr show $1 | /usr/bin/grep -oP '(?<=inet\s)\d+(\.\d+){3}'
    fi
}
lsip() {
     if [ -z "$1" ]; then
        ifconfig | /usr/bin/grep -v "^ " | /usr/bin/grep -v "^[[:space:]]*$" | cut -d' ' -f1
    else
        ifconfig | /usr/bin/grep ^$1 | cut -d' ' -f1
    fi
}

### DOCKER
alias dc='docker-compose'
alias dcres='dc down -v && dc up -d && dc logs -f'

### GIT
gitinit() {
    if [ -n $1 ]; then
        email="amarechal@mobilisis.hr"
    else
        email=$1
    fi
    if [ -n $2 ]; then
        name="amarechal"
    else
        name=$2
    fi
    git config --global user.email $email
    git config --global user.name $name
    git config --global credential.helper 'cache --timeout 3600'
}
git-sparse-clone() {
    CURR_PATH=$(pwd)
    mkdir $2
    git clone --no-checkout $1 $2
    cd $2
    git sparse-checkout set $3 $4 $5
    git checkout
    cd $CURR_PATH
}
alias gc='git clone'
alias gsc='git-sparse-clone'

### GST
#cam-stream() {
#    gst-launch-1.0 -v v4l2src device=/dev/video0 ! videoconvert ! vp8enc ! rtpvp8pay ! udpsink host=$1 port=$2
#}
#alias camstr='cam-stream'
#file-stream() {
#    gst-launch-1.0 -v filesrc location=$3 ! decodebin ! videoconvert ! vp8enc ! rtpvp8pay ! udpsink host=$1 port=$2
#}
#alias filestr='file-stream'
#alias kd='killall droidcam'
#alias cam-on='droidcam-cli 10.10.30.51 4747'

### VEGETA
#vegeta-attack() {
#    DATETIME=$(date +%F)_$(date +%T)
#    DIR_NAME="/vegeta/attack_$DATETIME"
#    mkdir "$DIR_NAME"
#    touch "$DIR_NAME/$(echo "$1" | sed "s/\//_/g")"
#    echo "$1" | vegeta attack -duration="$2"s -rate="$3" -timeout=60s -output="$DIR_NAME/attack_$DATETIME.log"
#    cat "$DIR_NAME/attack_$DATETIME.log" | vegeta encode -to=json -output="$DIR_NAME/attack_$DATETIME.json"
#    cat "$DIR_NAME/attack_$DATETIME.log" | vegeta encode -to=csv -output="$DIR_NAME/attack_$DATETIME.csv"
#    cat "$DIR_NAME/attack_$DATETIME.log" | vegeta report -type=text -output="$DIR_NAME/report_$DATETIME.log"
#    echo "$DIR_NAME/attack_$DATETIME.json"
#    echo "$DIR_NAME/attack_$DATETIME.csv"
#    echo "$DIR_NAME/report_$DATETIME.log"
#}
#alias vatt='vegeta-attack'
#decode64() {
#    cat $1 | cut -d',' -f7 | base64 --decode | sed -e "s/}/}\n/g"
#}
#vatt-cat() {
#    vegeta-attack "$1" "$2" "$3" > res.txt
#    cat $(cat res.txt | grep report)
#    decode64 $(cat res.txt | grep csv)
#    rm res.txt
#}
#alias cdvegeta='cd /vegeta/'

### ROS
source /opt/ros/noetic/setup.bash
source /home/$USER/moby/devel/setup.bash
alias rosgo='roslaunch rosbridge_server rosbridge_websocket.launch'
alias rtl='rostopic list'
alias rnl='rosnode list'
rosws() {
    if [ -z "$1" ]; then
        wscat -c "ws://0.0.0.0:9090"
    elif [ -z "$2" ]; then
        wscat -c "ws://$1:9090"
    else
        wscat -c "ws://$1:$2"
    fi
}
rospid() {
    ps aux | grep "rosbridge" | grep -v "grep" | awk '{print $2}'
}
roskill() {
        kill -9 $(rospid)
}

### PYTHON
alias python='python3'
alias py='python3'

### SSH
sshdis() {
    ssh $1 -L 5900:localhost:5900 'x11vnc -localhost -display :0 -forever'
}

### VNC
initVcXsrv() {
    current=$pwd
    cd /mnt/c/users/$USER/Documents
    cmd.exe /c taskkill /IM vcxsrv.exe /F
    cmd.exe /c config.xlaunch
    cd $current
    unset current
    export DISPLAY="`grep nameserver /etc/resolv.conf | sed 's/nameserver //'`:0"
}
initVcXsrv > /dev/null 2>&1
alias win='DISPLAY=10.10.10.232:0'

### WSL
alias c:='cd /mnt/c/users/$USER'
alias wsloff='cmd.exe /c wsl --shutdown'
xp() {
    if [ -z "$1" ]; then
        explorer.exe .
    else
       explorer.exe $1
    fi
}
switch_default_terminal() {
    settingsPath="/mnt/c/users/$USER/AppData/Local/Packages/Microsoft.WindowsTerminal_8wekyb3d8bbwe/LocalState/settings.json"
    greper="grep guid $settingsPath"

    if [ $($greper -c) -ge $1 ]; then
        uuid=$($greper | cut -d{ -f2 | cut -d} -f1 | head -n$1 | tail -n1)
        sed -i "s/\"defaultProfile\": \"{.*}\"/\"defaultProfile\": \"{$uuid}\"/" $settingsPath
    else
        echo -e "\033[1;31mERROR:\033[0;31m cannot switch to terminal profile n°$1 because it does not exist.\033[0m"
    fi
}
alias term='switch_default_terminal'

### FUN
pnis() {
    ascii-image-converter /usr/share/welcome.png -cCx
}
alias pp='pnis'