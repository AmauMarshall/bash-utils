# GLOBALS
NUM='^[0-9]+$'

## ALIAS
setalias() {
    if [ -n "$1" ] && [ -n "$2" ]; then
        echo "alias $1='$2'" >> ~/.bash_aliases 
        source ~/.bash_aliases
    fi
}
rmalias() {
    if [ -n "$1" ]; then
        sed -i "s/^alias $1=/#alias $1=/" ~/.bash_aliases 
    fi
}
dealias() {
    #echo -n "$1 >> "
    alias $1 | cut -d\' -f2
}
complete -a dealias
complete -a rmalias

### RENAME
renamepc() {
    sudo sed -i "s/$(hostname)/$1/" /etc/hosts /etc/hostname
    sudo hostname -b $1
    echo "Don't forget to restart to apply!"
}

### CD
alias cd..='cd ..'
alias cd.='cd /'
alias ..='cd ..'
alias ...='cd ../..'
alias ....='cd ../../..'
alias .....='cd ../../../..'
alias ......='cd ../../../../..'

### APT
alias aptup='sudo apt update && sudo apt upgrade -y && sudo apt autoremove -y'
alias aptcl='sudo apt autoremove -y && sudo apt clean -y && sudo apt autoclean -y'
alias aptin='sudo apt install -y'
alias aptrem='sudo apt autoremove -y'

### BASH
alias userls='cut -d: -f1 /etc/passwd'
alias hcl='rm -f ~/.bash_history && history -c && reset'
alias cls='rm -f ~/.bash_history && history -c && reset'
alias brc='currentBrc=$(pwd) && source ~/.bashrc && cd "$currentBrc" && unset currentBrc && [ -n "$VIRTUAL_ENV" ] && source $VIRTUAL_ENV/bin/activate || echo -n ""'
alias bashrc='vim ~/.bashrc && brc'
alias bashal='vim ~/.bash_aliases && brc'
alias profile='vim ~/.profile && source ~/.profile && brc'
alias vimrc='vim ~/.vimrc'
alias pyrc='vim ~/.pyrc'

### LS
alias ls='ls --color=auto'
alias ll='ls -lhF'
alias lll='ls -alhF'
alias la='ls -A'
alias l='ls -CF'
alias lsdev='ls /dev/*'
lsn() {
    if [ -z "$1" ]; then
        echo -n "$(pwd) "
    else
        echo -n "$(realpath $1) "
    fi
    ls -1 $1 | wc -l
}

### RM
alias rm*='rm *'

### GREP
alias grep='grep --color=auto -E'
alias ngrep='grep --color=auto -nE'
astrogrep () 
{ 
    find $1 ! -type d | while read -r line; do
        if [ $(grep -Enc $2 $line) -gt 0 ]; then
            echo $line;
            grep --color=auto -En $2 $line;
            echo;
        fi;
    done
}

### NET
lsip() {
    if [ -z "$1" ]; then
        ip addr list | grep -v "^[[:space:]]" | awk '{print $2}' | cut -d: -f1
    else
        if [[ $1 =~ $NUM ]]; then
            ip addr list | grep -v "^[[:space:]]" | awk '{print $2}' | cut -d: -f1 | sed -n $1p
        else
            ip addr list | grep -v "^[[:space:]]" | awk '{print $2}' | cut -d: -f1 | grep "^$1"
        fi
    fi
}
getip() {
    for net_dev in $(lsip $1); do
        echo -en "$net_dev:\r\033[20C"
        ip -4 addr show $net_dev | /usr/bin/grep -oP '(?<=inet\s)\d+(\.\d+){3}'
    done
}

### DOCKER
alias d='docker'
alias dc='docker-compose'
dcres() {
    if [ -n "$1" ]; then
        dc -f $1 down -v
        dc -f $1 up -d
        dc -f $1 logs -f
    else
        dc down -v
        dc up -d
        dc logs -f
    fi
}

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
    shift 2
    git sparse-checkout set $@
    git checkout
    cd $CURR_PATH
}
alias gc='git clone'
alias gsc='git-sparse-clone'

### RESET
alias x='exit'
alias r='brc && reset'
alias cdr='cd ~ && r'

### SSH
alias sshc='vim ~/.ssh/config'

### PYTHON
alias python='python3'
alias py='python3'
alias p='python3'

### CALC
=() {
    echo $(( $1 $2 $3 ))
}

### HISTORY
alias history='HISTTIMEFORMAT="%d/%m/%Y at %H:%M:%S - " history'
history-clear-from-or-last() {
    if [ -n $1 ] && [[ $1 =~ $NUM ]]; then
        count=50
        while [[ $count -gt 0 ]]; do
            history -d $1
            count=$(( $count - 1 ))
        done
    else
        history -d $(history | awk 'END{print $1-1}')
    fi
}

### MISCS
