export ZSH="$HOME/.oh-my-zsh"

ZSH_THEME="docker"

plugins=(git autojump)

source $ZSH/oh-my-zsh.sh

source /opt/ros/noetic/setup.zsh

alias zshconfig="vim ~/.zshrc"
alias ohmyzsh="vim ~/.oh-my-zsh"
alias wssetup=". ./devel/setup.zsh"
alias buildlimit="catkin build -j5 --mem-limit 70%"
