PROMPT="%(?:%{$fg_bold[green]%}➜ :%{$fg_bold[red]%}➜ )"
if [ ! -z "$DOCKER_CONTAINER_NAME" ]
then
    PROMPT+=' %{$fg[green]%}(🐳%{$fg[blue]%}%$DOCKER_CONTAINER_NAME%{$fg[green]%})%{$reset_color%}'
fi
PROMPT+=' %{$fg[cyan]%}%c%{$reset_color%} $(git_prompt_info)'


ZSH_THEME_GIT_PROMPT_PREFIX="%{$fg_bold[blue]%}git:(%{$fg[red]%}"
ZSH_THEME_GIT_PROMPT_SUFFIX="%{$reset_color%} "
ZSH_THEME_GIT_PROMPT_DIRTY="%{$fg[blue]%}) %{$fg[yellow]%}✗"
ZSH_THEME_GIT_PROMPT_CLEAN="%{$fg[blue]%})"
