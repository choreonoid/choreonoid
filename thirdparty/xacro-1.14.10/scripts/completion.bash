function _file_arg()
{
  # search backwards for an existing filename argument
  for (( i=${cword}-1 ; i > 0 ; i-- )) ; do
    if [[ ${words[i]} != -* ]] && [ -e ${words[i]} ] ; then
      echo ${words[i]}
      return
    fi
  done
}

function _complete_xacro {
    local cur prev words cword
    _init_completion || return # this handles default completion (variables, redirection)

    if [[ ${cur} =~ \-.* ]]; then
        COMPREPLY+=($(compgen -W "--help --legacy --inorder --check-order --deps --includes --xacro-ns -q -v --verbosity=" -- ${cur}))
        [[ $COMPREPLY == *= ]] && compopt -o nospace
    else
        local FILE=$(_file_arg)
        if [[ $FILE == "" ]]; then
            compopt -o filenames 2>/dev/null
            COMPREPLY+=($(compgen -o plusdirs -f -X "!*.xacro" -- ${cur}))
        else
            # search for <xacro:arg> tags and list corresponding argument names
            local AWK=$(which awk)
            if [[ -x ${AWK} ]]; then
                local _xacro_args=$(${AWK} '{ match($0, /<(xacro:)?arg.*name="([^"]*)"/, results); if(results[2] != "") print results[2] ":="}' $FILE 2> /dev/null)
                # awk should be very silent about errors and return 0 on success
                if [[ $? == 0 ]]; then
                    compopt -o nospace 2>/dev/null
                    COMPREPLY+=($(compgen -W "${_xacro_args}" -- "${cur}"))
                fi
            fi
        fi
    fi
}

complete -F "_complete_xacro" "xacro"
