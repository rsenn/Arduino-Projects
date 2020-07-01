clang-format () 
{ 
    for arg in "$@";
    do
        case "$arg" in 
            *.ino)
                mv -vf "$arg" "${arg%.ino}.cpp";
                f=${arg%.ino}.cpp
            ;;
            *)
                f=$arg
            ;;
        esac;
        command clang-format -style=file -i $f;
        mv -vf "$f" "$arg";
    done
}
