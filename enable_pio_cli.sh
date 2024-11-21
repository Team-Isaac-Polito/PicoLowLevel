#!/bin/bash
# This script is created to complete the setup of Platform.IO CLI tools, linking them from the Platform.IO environment

# Create the ~/.local/bin directory (if doesn't exist)
local_bin="$HOME/.local/bin"
if [ ! -e $local_bin ]; then
    mkdir $local_bin
    echo "$local_bin directory created"
fi

# Profile files for /bin/bash and /bin/zsh
if [ "$SHELL" = "/bin/bash" ]; then
    profile_file=~/.bash_profile
elif [ "$SHELL" = "/bin/zsh" ]; then
    profile_file=~/.zprofile   
fi

grep --quiet "export PATH=\$PATH:\$HOME/.local/bin" $profile_file

# Export PATH if not found in the file
if [ $? -eq 1 ]; then 
    echo "# Setting PATH for Platform.IO CLI tools" >> $profile_file
    echo -e "export PATH=\$PATH:\$HOME/.local/bin" >> $profile_file
fi

platformio_link=~/.local/bin/platformio
pio_link=~/.local/bin/pio
debug_link=~/.local/bin/piodebuggdb

# Creates the links (if don't exist)
if [ ! -e $platformio_link ]; then
    ln -s ~/.platformio/penv/bin/platformio $platformio_link
    echo "Link '$platformio_link' created"
fi

if [ ! -e $pio_link ]; then
    ln -s ~/.platformio/penv/bin/pio $pio_link
    echo "Link '$pio_link' created"
fi

if [ ! -e $debug_link ]; then
    ln -s ~/.platformio/penv/bin/piodebuggdb $debug_link
    echo "Link '$debug_link' created"
fi

exit 0