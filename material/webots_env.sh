#!/usr/bin/bash

# Usage: 
# - Source this script to set up the environment variables for Webots
#   > source webots_env.sh
# - Execute this script to add the environment variables to ~/.bashrc
#   > ./webots_env.sh

# Detect if this script is being sourced
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    # This script is being sourced, export the variables
    echo "Sourcing Webots environment variables..."

    export WEBOTS_HOME=/usr/local/webots
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$WEBOTS_HOME/lib/webots:$WEBOTS_HOME/lib/controller
    export PYTHONPATH=$PYTHONPATH:$WEBOTS_HOME/lib/controller/python
    export PATH=$PATH:$WEBOTS_HOME

    echo "Done. Webots environment variables have been set."
else
    # This script is being executed
    echo "Adding Webots environment variables to ~/.bashrc..."

    echo >> ~/.bashrc # empty line
    echo '### WEBOTS ENV ###' >> ~/.bashrc
    echo 'export WEBOTS_HOME=/usr/local/webots' >> ~/.bashrc # default webots install location
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$WEBOTS_HOME/lib/webots:$WEBOTS_HOME/lib/controller' >> ~/.bashrc
    echo 'export PYTHONPATH=$PYTHONPATH:$WEBOTS_HOME/lib/controller/python' >> ~/.bashrc
    echo 'export PATH=$PATH:$WEBOTS_HOME' >> ~/.bashrc

    echo "Done. Please restart your terminal or run 'source ~/.bashrc' to apply the changes."
fi
