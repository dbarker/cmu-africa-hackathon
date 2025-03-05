#!/bin/bash
set -e

if [ "$#" -ne 3 ]; then
    echo "Usage: $0 USER_NAME GID UID"
    exit 1
fi

USER_NAME=$1
USER_GID=$2
USER_UID=$3

if id "ubuntu" >/dev/null 2>&1; then
    userdel -r ubuntu
    echo "User 'ubuntu' removed."
else
    echo "User 'ubuntu' does not exist."
fi

# Check if the group exists, otherwise create it
if ! getent group "$USER_GID" >/dev/null; then
    groupadd -g "$USER_GID" "$USER_NAME"
    echo "Created group '$USER_NAME' with GID $USER_GID."
else
    echo "Group with GID $USER_GID exists, reusing it."
fi

# Create the user with specified UID and GID
useradd -m -u "$USER_UID" -g "$USER_GID" -s /bin/bash "$USER_NAME"
echo "Created user '$USER_NAME' with UID $USER_UID and GID $USER_GID."

# Grant passwordless sudo access
echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" | sudo tee "/etc/sudoers.d/$USER_NAME" >/dev/null

# Verify user and home directory
id "$USER_NAME"
ls -ld "/home/$USER_NAME"

echo "User '$USER_NAME' has passwordless sudo and a home directory."
  
echo "# Enable history search with up and down arrows" > /home/${USERNAME}/.inputrc 
echo "\"\e[A\": history-search-backward" >> /home/${USERNAME}/.inputrc 
echo "\"\e[B\": history-search-forward" >> /home/${USERNAME}/.inputrc 
chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.inputrc