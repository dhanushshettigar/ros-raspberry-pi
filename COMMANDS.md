# Here's a cheat sheet for some of the most commonly used commands in a Bash terminal:

## Navigation:
* **pwd**: Print working directory
* **ls**: List files and directories
* **ls -l**: Detailed list
* **ls -a**: List all files, including hidden ones
* **cd**: Change directory
* **cd ..**: Move up one directory
* **cd ~**: Move to the home directory

## File and Directory Manipulation:
* **touch filename**: Create an empty file
* **mkdir directory**: Create a new directory
* **cp source destination**: Copy file/directory
* **cp -r source_directory destination**: Recursive copy
* **mv source destination**: Move/rename file or directory
* **rm filename**: Remove/delete file
* **rm -r directory**: Recursive removal (use with caution)
* **nano filename**: Open a text editor (Ctrl+X to exit, Y to save changes)

## File Viewing:
* **cat filename**: Display file content
* **more filename** or **less filename**: Paginate through file content
* **head filename**: Display the first few lines of a file
* **tail filename**: Display the last few lines of a file

## Searching:
* **grep pattern filename**: Search for a pattern in a file
* **grep -r pattern directory**: Recursive search
* **find directory -name filename**: Find files/directories by name
* **locate filename**: Quickly find the location of a file (requires an updated database)

## Process Management:
* **ps**: Display information about active processes
* **ps aux**: Detailed process list
* **kill process_id**: Terminate a process
* **kill -9 process_id**: Forcefully terminate a process

## System Information:
* **uname -a**: Display system information
* **df -h**: Display disk space usage
* **free -m**: Display memory usage

## User Management:
* **whoami**: Display the current username
* **who**: Display information about users who are currently logged in
* **sudo command**: Execute a command with superuser privileges
* **useradd username**: Add a new user
* **passwd username**: Change password for a user

## Package Management (apt for Debian-based systems):
* **sudo apt update**: Update package information
* **sudo apt upgrade**: Upgrade installed packages
* **sudo apt install package**: Install a new package
* **sudo apt remove package**: Remove a package
* **dpkg -l**: List installed packages

## Network:
* **ifconfig** or **ip addr**: Display network interfaces and their configurations
* **ping hostname**: Send ICMP echo request to check network connectivity
* **traceroute hostname**: Display the route packets take to reach a network host

## Compression and Archiving:
* **tar -czvf archive.tar.gz files**: Create a compressed tar archive
* **tar -xzvf archive.tar.gz**: Extract files from a tar archive