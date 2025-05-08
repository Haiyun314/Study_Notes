# Command Line
- [File and Directory Management](#file-and-directory-management)
- [File Viewing and Editing](#file-viewing-and-editing)
- [Searching and Filtering](#searching-and-filtering)
- [Package Management](#package-management)
- [Process and Resource Monitoring](#process-and-resource-monitoring)
- [Networking](#networking)
- [Permissions and Ownership](#permissions-and-ownership)

## File and Directory Management
```bash
ls                # List files and directories
cd <dir>          # Change directory
pwd               # Show current directory path
mkdir <dir>       # Create a new directory
touch <file>      # Create a new empty file
cp <src> <dest>   # Copy files or directories
mv <src> <dest>   # Move or rename files or directories
rm <file>         # Remove file
rm -r <dir>       # Remove directory recursively
```

## File Viewing and Editing
```bash 
less <file>       # View file content page-by-page
head <file>       # Show first 10 lines
tail <file>       # Show last 10 lines
vim <file>        # Edit file using vim editor
    i : edit
    esc : exit editing
    :q : exit vim
    :wq : write and exit vim
```

## Searching and Filtering
```bash
grep "text" <file>         # Search for "text" in file
find <dir> -name "file"    # Find file by name
history                    # Show command history
```

## Package Management
```bash
sudo apt update                    # Update package list
sudo apt install <package>         # Install a package
sudo apt remove <package>          # Remove a package
```

## Process and Resource Monitoring
```bash
top                    # View running processes
htop                   # Enhanced process viewer (install first)
ps aux                 # List all running processes
kill <pid>             # Kill a process by PID
```

## Networking
```bash
ping <host>            # Ping a host
ifconfig               # Show network interfaces (older)
ip a                   # Show IP addresses (modern)
netstat -tulnp         # Show network ports in use
curl <url>             # Fetch content from a URL
```

## Permissions and Ownership
```bash
chmod +x <file>        # Make file executable
chown <user>:<group> <file>  # Change file ownership
```