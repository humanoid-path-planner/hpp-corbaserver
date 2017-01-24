Corba server for Humanoid Path Planner applications.

# Troubleshooting

## `CORBA::TRANSIENT` when launching a server

It very often happens that the OmniNames server failed to start properly at boot.

To check if the server is running, run:
```bash
ps -C omniNames -o pid,args
```

If the process is not running, delete omniNames related log and backup files in `/var/lib/omniorb`. They may have different names on your computer, but most likely, something like:
```bash
rm /var/lib/omniORB/omninames-<computer_name>.log
rm /var/lib/omniORB/omninames-<computer_name>.bak
```
then restart the server:
```bash
sudo service omniorb4-nameserver restart
```


