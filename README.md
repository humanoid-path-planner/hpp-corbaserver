# HPP-corbaserver

[![Building Status](https://travis-ci.org/humanoid-path-planner/hpp-corbaserver.svg?branch=master)](https://travis-ci.org/humanoid-path-planner/hpp-corbaserver)
[![Pipeline status](https://gitlab.laas.fr/humanoid-path-planner/hpp-corbaserver/badges/master/pipeline.svg)](https://gitlab.laas.fr/humanoid-path-planner/hpp-corbaserver/commits/master)
[![Coverage report](https://gitlab.laas.fr/humanoid-path-planner/hpp-corbaserver/badges/master/coverage.svg?job=doc-coverage)](https://gepettoweb.laas.fr/doc/humanoid-path-planner/hpp-corbaserver/master/coverage/)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/humanoid-path-planner/hpp-corbaserver/master.svg)](https://results.pre-commit.ci/latest/github/humanoid-path-planner/hpp-corbaserver)

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
