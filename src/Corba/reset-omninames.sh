#!/bin/sh

sudo /etc/init.d/omniorb4-nameserver stop
sudo rm /var/log/omniorb-nameserver.log
sudo rm /var/lib/omniorb/omninames-*.bak
sudo rm /var/lib/omniorb/omninames-*.log
sudo /etc/init.d/omniorb4-nameserver start
