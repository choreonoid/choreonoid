#!/bin/sh
sudo /etc/init.d/omniorb4-nameserver stop
sudo rm /var/lib/omniorb/omninames-*
sudo /etc/init.d/omniorb4-nameserver start
