#!/bin/bash

echo "=========================="
echo "Stopping App RLtesting"

systemctl stop rosnodeChecker
systemctl stop RLtesting
