#!/bin/bash

echo "=========================="
echo "Starting App RLtesting for {APP_PRETTY_NAME}"


systemctl start RLtesting
systemctl start rosnodeChecker
