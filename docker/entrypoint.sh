#!/bin/bash
 
set -e

echo "==============3D BBS Docker Env Ready================"

cd /root/workspace

exec "$@"
