#!/bin/bash
set -e

docker build -t testeng -f Dockerfile.openpilot .

docker run --rm \
  -v "$(pwd)"/selfdrive/test/tests/plant/out:/tmp/openpilot/selfdrive/test/tests/plant/out \
  testeng  /bin/sh -c 'cd /tmp/openpilot/selfdrive/test/tests/plant && OPTEST=1 ./test_longitudinal_te.py'
