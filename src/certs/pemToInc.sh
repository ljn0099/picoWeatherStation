#!/bin/bash

set -e

input=$1
out=$2

echo -n \#define MQTT_TLS_ROOT_CERT \" > $out
cat $input | awk '{printf "%s\\n\\\n", $0}' >> $out
echo "\"" >> $out
