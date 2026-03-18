#!/bin/bash

start=$(date -u -d "yesterday 23:00" +"%Y-%m-%dT%H:%M:%S.000Z")
end=$(date -u -d "tomorrow" +"%Y-%m-%dT%H:%M:%S.000Z")

curl -s "https://api.anwb.nl/energy/energy-services/v2/tarieven/electricity?startDate=$start&endDate=$end&interval=HOUR"
