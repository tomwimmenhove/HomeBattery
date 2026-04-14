#!/bin/bash

curl -s -X POST 'http://localhost:8000/api/write' -H 'Content-Type: application/json' -d '{"unit":11, "addr": 2, "value_format": "linear16", "value": 28.75}'
curl -s -X POST 'http://localhost:8000/api/write' -H 'Content-Type: application/json' -d '{"unit":11, "addr": 3, "value_format": "linear16", "value": 28.8}'
curl -s -X POST 'http://localhost:8000/api/coils/write' -H 'Content-Type: application/json' -d '{"unit":11, "addr": 1, "value":1}'
