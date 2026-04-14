#!/bin/bash

../inverter/inverter_controller.py --url http://pow.localdomain:8000/stream -H 0 | ../inverter/gtn_send_stream.py &
