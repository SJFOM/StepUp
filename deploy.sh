#!/bin/bash
pio run --target=upload --upload-port=/dev/tty.usbserial-1410
pio device monitor --echo --port=/dev/tty.usbserial-1410
