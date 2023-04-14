# GeoTIFF file transfer via ZeroMQ PUB-SUB

This proof-of-concept is designed to transfer a singular sample file (.tif or .txt) from one machine to another connected to the same network.

## Instructions:

1. Ensure that sub.py is the only file present in the sub folder; delete any other files so that we can verify that file transfer was successful.
2. In pub.py, set the fileName variable to the filename of the file you wish to transfer (e.g. sample.txt, sample.tif).

WORK IN PROGRESS
