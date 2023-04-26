# GeoTIFF file transfer via ZeroMQ PUB-SUB

This OS-agnostic proof-of-concept is designed to transfer a singular sample file (.tif or .txt) from one machine to another connected to the same network.

## Prerequisites:
1. Ensure you have the following installed:
- Latest version of Python 3
- ZeroMQ (pip install pyzmq)
- tqdm (pip install tqdm)

2. Confirm that both your sender and receiver are connected to the same Wi-Fi/LAN network.

## Instructions:

1. Ensure that sub.py is the only file present in the **sub** folder; delete any other files so that we can verify that file transfer was successful.
2. In pub.py, set the fileName variable to the filename of the file you wish to transfer (e.g. sample.txt, sample.tif).
3. Set serverAddress in pub.py to the IP address of the recipient device.
4. Run sub.py.
5. Run pub.py.

## Issues:
- As of now, files do not transfer completely; they stop at around the 6-8% progress mark. Currently investigating further.
