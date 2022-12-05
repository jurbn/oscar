# TODO: move to ~/.local/bin/
import os
import logging

path = r"~/oscar/odometry/"
for file_name in os.listdir(path):
    file = path + file_name
    if os.path.isfile(file):
        logginginfo('Deleting file:', file)
        os.remove(file)
path = 
