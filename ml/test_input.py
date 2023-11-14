import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import tensorflow as tf
import glob
import random

file_path = "processed_data/*"

print(f"TensorFlow version = {tf.__version__}\n")

# Set a fixed random seed value, for reproducibility, this will allow us to get
# the same random numbers each time the notebook is run
SEED = 1337
np.random.seed(SEED)
tf.random.set_seed(SEED)

# the list of gestures that data is available for
GESTURES = "abcdefghijklmnopqrstuvwxyz"

SAMPLES_PER_GESTURE = 150

NUM_GESTURES = 26

# create a one-hot encoded matrix that is used in the output
ONE_HOT_ENCODED_GESTURES = np.eye(NUM_GESTURES)

inputs = []
outputs = []

datafiles = glob.glob(file_path)
# datafiles.sort()
print("total files", len(datafiles))

for datafile in datafiles:
    out = []    

    tensor = pd.read_csv(datafile)
    print(tensor.head())
    reshaped_array = tensor.values.reshape(150, 3, 3).reshape(150,-1)
    tensor = pd.DataFrame(reshaped_array)
    print(tensor.head())

    # inputs.append(np.array(tensor.values.ravel()))
    inputs.append(tensor)

    gesture_index=0
    for i in range(NUM_GESTURES):
        if datafile[15] == GESTURES[i]:
            gesture_index = i
    output = ONE_HOT_ENCODED_GESTURES[gesture_index]
    outputs.append(output)
    # print ("processed ", datafile, "output=", GESTURES[gesture_index])
    
print("total ", len(inputs))
# convert the list to numpy arra
inputs = np.array(inputs)
outputs = np.array(outputs)

print ("input shape: ", inputs.shape, " output shape", outputs.shape)

print("Data set parsing and preparation complete.")