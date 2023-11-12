#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import tensorflow as tf
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import glob
import random

# Load the TFLite model
interpreter = tf.lite.Interpreter(model_path='gesture_model.tflite')

# Allocate memory for the model
interpreter.allocate_tensors()

# Get input and output tensors
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()



# In[ ]:


input_details


# In[ ]:


t1 = pd.read_csv("processed_data/a_10.dat")
input_data = np.array(t1.values.ravel(), dtype=np.float32)
# Prepare input data for prediction (replace this with your data)
# input_data = ...  # Prepare your input data as a NumPy array
input_shape = input_details[0]['shape']  # Get the expected input shape
input_data = input_data.reshape(input_shape)
# Set the input tensor
interpreter.set_tensor(input_details[0]['index'], input_data)

# Run inference
interpreter.invoke()

# Get the output tensor
output_data = interpreter.get_tensor(output_details[0]['index'])

# The output_data now contains the model's predictions
# print("Predictions:", output_data)
# Format and print the output_data with 2 decimal places
# Round the values to 2 decimal places and convert to strings
formatted_output_data = np.round(output_data, 2).astype(str)

print("Predictions with 2 decimal places:", formatted_output_data)


# In[ ]:


file_path = "processed_data/*"

datafiles = glob.glob(file_path)
# datafiles.sort()
print("total files", len(datafiles))
GESTURES = "abcdefghijklmnopqrstuvwxyz"
count_correct = 0
count_incorrect = 0

for datafile in datafiles:

    t1 = pd.read_csv(datafile)
    input_data = np.array(t1.values.ravel(), dtype=np.float32)
    # Prepare input data for prediction (replace this with your data)
    # input_data = ...  # Prepare your input data as a NumPy array
    input_shape = input_details[0]['shape']  # Get the expected input shape
    input_data = input_data.reshape(input_shape)
    # Set the input tensor
    interpreter.set_tensor(input_details[0]['index'], input_data)

    # Run inference
    interpreter.invoke()

    # Get the output tensor
    output_data = interpreter.get_tensor(output_details[0]['index'])

    # The output_data now contains the model's predictions
    # print("Predictions:", output_data)
    # Format and print the output_data with 2 decimal places
    # Round the values to 2 decimal places and convert to strings
    formatted_output_data = np.round(output_data, 2).astype(str)

    # print("Predictions with 2 decimal places:", formatted_output_data)
    ch = '.'
    for i in range(len(GESTURES)) :
        if output_data[0][i] > 0.5:
            ch = GESTURES[i]
    if ch == datafile[15]:
        # print ("correct")
        count_correct += 1
        pass
    else:
        count_incorrect += 1
        print ("incorrect ",  datafile[15] , " => ", ch)
        formatted_array = [f"{element:.2f}" for element in output_data[0]]
        print(formatted_array)

print("total incorrect ", count_incorrect, ", correct ", count_correct)    

