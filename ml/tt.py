import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os
import glob
import random

file_path = "../capture_bno085/data/*.dat"
file_index = 0

out_samples = 150

#                  i            j            k            r           aX           aY           aZ           gX           gY           gZ           rx           ry           rx
# min      -0.791140    -0.725280    -0.999940     0.000000   -21.003910   -27.644530   -15.917970   -12.785160    -6.890620    -8.861330    -6.264910    -1.171420    -2.541700
# max       0.528630     0.721860     0.999820     0.999940    27.953120     6.628910    24.125000    10.705080     9.562500     8.347660     6.264200     1.518310     1.513890



columns = ["i", "j", "k", "r", "aX", "aY", "aZ", "gX", "gY", "gZ", "rx", "ry", "rz"]
df = pd.DataFrame(columns=columns)
# maxdf = pd.DataFrame(columns=columns)

n=-5
zeros_data = {
    "i": [0] * -n,
    "j": [0] * -n,
    "k": [0] * -n,
    "r": [0] * -n,
    "aX": [0] * -n,
    "aY": [0] * -n,
    "aZ": [0] * -n,
    "gX": [0] * -n,
    "gY": [0] * -n,
    "gZ": [0] * -n,
    "rx": [0] * -n,
    "ry": [0] * -n,
    "rz": [0] * -n,
}
zeros_df = pd.DataFrame(zeros_data)

df = pd.concat([df, zeros_df], ignore_index=True)

df['rx'] = [-3.1,3.1,-1.0,1.3,0]
print(df)
df = df.apply(lambda x: np.where(x > 3, x - 6, np.where(x < -3, x + 6, x)))
print(df)
pass