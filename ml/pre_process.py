import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os
import glob
import random
import math

file_path = "../capture_bno085/data/*.dat"
file_index = 0

out_samples = 150


#                  i            j            k            r           aX           aY           aZ           gX           gY           gZ           rx           ry           rz
# min      -0.791140    -0.725280    -0.999940     0.000000   -21.003910   -27.644530   -15.917970   -12.785160    -6.890620    -8.861330    -1.218855    -1.171420    -2.541700
# max       0.528630     0.721860     0.999820     0.999940    27.953120     6.628910    24.125000    10.705080     9.562500     8.347660     1.711840     1.518310     1.513890

accl_min = -30.0
accl_max = 30.0
gyro_min = -15.0
gyro_max = 15.0
roto_min = -3.0
roto_max = 2.0

columns = ["i", "j", "k", "r", "aX", "aY", "aZ", "gX", "gY", "gZ", "rx", "ry", "rz"]
mindf = pd.DataFrame(columns=columns)
maxdf = pd.DataFrame(columns=columns)

datafiles = glob.glob(file_path)
datafiles.sort()
print("total files", len(datafiles))
for datafile in datafiles:
    out = []
    print("process ", datafile)
    df = pd.read_csv(datafile)
    df.drop(["lineno", "accurate", "time"], axis=1, inplace=True)
    df.drop(df.tail(5).index, inplace=True)
    if (len(df) ==0) :
        continue
    # print (df.shape)
    df["rx"] = (df["rx"] - df["rx"][0]).apply(lambda x: np.where(x > math.pi, x - math.pi*2, np.where(x < -math.pi, x + math.pi*2, x)))
    df["ry"] = df["ry"] - df["ry"][0]
    df["rz"] = df["rz"] - df["rz"][0]

    # print(df.describe())

    # To start with, take all samples

    print("Total points: ", len(df))
    if len(df) < 60:
        # too few points, ignore
        continue

    n = len(df) - out_samples
    if n > 0:
        # too many samples, drop them
        df.drop(df.tail(n).index, inplace=True)

    # print(df.min())
    mi = pd.DataFrame(df.min()).T
    ma = pd.DataFrame(df.max()).T
    mi.columns = columns
    ma.columns = columns
    mindf = pd.concat([mindf, mi], ignore_index=True)
    maxdf = pd.concat([maxdf, ma], ignore_index=True)

    # Add 0 to the end. This shall be done after get mindf/maxdf
    if n < 0:
        # too less samples
        zeros_data = {
            "i": [0] * -n,
            "j": [0] * -n,
            "k": [0] * -n,
            "r": [0] * -n,
            "aX": [accl_min] * -n,
            "aY": [accl_min] * -n,
            "aZ": [accl_min] * -n,
            "gX": [gyro_min] * -n,
            "gY": [gyro_min] * -n,
            "gZ": [gyro_min] * -n,
            "rx": [roto_min] * -n,
            "ry": [roto_min] * -n,
            "rz": [roto_min] * -n,
        }
        zeros_df = pd.DataFrame(zeros_data)

        df = pd.concat([df, zeros_df], ignore_index=True)
        
    saveto = "processed_" + datafile[18:]
    df.drop(["i", "j", "k","r"], axis=1, inplace=True)

    df['aX'] = (df['aX'] - accl_min) / (accl_max - accl_min)
    df['aY'] = (df['aY'] - accl_min) / (accl_max - accl_min)
    df['aZ'] = (df['aZ'] - accl_min) / (accl_max - accl_min)
    df['gX'] = (df['gX'] - gyro_min) / (gyro_max - gyro_min)
    df['gY'] = (df['gY'] - gyro_min) / (gyro_max - gyro_min)
    df['gZ'] = (df['gZ'] - gyro_min) / (gyro_max - gyro_min)
    df['rx'] = (df['rx'] - roto_min) / (roto_max - roto_min)
    df['ry'] = (df['ry'] - roto_min) / (roto_max - roto_min)
    df['rz'] = (df['rz'] - roto_min) / (roto_max - roto_min)

    df.to_csv(saveto, index=False, float_format="%.5f")

    pass

print(mindf.describe())
print(maxdf.describe())


pass