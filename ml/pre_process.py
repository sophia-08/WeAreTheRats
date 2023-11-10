import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os
import glob
import random

file_path = "../capture_bno085/data/*.dat"
file_index = 0

out_samples = 150


def min_max_scaling(column, min_val, max_val):
    scaled_column = (column - min_val) / (max_val - min_val)
    return scaled_column


def save_out(out, filename):
    min_a = min(out["aX"].min(), out["aY"].min(), out["aZ"].min())
    max_a = max(out["aX"].max(), out["aY"].max(), out["aZ"].max())
    min_g = min(out["gX"].min(), out["gY"].min(), out["gZ"].min())
    max_g = max(out["gX"].max(), out["gY"].max(), out["gZ"].max())

    print("Range of accel: ", min_a, "-", max_a, ", Range of gyro: ", min_g, "-", max_g)
    if out.shape[0] != out_samples:
        print("fix me, expect ", out_samples, "samples, but got ", out.shape[0])
        exit(1)
    # out = out[['aX','aY','aZ','gX', 'gY', 'gZ']]
    out = out.drop("lineno", axis=1)
    for col in ["aX", "aY", "aZ"]:
        out[col] = min_max_scaling(out[col], min_a, max_a)
    for col in ["gX", "gY", "gZ"]:
        out[col] = min_max_scaling(out[col], min_a, max_a)
    out.to_csv(filename, index=False, float_format="%.4f")


columns = ["i", "j", "k", "r", "aX", "aY", "aZ", "gX", "gY", "gZ", "rx", "ry", "rx"]
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
    df["rx"] = df["rx"] - df["rx"][0]
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
    if n < 0:
        # too less samples
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

    # print(df.min())
    mi = pd.DataFrame(df.min()).T
    ma = pd.DataFrame(df.max()).T
    mi.columns = columns
    ma.columns = columns
    mindf = pd.concat([mindf, mi], ignore_index=True)
    maxdf = pd.concat([maxdf, ma], ignore_index=True)

    saveto = "processed_" + datafile[18:]
    df.drop(["i", "j", "k","r"], axis=1, inplace=True)
    df.to_csv(saveto, index=False, float_format="%.5f")

    pass
