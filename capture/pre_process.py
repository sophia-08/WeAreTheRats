import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os
import glob

file_path = "data/*.dat"
file_index = 0


out_samples = 150

def save_out(out, filename):
    # print("removed=", removed, "size of out= ", out.shape)
    if out.shape[0] != out_samples:
        print("fix me, expect ", out_samples, "samples, but got ", out.shape[0])
        exit(1)
    # out = out[['aX','aY','aZ','gX', 'gY', 'gZ']]
    out = out.drop('lineno', axis=1)
    out.to_csv(filename, index=False)



datafiles = glob.glob(file_path)
datafiles.sort()
print(datafiles)
for datafile in datafiles:
    out = []    
    print ("process ", datafile)
    df = pd.read_csv(datafile)
    # print (df.shape)

    # To start with, take all samples
    start = 0
    end = df.shape[0]-1

    # trim begin
    # for i in range(20) :
    #     sumA = abs (df['aX'][i]) + abs (df['aY'][i]) +abs (df['aZ'][i])
    #     sumG = abs (df['gX'][i]) + abs (df['gY'][i]) +abs (df['gZ'][i])
    #     print ("begin byte ", i, ", acc=", sumA, ", gyro=", sumG)

    while (True) :
        count = 0
        for i in range(3):
            if abs (df['aX'][i+start]) + abs (df['aY'][i+start]) +abs (df['aZ'][i+start]) > 3:
                count += 1
        if count >= 2:
            break
        else:
            start += 1

    # print ("Start point: ", start)

    # Trim end
    while (True) :
        count = 0
        for i in range(3):
            if abs (df['aX'][end-i]) + abs (df['aY'][end-i]) +abs (df['aZ'][end-i]) > 3:
                count += 1
        if count >= 2:
            break
        else:
            end -= 1
    # print ("End point: ", end)
    print ("Total points: ", len(df), " Trimed ", start, " points at begin, ", df.shape[0]-1-end, " points at end.  Remaining points ", end-start+1 )
    if end-start+1 > out_samples :


        point_to_remove = end-start+1 - out_samples
        decimate =  float(point_to_remove) / float(out_samples) 

        print ("To remove ", point_to_remove, " points, 1 every ", decimate)

        
        i = start 
        removed = 0
        accumulated = 0.0
        while True:
            # print("process ", i)
            accumulated += decimate
            if (accumulated >= 1) :
                # print("remove ", i)               
                out.append((df.loc[i]+df.loc[i+1]))
                i += 2  
                removed += 1 
                accumulated -= 1.0
        
            else:
                out.append(df.loc[i])
                i += 1

            if i >= end:
                break
        if len(out) < out_samples: 
            print("add last point", i)
            out.append(df.loc[i])   
        out = pd.DataFrame (out)
        save_out(out, "processed_"+datafile)  
    else:
        for i in range(start, end+1):
            out.append(df.loc[i])
        out = pd.DataFrame (out)

        point_to_add =   out_samples-(end-start+1)
        for i in range (point_to_add): 
            out.loc[-1] = [0,0,0,0,0,0,0]
            out.index += 1         

        save_out(out, "processed_"+datafile)  


    

    # print(df)
    pass