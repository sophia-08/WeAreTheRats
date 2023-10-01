import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os
import glob
import random

file_path = "data/*.dat"
file_index = 0


out_samples = 120



def min_max_scaling(column, min_val, max_val):
    scaled_column = (column - min_val) / (max_val - min_val)
    return scaled_column

def save_out(out, filename):

    min_a = min( out['aX'].min(), out['aY'].min(), out['aZ'].min())
    max_a = max( out['aX'].max(), out['aY'].max(), out['aZ'].max())
    min_g = min( out['gX'].min(), out['gY'].min(), out['gZ'].min())
    max_g = max( out['gX'].max(), out['gY'].max(), out['gZ'].max())

    print("Range of accel: ", min_a, "-", max_a, ", Range of gyro: ", min_g, "-", max_g)
    if out.shape[0] != out_samples:
        print("fix me, expect ", out_samples, "samples, but got ", out.shape[0])
        exit(1)
    # out = out[['aX','aY','aZ','gX', 'gY', 'gZ']]
    out = out.drop('lineno', axis=1)
    for col in ["aX", "aY", "aZ"]:
        out[col] = min_max_scaling(out[col], min_a, max_a )
    for col in ["gX", "gY", "gZ"]:
        out[col] = min_max_scaling(out[col], min_a, max_a )
    out.to_csv(filename, index=False, float_format='%.3f')



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
                # out.append((df.loc[i]+df.loc[i+1]))
                out.append(df.loc[i])
                i += 2  
                removed += 1 
                accumulated -= 1.0

                # there were cases to remove more than half points
                if accumulated >= 1:
                    i+=1
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

        # generate 20 more samples
        # for i in range(20):
        #     aSet = set()
        #     while len(aSet) < point_to_remove :
        #         aSet.add(random.randint(start, end))
        #     print(aSet)
        #     out = []
        #     for j in range(start, end+1):
        #         if not j in aSet:
        #             out.append(df.loc[j])
        #     out = pd.DataFrame (out)
        #     save_out(out, "processed_"+datafile+"_gen_"+str(i))
             

    else:
        for i in range(start, end+1):
            out.append(df.loc[i])
        out = pd.DataFrame (out)

        point_to_add =   out_samples-(end-start+1)
        for i in range (point_to_add): 
            out.loc[-1] = [0,0,0,0,0,0,0]
            out.index += 1         

        save_out(out, "processed_"+datafile)  

        # generate 20 more samples
        # for loop in range(20):
        #     point_to_add =   out_samples-(end-start+1)

        #     new_row = pd.Series({'lineno': 0, 'aX': 0, 'aY': 0, 'aZ': 0, 'gX': 0, 'gY': 0, 'gZ': 0, })
        #     insert_at_middle = random.randint(0, point_to_add//2)
        #     insert_at_back = point_to_add - insert_at_middle
        #     aSet = set()
        #     while len(aSet) < insert_at_middle :
        #         aSet.add(random.randint(start, end))
        #     print(aSet)            

        #     out = []

        #     for i in range(start, end+1):
        #         out.append(df.loc[i])
        #         if i in aSet:
        #             out.append(new_row)
        #     for i in range (insert_at_back): 
        #         out.append(new_row) 
        #     out = pd.DataFrame (out)
        #     save_out(out, "processed_"+datafile+"_gen_"+str(loop))

    

    # print(df)
    pass