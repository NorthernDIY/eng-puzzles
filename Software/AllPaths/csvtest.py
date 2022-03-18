# -*- coding: utf-8 -*-
"""
Created on Wed Mar  2 22:33:38 2022

@author: NIcolas Hince DELL
"""
import csv 
list_data1=[(1,1),(2,2),(3,3),(4,4),(5,5),(6,6),(7,7),(8,8),(9,9),(0,0),(1,1),(2,2)]
list_data2=[(1,2),(2,3),(3,4),(4,5),(5,6),(6,7),(7,8),(8,9),(9,9),(0,0),(1,1),(2,2)]
with open('ALLPATHS.csv', 'a', newline='') as f_object:  
    # Pass the CSV  file object to the writer() function
    writer_object = csv.writer(f_object)
    # Result - a writer object
    # Pass the data in the list as an argument into the writerow() function
    writer_object.writerow(list_data1)  
    # Close the file object
    f_object.close()


def write2CSV(str,ans):

    with open(str, 'r') as fr_object:
        reader_object = csv.reader(fr_object,)
        #print(reader_object)
        for col in reader_object:
            if col not in ans:
                add=True;
            else: add=False;    
        if add==True:
            with open(str, 'a', newline='') as fw_object:
                # Pass the CSV  file object to the writer() function
                writer_object = csv.writer(fw_object)
                writer_object.writerow(ans)  
                # Close the file object
                f_object.close()
            
            



write2CSV('ALLPATHS.csv',list_data2)