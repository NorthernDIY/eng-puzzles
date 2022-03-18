# -*- coding: utf-8 -*-
"""
Created on Thu Mar  3 01:23:03 2022
    https://stackoverflow.com/questions/40855221/how-to-check-if-row-already-exists-in-csv
@author: NIcolas Hince DELL
"""
import csv

def DoubleCheckRepeats():
    with open('ALLPATHS1.csv','r') as file1:
        existingLines = [line for line in csv.reader(file1, delimiter=',')]
    
    newSols = []
    uniqueSols = []
    with open('ALLPATHS.csv','r') as file2:
        reader2 = csv.reader(file2,delimiter=',')
        for row in reader2:
            if row not in newSols and row not in existingLines:
                newSols.append(row)
            elif row in newSols or row in existingLines:
                uniqueSols.append(row[0])
        if len(newSols)!=0:
            with open('ALLPATHS1.csv','a') as file1w:
                # Pass the CSV  file object to the writer() function
                writer_object = csv.writer(file1w)
                writer_object.writerow(row[0])
                file1w.close()
            result="new path found, path added to ALLPATHS1"
        else:
            result= None
    file1.close()
    file2.close()
    return result

DoubleCheckRepeats()