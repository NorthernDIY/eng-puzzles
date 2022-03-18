# -*- coding: utf-8 -*-
"""
Created on Sat Mar  5 22:24:44 2022

@author: NIcolas Hince DELL
"""

import pickle
#import pandas as pd
p=[(0,0),(1,1),(2,2),(3,3),(4,4),(5,5),(6,6),(7,7),(8,8),(9,9)]
#p=pd.DataFrame(p)
i=range(10)
filename="picklefile"
with open(filename, 'wb') as fp:
    pickle.dump(p[0],fp)
    pickle.dump(p[1],fp)
filedata = open(filename, 'rb')        
pickleric = pickle.load(filedata)        