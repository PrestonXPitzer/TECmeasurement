#Author: Preston Pitzer (Virginia Tech/Space@VT)
#Version 1.0 (1-18-24)
"""
Read the contents of a RINEX file and determine a vertical TEC value for each epoch.
"""
import pandas as pd 
import matplotlib.pyplot as plt 
import datetime as dt
import os 
import georinex as gr #for reading RINEX files

"""
given a gnssId and sigId, return the frequency of the signal
"""
def determineFrequency(gnssId, sigId):
    if gnssId == 0 and sigId == 0:
        return 1575.42e6 # L1C/A
    elif gnssId == 0 and sigId == 3:
        return 1227.6e6 # L2CL
    elif gnssId == 0 and sigId == 4:
        return 1227.6e6 # L2CM
    elif gnssId == 0 and sigId == 6:
        return 1176.45e6 # L5 I
    elif gnssId == 0 and sigId == 7:
        return 1176.45e6 #L5 Q
    elif gnssId == 1 and sigId == 0:
        return 1575.42e6 # SBAS L1C/A
    elif gnssId == 2 and sigId == 0:
        return 1575.42e6 # GALI E1 C
    elif gnssId == 2 and sigId == 1:
        return 1207.14e6 # E1 B
    elif gnssId == 2 and sigId == 3:
        return 1176.45e6 # E5a 
    elif gnssId == 2 and sigId == 4:
        return 1176.45e6 #E5a
    elif gnssId == 2 and sigId == 5:
        return 1207.14e6 # E5b
    elif gnssId == 2 and sigId == 6:
        return 1207.14e6 # E5b
    elif gnssId == 3 and sigId == 0:
        return 1561.091e6 # B1I D1
    elif gnssId == 3 and sigId == 1:
        return 1561.091e6 # B1I D2
    elif gnssId == 3 and sigId == 2:
        return 1207.14e6 # B2I D1
    elif gnssId == 3 and sigId == 3:
        return 1207.14e6 # B2I D2
    elif gnssId == 3 and sigId == 5:
        return 1575.42e6 # B1C
    elif gnssId == 3 and sigId == 7:
        return 1176.45e6 #B2a
    elif gnssId == 5 and sigId == 0:
        return 1575.42e6 # QZSS L1C/A 
    elif gnssId == 5 and sigId == 1:
        return 1575.42e6 # QZSS L1S
    elif gnssId == 5 and sigId == 4:
        return 1227.6e6 # QZSS L2 CM
    elif gnssId == 6 and sigId == 0:
        return 1598.0625e6 #GLONASS L1
    elif gnssId == 6 and sigId == 2:
        return 1242.9375e6 #GLONASS L2
    elif gnssId == 7 and sigId == 0:
        return 1176.45e6 #NAVIC L5

"""
take a filename for a rinex file and return a pandas dataframe with the accessible data
"""
def read_rinex(rinex_file):
    obs = gr.load(rinex_file) #read in RINEX file, handles any format / uses the entire time span
    #now convert to a pandas dataframe
    df = obs.to_dataframe()
    return df



