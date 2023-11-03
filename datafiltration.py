#read in the values from a csv file with two columns, one contains a datetime object and the other contains a TEC value

import csv
import datetime
import matplotlib.pyplot as plt

#read the csv
def readData(filename):
    with open(filename, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        data = []
        for row in reader:
            data.append(row)
    return data

#convert the datetime strings to datetime objects
def convertToDatetime(data):
    for row in data:
        date = row[0]
        ymd = date.split(' ')[0]
        hms = date.split(' ')[1]
        year = int(ymd.split('-')[0])
        month = int(ymd.split('-')[1])
        day = int(ymd.split('-')[2])
        hour = int(hms.split(':')[0])
        minute = int(hms.split(':')[1])
        #the second value has a decimal point, keep only the integer part
        second = int(float(hms.split(':')[2]))
        row[0] = datetime.datetime(year, month, day, hour, minute, second)
    return data

#writer.writerow(["Time", "Data", "SV", "psuedo-1", "pseudo-2", "gnssid-1", "gnssid-2"])
#grab the SVID
def extractSVID(data_in):
    svids = [] 
    times = []
    data = []

    for row in data_in:
        svids.append(int(row[2]))
        times.append(row[0])
        data.append(row[1])
    superlist = []
    superlength = 0
    if len(svids) > 0:
        superlength = int(max(svids))
    for i in range(superlength):
        superlist.append([[],[]])
    rolling_index = 0
    for sv in svids:
        superlist[sv-1][0].append(times[rolling_index])
        superlist[sv-1][1].append(data[rolling_index])
        rolling_index += 1
    

        # for my brain, each index in superlist corresponds to a value of svid
        # for each of these indecies, there are two lists, 0 is time, 1 is data

        #once the threads are finished, create a plot of the data and display/save it
        # create a stack plot, where each line is a different svid

        #copy the non-empty lists into a new list
    newlist = []
    for i in range(len(superlist)):
        if len(superlist[i][0]) > 0:
            newlist.append(superlist[i])
    return newlist
#Convert the TEC values to TECU
def convertToTECU(data):
    for row in data:
        row[1] = float(row[1])/10**16
    return data

#filter out any values greater than 5 TECU
def filterTEC(data):
    filteredData = []
    for row in data:
        if row[1] < 5:
            filteredData.append(row[1])
    return filteredData

def extractTime(data):
    time = []
    for row in data:
        time.append(row[0])
    return time

#plot the data
def plotData(newlist):
    for i in range(len(newlist)):
        plt.plot(newlist[i][0], (newlist[i][1]), label = 'SVID ' + str(i+1))
    plt.xlabel('Time')
    plt.ylabel('TEC Units')
    plt.title('TEC vs Time')
    #limit the y axis to 0-100 TECU
    
    plt.legend()
    plt.show()
def filternewlist(newlist):
    for i in range(len(newlist)):
        for j in range(len(newlist[i][1])):
            newlist[i][1][j] = float(newlist[i][1][j])/10**18
    return newlist
#main 

def main():
    data = readData('multi-data.csv')
    data = convertToDatetime(data)
    newlist = extractSVID(data)
    newlist = filternewlist(newlist)
    plotData(newlist)


if __name__ == "__main__":
    main()




