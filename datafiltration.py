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
def plotData(time, filteredData):
    plt.plot(time, filteredData)
    plt.show()

#main 

def main():
    data = readData('data.csv')
    data = convertToDatetime(data)
    data = convertToTECU(data)
    filteredData = filterTEC(data)
    time = extractTime(data)
    plotData(time, filteredData)

if __name__ == "__main__":
    main()




