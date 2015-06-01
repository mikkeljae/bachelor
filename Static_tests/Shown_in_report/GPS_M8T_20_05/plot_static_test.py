# parameters
csv_file = 'test_nmea_20_05_clean.txt'		#Name of input file
max_lines = 2000000			#Maximum number of files in input file

earth_radius = 6378137 			# WGS-84 equatorial radius

c_mew = 0.2				#Custom marker width

outlier_threshold = 0.03			#Threshold i meters. if == 0, no outliers rm 
midpoint_calculation_solution = 4	#Base midpoint calculation on this solution
alt_mean_calculation_solution = 4	#Base altitude mean calculation on this solu
map_plot_solution = 4			#Choose solution plotted in the map plot
histogram_plot_solution = 4		#Choose solution plotted in histogram
alt_plot_solution = 4			#Choose solution plotted in alt plot
alt_hist_plot_solution = 4		#Choose solution plotted in alt histogram
utm_error_plot_solution = 4
					#All = 0, Single = 1, DGNSS = 2, Float = 5,
					#Fix = 4, Float&Fix = 6


# load modules
import numpy as np
import csv
import matplotlib.pyplot as plt

from math import *
from numpy import *
from pylab import *

# GEOGRAPHIC MIDPOINT ########################################################
# This method calculates the center of mass for all points in a list.
# This method does not handle measurement uncertainty.
#
#    Convert each coordinate latitude and longitude to Cartesian coordinates:
#        x1 = cos(lat1) * cos(lon1)
#        y1 = cos(lat1) * sin(lon1)
#        z1 = sin(lat1)
#    Average x, y and z coordinates:
#        x = (x1 + x2 + ... xn) / n
#        y = (y1 + y2 + ... yn) / n
#        z = (z1 + z2 + ... yn) / n
#    Convert averaged coordinate to latitude and longitude:
#        h = sqrt(x^2 + y^2)
#        lat_midpt = atan2(z, h)
#        lon_midpt = atan2(y, x)

def geographic_midpoint (latlons):
    x = 0
    y = 0
    z = 0
    lat_radian = 0 
    lon_radion = 0
    n = len(latlons)
    for i in range(n):
	lat_radian = radians(latlons[i][0])
	lon_radian = radians(latlons[i][1])	
        x += (cos(lat_radian) * cos(lon_radian))
        y += (cos(lat_radian) * sin(lon_radian))
        z += sin(lat_radian)
    x = x/n
    y = y/n
    z = z/n
    h = sqrt(x**2 + y**2)
    return [degrees(atan2(z, h)), degrees(atan2(y, x))]

## GPS Import Function ###
def gps_import (filename, max_lines):
    file = open(filename, 'r')
    data = csv.reader(file, delimiter=',')
    i=0
    gps = []
    alt_array = []
    for time, lat_full, temp , lon_full, temp1, fix, sat, hdop, alt, temp2, temp3, temp5, temp6, checksum in data:
	#print lat_full
	#print lon_full

	lat_full_fl = float(lat_full)
	lon_full_fl = float(lon_full)
	lat_deg = floor(lat_full_fl/100)
	lon_deg = floor(lon_full_fl/100)
	
	lat_min = lat_full_fl-lat_deg*100
	lon_min = lon_full_fl-lon_deg*100
	
	lat_deg += lat_min/60
	lon_deg += lon_min/60
			
	#print lat_deg
	#print lon_deg
	
	(znum, zlet, utm_n, utm_e) = ll2utm (lat_deg, lon_deg)

	
        #if int(fix) > 0:
	if 1:
		gps.append([]) 
		gps[i].append (int(fix))
		gps[i].append (float(lat_deg))
		gps[i].append (float(lon_deg))
		gps[i].append (float(utm_n))
		gps[i].append (float(utm_e))
		gps[i].append (int(sat))
		gps[i].append (float(hdop))
		gps[i].append (float(alt))
	 
		i += 1

        if i == max_lines:
            break

    file.close()
    return (gps)


### LATITUDE/LONGITUDE TO UTM CONVERTER FUNCTION ###
def ll2utm (lat, lon):

    flat = 1/298.257223563 # WGS84 flat
    a = 6378137 # WGS84 equatorial radius
    k0 = 0.9996
    latr = lat * pi/180.
    lonr = lon * pi/180.
    e = 0
    n = 0
    znum = -1
    zlet = '0'

    # test if the UTM projection is defined for this latitude and longitude
    if lat <= 84.0 and lat >= -80.0:
        # determine the UTM zone number 
        znum = int((lon + 180)/6) + 1

        if lat >= 56. and lat < 64. and lon >= 3. and lon < 12.:
            znum = 32

	# take care of zone numbers for Svalbard
        if lat >= 72. and lat < 84.:
            if lon >=  0. and lon <  9.: znum = 31
            elif lon >=  9. and lon < 21.: znum = 33
            elif lon >= 21. and lon < 33.: znum = 35
            elif lon >= 33. and lon < 42.: znum = 37
	    
	# determine the UTM zone letter
        if  84.0 >= lat and lat >= 72.0: zlet = 'X'
        elif 72.0 > lat and lat >= 64.0: zlet = 'W'
        elif 64.0 > lat and lat >= 56.0: zlet = 'V'
        elif 56.0 > lat and lat >= 48.0: zlet = 'U'
        elif 48.0 > lat and lat >= 40.0: zlet = 'T'
        elif 40.0 > lat and lat >= 32.0: zlet = 'S'
        elif 32.0 > lat and lat >= 24.0: let = 'R'
        elif 24.0 > lat and lat >= 16.0: zlet = 'Q'
        elif 16.0 > lat and lat >= 8.0: zlet = 'P'
        elif 8.0 > lat and lat >= 0.0: zlet = 'N'
        elif 0.0 > lat and lat >= -8.0: zlet = 'M'
        elif -8.0 > lat and lat >= -16.0: zlet = 'L'
        elif -16.0 > lat and lat >= -24.0: zlet = 'K'
        elif -24.0 > lat and lat >= -32.0: zlet = 'J'
        elif -32.0 > lat and lat >= -40.0: zlet = 'H'
        elif -40.0 > lat and lat >= -48.0: zlet = 'G'
        elif -48.0 > lat and lat >= -56.0: zlet = 'F'
        elif -56.0 > lat and lat >= -64.0: zlet = 'E'
        elif -64.0 > lat and lat >= -72.0: zlet = 'D'
        elif -72.0 > lat and lat >= -80.0: zlet = 'C'

        # calculate UTM northing and easting
        es = 2*flat-flat*flat
        eps = (es)/(1-es)

        # find the center longitude for the UTM zone 
        lonr_center = ((znum -1)*6-180+3) * pi/180.

        N = a/sqrt(1-es*sin(latr)*sin(latr))
        T = tan(latr)*tan(latr)
        C = eps*cos(latr)*cos(latr)
        A = cos(latr)*(lonr-lonr_center)
        M = a*((1 - es/4 - 3*es*es/64 - 5*es*es*es/ 256)*latr - (3* es/8 + 3*es*es/32 + 45*es*es*es/1024)*sin(2*latr) + (15*es*es/256 + 45*es*es*es/1024)*sin(4*latr)-(35*es*es*es/3072)*sin(6*latr))
        e = (k0*N*(A+(1-T+C)*A*A*A/6 + (5-18*T+T*T+72*C-58*eps)*A*A*A*A*A/120) + 500000.0)
        n = (k0*(M+N*tan(latr)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24 + (61-58*T+T*T+600*C-330*eps)*A*A*A*A*A*A/720)));

        if lat < 0:
            n += 10000000.0 # 10000000 meter offset for southern hemisphere 

    return (znum, zlet, n, e)

### GNSS data extract in list of different solutions ###
def gnss_extract (data):
    f1 = []
    f2 = []
    f4 = []
    f5 = []
    for i in range(len(data)):
        if data[i][0] == 1:
            f1.append(data[i])
        elif data[i][0] == 2:
            f2.append(data[i])
        elif data[i][0] == 4:
            f4.append(data[i])
        elif data[i][0] == 5:
            f5.append(data[i])
    return (f1, f2, f5, f4)

### Data chooser function ###
def data_choose(data,solution):
	(f1, f2, f5, f4) = gnss_extract(data)

	if solution == 0:
		solution_data = data
		solution_string = "All solutions"
	elif solution == 1:
		solution_data = f1
		solution_string = "Single"
	elif solution == 2:
		solution_data = f2
		solution_string = "DGNSS"
	elif solution == 5:
		solution_data = f5
		solution_string = "Float"
	elif solution == 4:
		solution_data = f4
		solution_string = "Fix solution"
	elif solution == 6:
		#solution_data = f5 + f4
		solution_data = []
		for i in range(len(data)):
			if data[i][0] == 4 or data[i][0] == 5:
				solution_data.append(data[i])
		solution_string = "Float and Fix"
	return (solution_data,solution_string)

### Outlier remove function ###
def outlier_remove(data, threshold):
	new_data = []
	outliers = [0,0,0,0,0,0,0]
	for i in range(len(data)):
		if data[i][10] < threshold:
			new_data.append(data[i])
		else:
			if data[i][0] == 1:
				outliers[1] += 1
			elif data[i][0] == 2:
				outliers[2] += 1
			elif data[i][0] == 4:
				outliers[4] += 1
			elif data[i][0] == 5:
				outliers[5] += 1
			outliers[0] += 1	
	return (new_data,outliers)	

### Main ###

print "Importing GNSS data from file: "+ csv_file
data = gps_import(csv_file, max_lines)

print "Number of samples", (len(data))

#Split the total data into list based on GNSS solution
(f1, f2, f5, f4) = gnss_extract(data)
print "Single solution: ", len(f1)
print "DGNSS solution:", len(f2)
print "RTK float solution: ", len(f5)
print "RTK fix solution: ", len(f4)

latlons = []

(data_solution,midpoint_solution) = data_choose(data,midpoint_calculation_solution)
print "Midpoint calculation is based on "+midpoint_solution

#make a tuple list of all latitude and longitude in the choosen solution
for i in range(len(data_solution)):
	latlons.append((data_solution[i][1],data_solution[i][2]))

# Calcalute the geographic midpoint
geompt = geographic_midpoint(latlons)

# Store midpoints
midpoint_lat = geompt[0]
midpoint_lon = geompt[1]
print "Geographic midpoint: ", midpoint_lat, midpoint_lon
print "Distance between averaged and center of mass: ", dist

# Convert midpoints in lat & lon to midpoints in UTM
(znum, zlet, midpoint_n, midpoint_e) = ll2utm (midpoint_lat, midpoint_lon)
print "UTM ",znum,zlet,"          N: ",midpoint_n,"m","       E: ",midpoint_e,"m"

# add relative distances to the list
for i in range(len(data)):
	data[i].append (data[i][3]-midpoint_n)
	data[i].append (data[i][4]-midpoint_e)
	data[i].append (sqrt(data[i][8]**2 + data[i][9]**2))


# Divide data by solution
(f1, f2, f5, f4) = gnss_extract(data)
length_data = [len(f1+f2+f4+f5),len(f1),len(f2),1,len(f4),len(f5)]

# Remove outliers from the data
if outlier_threshold != 0:
	(data_no_outliers,outliers) = outlier_remove(data,outlier_threshold)
	data = list(data_no_outliers)
else:
	outliers = [0,0,0,0,0,0]
print outliers, "outliers were removed from the data"
print (100*float(outliers[0])/(float(1) if length_data[0]==0 else float(length_data[0]))), "% of total data was removed"
print (100*float(outliers[1])/(float(1) if length_data[1]==0 else float(length_data[1]))), "% of single solution data was removed"
print (100*float(outliers[2])/(float(1) if length_data[2]==0 else float(length_data[2]))), "% of differential solution data was removed"
print (100*float(outliers[5])/(float(1) if length_data[5]==0 else float(length_data[5]))), "% of float solution data was removed"
print (100*float(outliers[4])/(float(1) if length_data[4]==0 else float(length_data[4]))), "% of fix solution data was removed"

# Calculate new midpoint
(data_solution,midpoint_solution) = data_choose(data,midpoint_calculation_solution)
#make a tuple list of all latitude and longitude in the choosen solution
latlons1 = []
for i in range(len(data_solution)):
	latlons1.append((data_solution[i][1],data_solution[i][2]))
# Calculate the average of all latitude and longitude
avg1 = geographic_midpoint(latlons1)
# Store midpoints
midpoint_lat1 = avg1[0]
midpoint_lon1 = avg1[1]
print "New midpoint: ", midpoint_lat, midpoint_lon

# Convert midpoints in lat & lon to midpoints in UTM
(znum1, zlet1, midpoint_n1, midpoint_e1) = ll2utm (midpoint_lat1, midpoint_lon1)

# add relative distances to the list
for i in range(len(data)):
	data[i][8] = (data[i][3]-midpoint_n1)
	data[i][9] = (data[i][4]-midpoint_e1)
	data[i][10] = (sqrt(data[i][8]**2 + data[i][9]**2))



### PLOTTING BEGINS ###


#Plot UTM errors in choosen solution
#Split the updated data into list based on GNSS solution

(map_plot_data, solution_string) = data_choose(data,map_plot_solution)

list_utmn_error = []
list_utme_error = []
list_utmn_e_error = []
list_utme_e_error = []

#If both Fix and Float are plotted then split into different lists
for i in range(len(map_plot_data)):
	if map_plot_solution == 6:
		if map_plot_data[i][0]==4:
			list_utmn_error.append(map_plot_data[i][8])
			list_utme_error.append(map_plot_data[i][9])		
		elif map_plot_data[i][0]==5:
			list_utmn_e_error.append(map_plot_data[i][8])
			list_utme_e_error.append(map_plot_data[i][9])
	else:
		list_utmn_error.append(map_plot_data[i][8])
		list_utme_error.append(map_plot_data[i][9])

plt.figure(1)
plot(list_utme_error, list_utmn_error, 'gx')
plt.scatter([0],[0],marker='x',c='b',s=35, lw = 2,zorder=10)
if map_plot_solution == 6:
	plot(list_utme_e_error, list_utmn_e_error, 'rx')
axis ('equal')
xlabel('Easting [m]')
ylabel('Northing [m]')
title ("Map of measurements")#"Map of "+solution_string)
#suptitle("Midpoint calculation is based on "+midpoint_solution+" solution")
plt.savefig('plot_UTM_errors_map.pdf')

#Plot UTM errors in histogram
(hist_plot_data, hist_solution_string) = data_choose(data,histogram_plot_solution)
hist_data = []

for i in range(len(hist_plot_data)):
	hist_data.append(hist_plot_data[i][10])

plt.figure(2)
#suptitle("Midpoint calculation is based on "+midpoint_solution)
title ("Measurement distance to center")#('Measurement distances of '+ hist_solution_string)
xlabel('Distance [m]')
ylabel('Samples')

hist,bins=np.histogram(hist_data,bins=100)

width=1*(bins[1]-bins[0])
center=(bins[:-1]+bins[1:])/2
plt.bar(center,hist,align='center',width=width)
#plt.axis([0, 1,0,500])
#plt.grid(True)
plt.savefig('plot_hist.pdf')

#Plot solution over time
time = []
solution = []
for i in range(len(data)):
	time.append(i)	
	solution.append(data[i][0]) 

plt.figure(3)
title ('GNSS Solution ')
xlabel('Samples')
ylabel('Solution')
plt.plot(time,solution)#, 'ro')
plt.axis([0, len(time), 0, 5.5])
plt.savefig('solution_plot.pdf')

#Plot satellites over time
time_sat = []
sat = []
sat_avg = 0.
for i in range(len(data)):
	time_sat.append(i)	
	sat.append(data[i][5]) 
	sat_avg += data[i][5]

sat_avg = sat_avg/len(data)
print "Average satellites: ",sat_avg
plt.figure(4)
title ('GNSS Satellites ')
xlabel('Samples')
ylabel('Number of satellites')
plt.plot(time,sat)
plt.axis([0, len(time_sat), 0, max(sat)+1])
plt.savefig('satellite_plot.pdf')

#Plot altitude over time
time_alt = []
time_alt_e =[] 
alt = []
alt_e=[]
(alt_plot_data, alt_solution_string) = data_choose(data,alt_plot_solution)

for i in range(len(alt_plot_data)):
	if alt_plot_solution == 6:
		if alt_plot_data[i][0]==4:
			time_alt.append(i)	
			alt.append(alt_plot_data[i][7])
		elif  alt_plot_data[i][0]==5:
			time_alt_e.append(i)	
			alt_e.append(alt_plot_data[i][7])
	else:
		time_alt.append(i)	
		alt.append(alt_plot_data[i][7]) 

plt.figure(5)
title ('Altitude of '+alt_solution_string)
xlabel('Samples')
ylabel('Altitude [m]')
plt.plot(time_alt,alt, 'go', mew=c_mew)
if utm_error_plot_solution == 6:
	plt.plot(time_alt_e,alt_e, 'ro', mew=c_mew)
	plt.axis([0, len(alt_plot_data), min(alt_e)-0.05, max(alt_e)+0.05])
else:
	plt.axis([0, len(time_alt), min(alt)-0.05, max(alt)+0.05])
plt.savefig('alt_plot.pdf')


#Plot altitude errors in histogram
(alt_mean_data, alt_mean_solution_string) = data_choose(data,alt_mean_calculation_solution)
alt_mean_alt_data = []

for i in range(len(alt_mean_data)):
	alt_mean_alt_data.append(alt_mean_data[i][7])

alt_mean = np.mean(alt_mean_alt_data)
print "Altitude mean is ",alt_mean,"m"

(alt_hist_data,alt_hist_solution_string)=data_choose(data,alt_hist_plot_solution)
alt_hist_error_data = []
for i in range(len(alt_hist_data)):
	alt_hist_error_data.append(alt_hist_data[i][7]-alt_mean)

plt.figure(6)
suptitle("Midpoint calculation is based on "+alt_mean_solution_string)
title ('Altitude error distances of '+ alt_hist_solution_string)
xlabel('Distance [m]')
ylabel('Samples')

hist,bins=np.histogram(alt_hist_error_data,bins=100)

width=1*(bins[1]-bins[0])
center=(bins[:-1]+bins[1:])/2
plt.bar(center,hist,align='center',width=width)
plt.savefig('alt_plot_hist.pdf')


#Plot utm_error over time
time_utm_error = []
utm_error = []
time_utm_e_error = []
utm_e_error = []
(utm_error_data, utm_error_solution_string) = data_choose(data,utm_error_plot_solution)

for i in range(len(utm_error_data)):
	if utm_error_plot_solution == 6:
		if utm_error_data[i][0]==4:
			time_utm_error.append(i)	
			utm_error.append(utm_error_data[i][10])
		elif  utm_error_data[i][0]==5:
			time_utm_e_error.append(i)	
			utm_e_error.append(utm_error_data[i][10])
	else:
		time_utm_error.append(i)	
		utm_error.append(utm_error_data[i][10]) 

plt.figure(7)
title ('UTM error of '+utm_error_solution_string)
xlabel('Samples')
ylabel('Error [m]')
plt.plot(time_utm_error,utm_error, 'go',mew = c_mew)
if utm_error_plot_solution == 6:
	plt.plot(time_utm_e_error,utm_e_error, 'ro',mew = c_mew)
	plt.axis([0, len(utm_error_data), min(utm_e_error)-0.05, max(utm_e_error)+0.05])
else:
	plt.axis([0, len(time_utm_error), min(utm_error)-0.05, max(utm_error)+0.05])
plt.savefig('utm_error_time_plot.pdf')



show()

# fix solution 				--> data[i][0]
# latitude in degrees 			--> data[i][1]
# longtitude in degress			--> data[i][2]
# utm north in meters			--> data[i][3]
# utm east in meters			--> data[i][4]
# number of satellites			--> data[i][5]
# hdop					--> data[i][6]
# altitude in meters			--> data[i][7] Altitude over what? water?
# utm_n distance to midpoint in meters	--> data[i][8]
# utm_e distance to midpoint in meters	--> data[i][9]
# utm total error distance to midpoint 	--> data[i][10]

