from matplotlib import pyplot as plt
import time
from IPython import display
import gpxpy
import gpxpy.gpx
import math

#READ GPX FILE
gpx_file=open('F:/run.gpx') #open gpx file
gpx = gpxpy.parse(gpx_file) #read through file
track = gpx.getElementsByTagName('trkpt') #get trackpoint elements
elevation=gpx.getElementsByTagName('ele') #get elevation
datetime=gpx.getElementsByTagName('time') #get time
n_track=len(track)

#PARSING GPX ELEMENT
lon_list=[] #longitude array
lat_list=[] #latitude array
elvs_list=[]  #elevation array
time_list=[] #times array
for r in range(n_track):
    lon,lat=track[r].attributes['lon'].value,track[r].attributes['lat'].value #finding long and lat
    elev=elevation[r].firstChild.nodeValue #finding elev
    lon_list.append(float(lon)) #save longitude points in array
    lat_list.append(float(lat)) #save latitude points in array
    elvs_list.append(float(elev)) #save elevation points in array
    # PARSING TIME ELEMENT
    dt=datetime[r].firstChild.nodeValue #finding time
    time_split=dt.split('T') #splits in time list
    hms_split=time_split[1].split(':') #organization of time
    time_hour=int(hms_split[0]) #first is hour
    time_minute=int(hms_split[1]) #second is minute
    time_second=int(hms_split[2].split('Z')[0]) #last is seconds
    total_second=time_hour*3600+time_minute*60+time_second
    time_list.append(total_second) #save time to array

#DISTANCE FUNCTION
def distance(x1,y1,x2,y2):
    d=math.sqrt((x1-x2)**2+(y1-y2)**2) #math to get total distance
    return d

#SPEED FUNCTION
def speed(x0,y0,x1,y1,t0,t1):
    d=math.sqrt((x0-x1)**2+(y0-y1)**2) #distance
    t=t1-t0 #time
    s=float(d/t) #speed
    return s

#GEODETIC TO CARTERSIAN FUNCTION
def geo2cart(lon,lat,h):
    j=6378137 #WGS 84 Major axis
    o=6356752.3142 #WGS 84 Minor axis
    g=1-(o**2/j**2)
    N=float(j/math.sqrt(1-g*(math.sin(math.radians(abs(lat)))**2)))
    X=(N+h)*math.cos(math.radians(lat))*math.cos(math.radians(lon))
    Y=(N+h)*math.cos(math.radians(lat))*math.sin(math.radians(lon))
    return X,Y


# POPULATE DISTANCE AND SPEED LIST
dist_list = [0.0] #distance list
speed_list = [0.0] #speed list
l = 0
for k in range(n_track - 1):
    if k < (n_track - 1):
        l = k + 1
    else:
        l = k
    XY0 = geo2cart(lon_list[k], lat_list[k], elvs_list[k]) #geoditic locations first
    XY1 = geo2cart(lon_list[l], lat_list[l], elvs_list[l]) #geoditic locations second

    # DISTANCE
    d = distance(XY0[0], XY0[1], XY1[0], XY1[1]) #using distance varaible to get the distance between two goeditic locations
    sum_d = d + dist_list[-1] #total of distance
    dist_list.append(sum_d) #add distance to array

    # SPEED
    s = speed(XY0[0], XY0[1], XY1[0], XY1[1], time_list[k], time_list[l]) #using speed variable to get the speed from geoditic and time variables
    speed_list.append(s) #add speed to array
#PLOT TRACK
f,(track,speed,elevation)=plt.subplots(3,1)
f.set_figheight(8) #plot height
f.set_figwidth(5) #plot width
plt.subplots_adjust(hspace=0.5)
track.plot(lon_list,lat_list,'k') #set variables on plot
track.set_ylabel("Latitude") #set variable name
track.set_xlabel("Longitude") #set variable name
track.set_title("Track Plot") #set tile name

#PLOT SPEED
speed.bar(dist_list,speed_list,30,color='w',edgecolor='w') #plot details
speed.set_title("Speed")  #set title name
speed.set_xlabel("Distance(m)") #set variable name
speed.set_ylabel("Speed(m/s)") #set variable name

#PLOT ELEVATION PROFILE
base_reg=0 #STARTING VALUE
elevation.plot(dist_list,elvs_list) #plot variables
elevation.fill_between(dist_list,elvs_list,base_reg,alpha=0.1)
elevation.set_title("Elevation Information") #set title name
elevation.set_xlabel("Distance(m)") #set variable name
elevation.set_ylabel("GPS Elevation(m)") #set variable name
elevation.grid()

# ANIMATE PLOT
for i in range(n_track):
    track.plot(lon_list[i], lat_list[i], 'yo')
speed.bar(dist_list[i], speed_list[i], 30, color='g', edgecolor='g')
elevation.plot(dist_list[i], elvs_list[i], 'ro')
display.display(plt.gcf()) #display plots
display.clear_output(wait=True)  #display plots
time.sleep(1)

plt.show() #display plots
