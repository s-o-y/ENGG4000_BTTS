import time as t
import cv2
import numpy as np
import math
import openpyxl as xl

#Mapping function
def map(x, in_min, in_max, out_min, out_max):
	if x > in_max:
		return out_max
	return (x - in_min)*(out_max-out_min)//(in_max - in_min)

#load workbook
__file__ = '/home/msnell/Documents/ENGG4000/test.CALC'
workbook = xl.load_workbook(__file__)
alt = workbook['alt']
az = workbook['az']

#create array of possible chars
DEC = 65
A = [0] * 26
for i in range(26):
	A[i] = chr(DEC)
	DEC += 1

#make video obj
video = cv2.VideoCapture(0)

#set to 480p width,height
video.set(3,640)
video.set(4,480)

#initial values
prevMax = (0,0)
frames = 0
tot = 0
flag = 0
c = 1
azAr = [0] * 4
altAr = [0] * 4
q1=0

while(1):
	#timing
	start = t.time()

	#read frame
	ret,frame = video.read()


	#preform image preprocessing
	frameC = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	frameC = cv2.GaussianBlur(frameC, (7,7), 0)   
	
	#calculate max location
	(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(frameC)
	#calculate distance from previous to new max
	diff = math.sqrt(abs((maxLoc[0]+prevMax[0])^2 + (maxLoc[1]+prevMax[1])^2))
	
	#trying to handle jitter :(
	if diff>10:
		cv2.circle(frame, maxLoc, 5, (255,0,150), 2) #purple :)
		prevMax = maxLoc
	else:
		cv2.circle(frame, prevMax, 5, (255,0,150), 2)
	cv2.imshow("Frame", frame)
	
	#map values
	new_val = map(maxLoc[0], 0, 640, 0, 13)
	new_valk = map(maxLoc[1], 0, 480, 0, 13)
		
	#get x,y indicies for excel workbook
	ind1 = A[new_val]
	ind2 = str(new_valk)
	
	#calculating duration and frames processed
	end = t.time()
	proctime = end-start
	tot += proctime
	frames += 1
	
	#only print every 5 sec
	if math.floor(tot) % 5 == 0 and tot > 1 and flag == 0:
		#set flag
		flag += 1
		
		#print out
		print("Altitude and Azimuth:")
		print("(" + str(alt[ind1+ind2].value) + "," + str(az[ind1+ind2].value) + ")")
		altAr[c-1] = alt[ind1+ind2].value
		azAr[c-1] = az[ind1+ind2].value
		q1 = q1+90
		print("Move to azimuth" + str(q1) + "location\n")

		#increment multiplier
		c += 1
	#display alt and az once  every 5 sec
	if tot > c*5:
		flag -= 1
	
	#break on 's' key
	if cv2.waitKey(1) & 0xFF == ord('s'):
		break

video.release()
cv2.destroyAllWindows

#getting error terms
altErr = [0] * 4
azErr = [0] * 4
j=0
q2=0

for j in range(4):
	azErr[j] = abs(azAr[j] - q2)
	q2 += 90
	altErr[j] = abs(altAr[j] - 60)

print(altErr)
print(azErr)
