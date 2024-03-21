import time as t
import cv2
import numpy as np
import math
import openpyxl as xl
import matplotlib.pyplot as plt

#Mapping function
def map(x, in_min, in_max, out_min, out_max):
	if x > in_max:
		return out_max
	return (x - in_min)*(out_max-out_min)//(in_max - in_min)
	
x, y, w, h = (0, 102, 639, 243)
#load camera calibration data
__calib_file__ = "/home/msnell/Documents/ENGG4000/calibration_data.npz"
calib_data = np.load(__calib_file__)
repError = calib_data["repError"]
camMtx = calib_data["camMtx"]
distCoeff = calib_data["distCoeff"]
rvecs = calib_data["rvecs"]
tvecs = calib_data["tvecs"]

#load workbook
__file__ = '/home/msnell/Documents/ENGG4000/CALC.xlsx'
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
procArr = [0]*800 #in 30s, approximately 750 frames will be processed, leving additional 50 in case program is not stopped exactly at the 30s marker
while(1):
	#timing
	start = t.time()

	#read frame
	ret,frame = video.read()
	h, w = frame.shape[:2]
	z = (x,y,w,h)
	newcameramtx, z = cv2.getOptimalNewCameraMatrix(camMtx, distCoeff, (w,h), 1, (w,h))
	##Undistoring
	# undistort
	k = t.time()
	dst = cv2.undistort(frame, camMtx, distCoeff, None, newcameramtx)
	k1 = t.time()
	# crop the image
	dst = dst[y:y+h, x:x+w]
	
	#preform image preprocessing
	frameC = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
	frameC = cv2.GaussianBlur(frameC, (7,7), 0)   
	
	#calculate max location
	(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(frameC)
	#calculate distance from previous to new max
	diff = math.sqrt(abs((maxLoc[0]+prevMax[0])^2 + (maxLoc[1]+prevMax[1])^2))
	
	#trying to handle jitter :(
	if diff>10:
		cv2.circle(dst, maxLoc, 5, (255,0,150), 2) #purple :)
		prevMax = maxLoc
	else:
		cv2.circle(dst, prevMax, 5, (255,0,150), 2)
	cv2.imshow("Frame", dst)
	
	#map values
	new_val = map(maxLoc[0], 0, 640, 0, 13)
	new_valk = map(maxLoc[1], 0, 480, 0, 13)
		
	#get x,y indicies for excel workbook
	ind1 = A[new_val]
	ind2 = str(new_valk)
	
	#calculating duration and frames processed
	end = t.time()
	proctime = end-start
	procArr[frames] = proctime
	tot += proctime
	frames += 1
	
	#Testing Purposes
	# #only print every 5 sec
	# if math.floor(tot) % 5 == 0 and tot > 1 and flag == 0:
		# #set flag
		# flag += 1
		# #increment multiplier
		# c += 1
	# #display alt and az once  every 5 sec
	# if tot > c*5:
		# flag -= 1
		# print("Undistort time = " + str(k1-k))
	
	#break on 's' key
	if cv2.waitKey(1) & 0xFF == ord('s'):
		break

#printing stats
print("----------------------------------")
print(" ")
print("Duration:")
print(tot)
print("Frames Processed:")
print(frames)
print("Average FPS:")
print(frames / tot)

video.release()
cv2.destroyAllWindows

#Plot Values
xpoints = np.arange(frames)
ypoints = procArr[:frames]

plt.plot(xpoints,ypoints)
plt.xlabel("Frame")
plt.ylabel("Time to Process")
plt.ylim(0,0.2)
plt.show() 
