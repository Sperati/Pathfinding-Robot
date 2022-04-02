
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
from random import randint
import heapq
import RPi.GPIO as GPIO
from time import sleep

# set up GPIO for motors
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)


motor1A = 13
motor1B = 15
motor1E = 11

motor2A = 12
motor2B = 16
motor2E = 18

GPIO.setup(motor1A, GPIO.OUT)
GPIO.setup(motor1B, GPIO.OUT)
GPIO.setup(motor1E, GPIO.OUT)

GPIO.setup(motor2A, GPIO.OUT)
GPIO.setup(motor2B, GPIO.OUT)
GPIO.setup(motor2E, GPIO.OUT)
GPIO.output(motor1A, 0)
GPIO.output(motor1B, 0)
GPIO.output(motor1E, 0)
GPIO.output(motor2A, 0)
GPIO.output(motor2B, 0)
GPIO.output(motor2E, 0)

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
args = vars(ap.parse_args())


# define the lower and upper boundaries of the "green"
# object in the HSV color space, then initialize the
# list of tracked points
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
pts = deque(maxlen=args["buffer"])
# if a video path was not supplied, grab the reference
# to the webcam
#global vs
vs = PiCamera()
vs.resolution = (640,480)
vs.framerate = 50
vs.hflip = True
#global rawCapture
rawCapture = PiRGBArray(vs, size = (640, 480))



# allow the camera or video file to warm up
time.sleep(2.0)
# define the lower and upper boundaries of the "green"
# object in the HSV color space, then initialize the
# list of tracked points
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
blueLower = (110, 50, 50)
blueUpper = (130,255,255)
redLower = (0, 150, 50)
redUpper = (5, 255, 255)
yellowLower = (22,93,0)
yellowUpper = (45, 255, 255)
pts = deque(maxlen=args["buffer"])
# if a video path was not supplied, grab the reference
# to the webcam

# allow the camera or video file to warm up
time.sleep(.1)

def turn():
    print('turning....')
    GPIO.output(motor1A, 0)
    GPIO.output(motor1B, 0)


    GPIO.output(motor1E,0)

    sleep(.1)
    
    GPIO.output(motor1A, 0)
    GPIO.output(motor1B, 0)
    GPIO.output(motor1E, 0)
    GPIO.output(motor2A, 0)
    GPIO.output(motor2B, 0)
    GPIO.output(motor2E, 0)
    return

def goForward():
  
    print('center, go forward..')
    GPIO.output(motor1A, 0)
    GPIO.output(motor1B,0)

    GPIO.output(motor2A, 0)
    GPIO.output(motor2B, 0)

    

    GPIO.output(motor1E, 0)

    sleep(1)

    GPIO.output(motor1E, 0)
    GPIO.output(motor1A, 0)
    GPIO.output(motor1B, 0)
    GPIO.output(motor1E, 0)
    GPIO.output(motor2A, 0)
    GPIO.output(motor2B, 0)
    GPIO.output(motor2E, 0)
    GPIO.output(motor2E, 0)
    return

class Node(object):
    def __init__(self, name, parent):
        self.name = name
        self.parent = parent
        self.g = 0 
        self.h = 0
        self.f = 0 

    def __eq__(self, other):
        return self.name == other.name

    def __lt__(self, other):
         return self.f < other.f

    def __repr__(self):
       return (u'({0},{1})'.format(self.name, self.f))
    
class Map(object):
    
    def __init__(self, map_dict=None):
        
        self.map_dict = map_dict or {}
        
    def __str__(self):
        
        return str(self.__class__) + ': ' + str((self.__dict__))
    
    def add(self, A, B, distance=1):
        
        self.map_dict.setdefault(A, {})[B] = distance
        
    def get(self, a, b=None):
        
        roads = self.map_dict.setdefault(a, {})
        if b is None:
            return roads
        else:
            return roads.get(b)
        
def is_empty(structure):
    
    if structure:
         return False
    else:
         return True

openList = []
closedList = []
nodeCounter = 0 



def ObjectColour(mask, colour, frame, map, pathList, spinTime):
   

    if colour in pathList:
        
        print('already done??')
        turn()
        return;
        
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    # only proceed if at least one contour was found
    if len(cnts) > 0:
    # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        if radius > 60 : 
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # only proceed if the radius meets a minimum size

            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),
            (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            if(( 400 > x > 200) and (300 > y > 150)):
                print(colour, ' center??')
                pathList.append(colour)
                if spinTime:
                    goForward()
           
    else:
        print()
               # sleep(1)
        turn()

            # turn()
            # turn untill full pathList, or map is found??

        pts.appendleft(center)
        

def spinnyTime(colour):
    spinTime = True
    
    print('out spin')    
    for frame in vs.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        if frame is None:
            break
        frame = imutils.resize(frame.array, width=640)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
 
        spinnyTime.maskGreen = cv2.inRange(hsv, greenLower, greenUpper)
        spinnyTime.maskGreen = cv2.erode(spinnyTime.maskGreen, None, iterations=2)
        spinnyTime.maskGreen = cv2.dilate(spinnyTime.maskGreen, None, iterations=2)

        spinnyTime.maskBlue = cv2.inRange(hsv, blueLower, blueUpper)
        spinnyTime.maskBlue = cv2.erode(spinnyTime.maskBlue, None, iterations=2)
        spinnyTime.maskBlue = cv2.dilate(spinnyTime.maskBlue, None, iterations=2)

        spinnyTime.maskYellow = cv2.inRange(hsv, yellowLower, yellowUpper)
        spinnyTime.maskYellow = cv2.erode(spinnyTime.maskYellow, None, iterations=2)
        spinnyTime.maskYellow = cv2.dilate(spinnyTime.maskYellow, None, iterations=2)

        spinnyTime.maskRed = cv2.inRange(hsv, redLower, redUpper)
        spinnyTime.maskRed = cv2.erode(spinnyTime.maskRed, None, iterations=2)
        spinnyTime.maskRed= cv2.dilate(spinnyTime.maskRed, None, iterations=2)
        emptyList = []
        ObjectColour(colourSwitch(colour), colour, frame, map, emptyList, spinTime)

        maskColour = colourSwitch(colour)
        if colour in emptyList:
            
            print('already done??')
            turn()
            rawCapture.truncate(0)
            break
            
                
   
        rawCapture.truncate(0)
            # show the frame to our screen
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
            # if the 'q' key is pressed, stop the loop
        if len(emptyList) > 4:
            rawCapture.truncate(0)
            break
        if key == ord("q"):
            break
        rawCapture.truncate(0)

def colourSwitch(colour):
    switcher = {
        'yellow' : spinnyTime.maskYellow,
        'green' : spinnyTime.maskGreen,
        'blue' : spinnyTime.maskBlue,
        'red' : spinnyTime.maskRed}
    return switcher.get(colour)
def Explore(start, goal):
    global pathList
    start_node = Node(start,  None)
    goal_node = Node(goal, None)
    openList.append(start_node)
    Explore.pathList = []
    nodeCounter = 0
    map = Map()
    
    spinTime = False
    while len(openList) > 0:

        openList.sort()

        current_node = openList.pop(0)
        closedList.append(current_node)

            
        
        # keep looping
        #rawCapture.truncate()
        for frame in vs.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the current frame
           # frame = vs.read()
            # handle the frame from VideoCapture or VideoStream
            #frame = frame[1] if args.get("video", False) else frame
            # if we are viewing a video and we did not grab a frame,
            # then we have reached the end of the video
           # if frame is None:
              #      break
            # resize the frame, blur it, and convert it to the HSV
            # color space
            frame = imutils.resize(frame.array, width=640)
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            # construct a mask for the color "green", then perform
            # a series of dilations and erosions to remove any small
            # blobs left in the mask
            maskGreen = cv2.inRange(hsv, greenLower, greenUpper)
            maskGreen = cv2.erode(maskGreen, None, iterations=2)
            maskGreen = cv2.dilate(maskGreen, None, iterations=2)

            maskBlue = cv2.inRange(hsv, blueLower, blueUpper)
            maskBlue = cv2.erode(maskBlue, None, iterations=2)
            maskBlue = cv2.dilate(maskBlue, None, iterations=2)

            maskYellow = cv2.inRange(hsv, yellowLower, yellowUpper)
            maskYellow = cv2.erode(maskYellow, None, iterations=2)
            maskYellow = cv2.dilate(maskYellow, None, iterations=2)

            maskRed = cv2.inRange(hsv, redLower, redUpper)
            maskRed = cv2.erode(maskRed, None, iterations=2)
            maskRed= cv2.dilate(maskRed, None, iterations=2)



            
        # find contours in the mask and initialize the current
            # (x, y) center of the object
            ObjectColour(maskGreen, 'green', frame, map, pathList, spinTime )
            ObjectColour(maskBlue, 'blue', frame, map, pathList, spinTime)
            ObjectColour(maskRed, 'red', frame, map, pathList, spinTime)
            ObjectColour(maskYellow, 'yellow', frame, map, pathList, spinTime)




            # show the frame to our screen
          

            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF
            rawCapture.truncate(0)
            # if the 'q' key is pressed, stop the loop
            if len(pathList) > 3:
                rawCapture.truncate(0)
                print(Explore.pathList)
                break
            if key == ord("q"):
                rawCapture.truncate(0)
                print(Explore.pathList)
                break
            print(Explore.pathList)
        

def ReturnToStart():
    print()

def PathFinder(pathMap, goal, start):
    map = Map()

    mapRoadYel = ['blue', 'green']
    mapRoadGre = ['yellow', 'blue']
    mapRoadBlu = ['yellow', 'green', 'red']
    mapRoadRed = ['red', 'blue']

    mapRoad = { 'yellow' : mapRoadYel,
                 'green' : mapRoadGre,
                 'blue' : mapRoadBlu,
                 'red' : mapRoadRed}
    counter = 0
    for i in (pathMap):


        for key, value in mapRoad.items():
            if ((i == key) and (pathMap[counter + 1] in value)):
                for val in value:
                    randomInt = randint(1,10)
                    map.add(i, val, randomInt)
                    map.add(val, i, randomInt)
             
    counter += 1
    

   
    print()
    print( map)
    print()
  
    path = A_Star(map, start, goal)
    print(path)
    executePath(path)    
    print()
def A_Star(map, start, goal):
    
    openList = []
    closed = []
    
    start_node = Node(start, None)
    goal_node = Node(goal, None)

    openList.append(start_node)
    
   
    while len(openList) > 0:
       
        openList.sort()
      
        
        current_node = openList.pop(0)
        
        closed.append(current_node)
        
      
        if current_node == goal_node:
            path = []
            while current_node != start_node:
   
                path.append(current_node.name)
                path.append(unicode(current_node.g))
                current_node = current_node.parent
            path.append(start_node.name)
            path.append(unicode(start_node.g))

            return path[::-1]
    
        neighbors = map.get(current_node.name)
       
        for key, value in neighbors.items():
           
            neighbor = Node(key, current_node)
          
            if(neighbor in closed):
                continue
            
            neighbor.g = current_node.g + map.get(current_node.name, neighbor.name)
            
            neighbor.h = (int(neighbor.g - goal_node.g) **2) + (int(neighbor.g - goal_node.g)**2)
            neighbor.f = neighbor.g + neighbor.h

            if(add_to_openList(openList, neighbor) == True):
               
                openList.append(neighbor)

    return None

def add_to_openList(openList, neighbor):
    for node in openList:
        if (neighbor == node and neighbor.f > node.f):
            return False
    return True
def executePath(map):

    n = 2
    costList =[]
    pathList = []
    for i, x in enumerate(map):
        if i % n == 0:
            costList.append(x)
        else:
            pathList.append(x)
    print('cost ', costList)
    print('path ', pathList)
    for x in pathList[1:]:
        spinnyTime(x)


    
pathList = []
def spinCir():
    print('turning....')
    i = 0
    y = 0
    b = 0
    r = 0
    while i < 23:
        sleep(.75)
        GPIO.output(motor1A, 0)
        GPIO.output(motor1B, 1)


        GPIO.output(motor1E, 1)

        sleep(.1)
        
        GPIO.output(motor1A, 0)
        GPIO.output(motor1B, 0)
        GPIO.output(motor1E, 0)
        GPIO.output(motor2A, 0)
        GPIO.output(motor2B, 0)
        GPIO.output(motor2E, 0)
        i = i + 1
    sleep(.75)
    y = 100
    while y < 3:
        sleep(.75)
        GPIO.output(motor1A, 0)
        GPIO.output(motor1B, 1)
        GPIO.output(motor1A, 0)
        GPIO.output(motor2B, 0)

        GPIO.output(motor1E, 1)
        GPIO.output(motor2E, 1)

        sleep(.1)
        
        GPIO.output(motor1A, 0)
        GPIO.output(motor1B, 0)
        GPIO.output(motor1E, 0)
        GPIO.output(motor2A, 0)
        GPIO.output(motor2B, 0)
        GPIO.output(motor2E, 0)
        y = y + 1

    GPIO.output(motor1A, 0)
    GPIO.output(motor1B, 1)
    GPIO.output(motor1A, 0)
    GPIO.output(motor2B, 1)

    GPIO.output(motor1E, 1)
    GPIO.output(motor2E, 1)

    sleep(.2)

    GPIO.output(motor1A, 0)
    GPIO.output(motor1B, 0)
    GPIO.output(motor1E, 0)
    GPIO.output(motor2A, 0)
    GPIO.output(motor2B, 0)
    while b < 5:
        sleep(.75)
        GPIO.output(motor1A, 0)
        GPIO.output(motor1B, 1)
        GPIO.output(motor1A, 0)
        GPIO.output(motor2B,0)

        GPIO.output(motor1E, 1)
        GPIO.output(motor2E, 1)

        sleep(.1)
        
        GPIO.output(motor1A, 0)
        GPIO.output(motor1B, 0)
        GPIO.output(motor1E, 0)
        GPIO.output(motor2A, 0)
        GPIO.output(motor2B, 0)
        GPIO.output(motor2E, 0)
        b = b + 1
  
    GPIO.output(motor1A, 0)
    GPIO.output(motor1B, 1)
    GPIO.output(motor1A, 0)
    GPIO.output(motor2B,1)

    GPIO.output(motor1E, 1)
    GPIO.output(motor2E, 1)

    sleep(.2)
    
    GPIO.output(motor1A, 0)
    GPIO.output(motor1B, 0)
    GPIO.output(motor1E, 0)
    GPIO.output(motor2A, 0)
    GPIO.output(motor2B, 0)
    GPIO.output(motor2E, 0)
    while r < 8:
        sleep(.75)
        GPIO.output(motor1A, 0)
        GPIO.output(motor1B, 1)
        GPIO.output(motor1A, 0)
        GPIO.output(motor2B, 0)

        GPIO.output(motor1E, 1)
        GPIO.output(motor2E, 1)

        sleep(.1)
        
        GPIO.output(motor1A, 0)
        GPIO.output(motor1B, 0)
        GPIO.output(motor1E, 0)
        GPIO.output(motor2A, 0)
        GPIO.output(motor2B, 0)
        GPIO.output(motor2E, 0)
        r = r + 1
    GPIO.output(motor1A, 0)
    GPIO.output(motor1B, 1)
    GPIO.output(motor1A, 0)
    GPIO.output(motor2B, 1)

    GPIO.output(motor1E, 1)
    GPIO.output(motor2E, 1)

    sleep(.2)
    
    GPIO.output(motor1A, 0)
    GPIO.output(motor1B, 0)
    GPIO.output(motor1E, 0)
    GPIO.output(motor2A, 0)
    GPIO.output(motor2B, 0)
    GPIO.output(motor2E, 0)
    return
def dijkstra(graph, start):
    distances = {vertex : float('infinity') for vertex in  graph}
    distances[start] = 0

    pq = [(0, start)]
    while len(pq) > 0:
        current_distance, current_vertex = heapq.heappop(pq)

        if current_distance > distances[current_vertex]:
            continue

        for neighbor, weight in graph[current_vertex].items():
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heaph.heappush(pq, (distance, neighbor))
    return distances
def main():

    Explore('red', 'blue')

    print(pathList)
    PathFinder(pathList, 'red', 'yellow' )

    

    
# close all windows
main()


GPIO.cleanup()

cv2.destroyAllWindows()

