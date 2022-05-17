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
pwm = GPIO.PWM(motor1E, 100)
pwm.start(0)
pwm.ChangeDutyCycle(50)
pwm2 = GPIO.PWM(motor2E, 100)
pwm2.start(0)
pwm2.ChangeDutyCycle(50)
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
args = vars(ap.parse_args())


# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
pts = deque(maxlen=args["buffer"])
# sets up the Pi camera
vs = PiCamera()
vs.resolution = (640,480)
vs.framerate = 50
vs.hflip = True
rawCapture = PiRGBArray(vs, size = (640, 480))

pathList = [] # global path list 


# allow the camera or video file to warm up
time.sleep(2.0)
# define the lower and upper ranges of the colours
greenLower = (37, 42, 0)
greenUpper = (84, 255, 255)
blueLower = (100, 150, 0)
blueUpper = (140,255,255)
redLower = (0, 120, 70)
redUpper = (20, 255, 255)
redLower2 = (170, 120, 70)
redUpper2 = (180, 255, 255)
yellowLower = (15,0,0)
yellowUpper = (36, 255, 255)
pts = deque(maxlen=args["buffer"])
# if a video path was not supplied, grab the reference
# to the webcam

# allow the camera or video file to warm up
time.sleep(.2)


# function to turn the robot
def turn():
    print('turning....')
    sleep(.50)
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
    return

# function to move the robot forward
def goForward():
    print('center, go forward..')
    GPIO.output(motor1A, 0)
    GPIO.output(motor1B, 1)
    GPIO.output(motor1A, 0)
    GPIO.output(motor2B, 1)

    GPIO.output(motor1E, 1)
    GPIO.output(motor2E, 1)

    sleep(.50)
    
    GPIO.output(motor1A, 0)
    GPIO.output(motor1B, 0)
    GPIO.output(motor1E, 0)
    GPIO.output(motor2A, 0)
    GPIO.output(motor2B, 0)
    GPIO.output(motor2E, 0)
    
    return
# for A Star searches
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
        
# checks if the given structure is empty
def is_empty(structure):
    
    if structure:
         return False
    else:
         return True

openList = []
closedList = []
nodeCounter = 0 


# function to detect a coloured object, modified from Adrian Rosebrock's code
def ObjectColour(mask, colour, frame, map, pathList, spinTime):


    if colour in pathList:
        # checks if the given colour has already been scanned
        print('already done??')
        sleep(.50)
        turn()
        return;

    # ges contour of object
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

            # checks if the object is in the center,
            # and checks if its time to spin; ie if the path is being explored
            # or executed
            if(( 426 > x > 213)):
                print(colour, ' center??')
                pathList.append(colour)
                if spinTime:
                    sleep(.50)
                    goForward()
                    return;
            else:
                turn()
        else:
            turn()
                
    else:
        print()
               
        sleep(.50)
        turn()

        pts.appendleft(center)
        

# similar to the explore function,
# but the purpose of this is to execute the path
# gets passed a colour, the function sees if the colour is on the screen,
# and turns until the colour is in the center
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

        spinnyTime.maskRed1 = cv2.inRange(hsv, redLower, redUpper)
        spinnyTime.maskRed1 = cv2.erode(spinnyTime.maskRed1, None, iterations=2)
        spinnyTime.maskRed1= cv2.dilate(spinnyTime.maskRed1, None, iterations=2)
        spinnyTime.maskRed2 = cv2.inRange(hsv, redLower2, redUpper2)
        spinnyTime.maskRed2 = cv2.erode(spinnyTime.maskRed2, None, iterations=2)
        spinnyTime.maskRed2= cv2.dilate(spinnyTime.maskRed2, None, iterations=2)
        spinnyTime.maskRed = spinnyTime.maskRed2 + spinnyTime.maskRed2
        
        emptyList = []
        ObjectColour(colourSwitch(colour), colour, frame, map, emptyList, spinTime)

        maskColour = colourSwitch(colour)
        if colour in emptyList:
            
            print('already done??')
            sleep(.50)
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
# function to determine the right mask for the given colour,
# make-shift python version of case - switch
def colourSwitch(colour):
    switcher = {
        'yellow' : spinnyTime.maskYellow,
        'green' : spinnyTime.maskGreen,
        'blue' : spinnyTime.maskBlue,
        'red' : spinnyTime.maskRed}
    return switcher.get(colour)


# function to find out the map/path.
# Robot spins until a full map is made, ie 4 nodes
def Explore(start, goal):
    global pathList
    start_node = Node(start,  None)
    goal_node = Node(goal, None)
    openList.append(start_node)
    Explore.pathList = []
    nodeCounter = 0
    map = Map()
    
    spinTime = False
    timer = 0
    while len(openList) > 0 and timer < 25:
        timer = timer + 1

        openList.sort()

        current_node = openList.pop(0)
        closedList.append(current_node)

            
        
        # keep looping
        
        for frame in vs.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    
            # resize the frame, blur it, and convert it to the HSV
            # color space
            frame = imutils.resize(frame.array, width=640)
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            # construct a mask for the colour, then perform
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

            maskRed1 = cv2.inRange(hsv, redLower, redUpper)
            maskRed1 = cv2.erode(maskRed1, None, iterations=2)
            maskRed1= cv2.dilate(maskRed1, None, iterations=2)
            maskRed2 = cv2.inRange(hsv, redLower2, redUpper2)
            maskRed2 = cv2.erode(maskRed2, None, iterations=2)
            maskRed2= cv2.dilate(maskRed2, None, iterations=2)

            maskRed = maskRed1 + maskRed2


            
        # find contours in the mask and initialize the current
            # (x, y) center of the ball
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

# the middleman function between sorting the map and executing it
def PathFinder(pathMap, goal, start):
    map = Map()

    # this creates the connections
    mapRoadYel = ['blue', 'green']
    mapRoadGre = ['yellow', 'blue']
    mapRoadBlu = ['yellow', 'green', 'red']
    mapRoadRed = ['red', 'blue']
    # mapRoad is the dict of conections
    mapRoad = { 'yellow' : mapRoadYel,
                 'green' : mapRoadGre,
                 'blue' : mapRoadBlu,
                 'red' : mapRoadRed}
   
    counter = 0
    for i in (pathMap): # goes through every node(colour) in the passed map

        for key, value in mapRoad.items(): # gets the colour and connected colours
            #  if the given colour is equal to the colour in the connection
            # and if the next colour in the map
            # is in the list of connects of that colour
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
    pathWeighted = Weighted_A_Star(map, start, goal)
    print(path)
    print(pathWeighted)
    executePath(path,pathWeighted)    
    print()


    # just A_Star function, used from past assignment
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
# similar to A_Star, but the heuristics are weighted 1.5 times more. 
def Weighted_A_Star(map, start, goal):
    
    openList = []
    closed = []
    weight = 1.5
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
            neighbor.f = neighbor.g + weight*neighbor.h

            if(add_to_openList(openList, neighbor) == True):
               
                openList.append(neighbor)

    return None
# part of A_Star and Weighted_A_Star
def add_to_openList(openList, neighbor):
    for node in openList:
        if (neighbor == node and neighbor.f > node.f):
            return False
    return True
# executs the sorted map. 
def executePath(map, weightedMap):
 
    n = 2
    costList =[]
    pathList = []
    costListWeighted =[]
    pathListWeighted = []
    # splits the maps up into colours and cost. 
    for i, x in enumerate(map):
        if i % n == 0:
            costList.append(x)
        else:
            pathList.append(x)
    print('cost ', costList)
    print('path ', pathList)

    for i, x in enumerate(weightedMap):
        if i % n == 0:
            costListWeighted.append(x)
        else:
            pathListWeighted.append(x)
    print('cost weighted ', costListWeighted)
    print('path weighted ', pathListWeighted)

    
    # if the cost of the weighted A Star is less, use  that map
    # lights up LED on breadboard on robot
    if costListWeighted[-1] < costList[-1]:
        print('using weighted A star ')
        GPIO.output(motor1E, 1)
        sleep(1)
        GPIO.output(motor1E, 0)
        for x in pathListWeighted[1:]: # excludes the first colour because that
            # is where we are starting
            # then for all the other colours, runs spinnyTime
            spinnyTime(x)
    else:
        print('using A star')
        GPIO.output(motor1E, 1)
        sleep(1)
        GPIO.output(motor1E, 0)
        for x in pathList[1:]:
            spinnyTime(x)
    


def dijkstra(graph, start):
    distances = {vertex : float('infinity') for vertex in  graph}
    distances[start] = 0

    pq = [(0, start)]
    while len(pq) > 0:
        current_distance, current_vertex = heapq.heappop(pq)

        if current_distance > distances[current_vertex]:
            continue

        for neighbor, weight in (graph.items()):
            distance = current_distance + int(weight)

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heaph.heappush(pq, (distance, neighbor))
    print distances
    return distances
def main():
    # runs the explore function to find the map/path, give red is the goal
    # and yellow is the start
    Explore('red', 'yellow')
    #sleep(2)
    #pathList = ['yellow', 'green','blue','red']

    print(pathList)
    
    PathFinder(pathList, 'red', 'yellow' )

    

main()

# close everything and tidy up
GPIO.cleanup()
vs.close()
cv2.destroyAllWindows()

