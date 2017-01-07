# -*- coding: utf-8 -*-
'''
Functions       :heuristic(a,b) , astar(array, start, goal) , path_planner(start,end,grid) , main(image_filename) 
Global Variables:None
'''





import cv2
import numpy as np

from heapq import *

'''
Function Name   :heuristic
Input           :tupple a and b -> which stores the coordinates of the points between which we need to find the heuristic distance
Output          :gives the square of the distance between two points ((x1-x2)**2 +(y1-y2)**2)
Logic           :We keep the check of the distance of current position from goal and start to find our final score of the path being taken
Example Call    :heuristic((0,0),(5,5))
'''

def heuristic(a, b):
        return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2


'''
Function Name   :astar
Input           :numpy array,starting co-ordinate,goal co-ordinate
                 numpy array->We convert our grid to numpy array of 10x10,free path is represnted with a 0 and obstacles with 1
                 starting co-ordinate->the point(a tupple) from which we start 
                 goal co-ordinate->the point(a tupple) to which we need the shortest path
Output          :A list which contains the path to goal point
Logic           :We are using A* algorithm to find the shortest path.This function defines the neighbours
                 The score of each step is calculated by the sum of its heuristic distance from goal and starting point
                 A* alogith has been implemented using heaps.
Example Call    :astar(qmap,start,end)
'''

def astar(array, start, goal):
        '''
        this function first defines neighbours,then our sole aim is to minimise the sum of score(distance) from goal and start.
        We used A* to avoid the risk of travelling along the walls of obstacle which arises due to use of heuristic distance.
        '''
        neighbors = [(0,1),(0,-1),(1,0),(-1,0)]     #Defines the valid neighbours of the current point

        close_set = set()       #
        came_from = {}          #dictionary to keep track of coordinates travelled
        gscore = {start:0}      #dictionary to store score from start
        fscore = {start:heuristic(start, goal)} #dictionary to store final score
        oheap = []              #list to push in heap

        heappush(oheap, (fscore[start], start))
        
        while oheap:
                current = heappop(oheap)[1] #current position

                if current == goal: #finally when we reach goal,we append all points in came from to a list 'data'.Hence our path is stored backwards
                        data = []
                        while current in came_from:
                                data.append(current)
                                current = came_from[current]
                        return data #list we get upon calling the function

                close_set.add(current)
                for i, j in neighbors:
                        neighbor = current[0] + i, current[1] + j       #this lets us go in all four directions from current position
                        tentative_g_score = gscore[current] + heuristic(current, neighbor) #extracting current score of our current position and then adding 1
                        if 0 <= neighbor[0] < array.shape[0]:
                                if 0 <= neighbor[1] < array.shape[1]:                
                                        if array[neighbor[0]][neighbor[1]] == 1: #if neighbor value is 1 i.e it is an obstacle,we leave that path
                                                continue
                                else:
                                        # array bound y walls
                                        continue
                        else:
                                # array bound x walls
                                continue
                                
                        if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                                continue        #if tentative score is higher then leave that path
                                
                        if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:    #our most imp loop
                                came_from[neighbor] = current                                                       #storing valid path till now in came from
                                gscore[neighbor] = tentative_g_score                                                #score from start
                                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)                    #total score=score from start +score from goal
                                heappush(oheap, (fscore[neighbor], neighbor))
                                
        return False

    
'''
Function Name   :path_planner
Input           :start->a tupple containing starting point
                 end->a tupple containing goal point
                 grid->a numpy array which will get the list of occupied positions
Output          :a list which contains the value the requested key of our planned_path dictionary
Logic           :Here,we created our grid in terms of numpy array with obstacles as  and free path as 0
                 Actually,we can't work with numpy array with the way the our x and y axis have been given in problem statement.
                 So we first convert our start and end point to standard points using x->(y-1) and y->(x-1)
                 this way,x represnts the rows and y reprensents columns both starting from 0
                 then we finally create dict_val(a list) as the required format of our problem statement
Example Call    :path=path_planner(start,end,occupied_grids)
'''

def path_planner(start,end,grid):
        occupied_grids=grid #our list of occupied grids
        dict_val=[] #the list which this function will return
        dict_val_inner=[] #llist used to convert back from cartesian system to the way described by our task
        qmap=np.zeros((10,10)).astype(int)  # a 10x10 numpy array with all values 0
        start=(start[1]-1,start[0]-1)   #converting start point to cartesian system
        end=(end[1]-1,end[0]-1) #converting goal point to cartesian system
        for (i,j) in occupied_grids:
                qmap[j-1][i-1]=1 #putting obstacles as 1
        qmap[start[0]][start[1]] = 0    #storing 0 for the start position,else it will also get considered as an obstacle and A* will never succeed
        qmap[end[0]][end[1]] = 0    ##storing 0 for the end position,else it will also get considered as an obstacle and A* will never succeed
        path_list=astar(qmap,start,end #path between the points
        if path_list==False:
                dict_val=["NO PATH",[],0] #if no path possible

        else:
                for (x,y) in path_list:
                        dict_val_inner=dict_val_inner + [(y+1,x+1)] #changing coordinates back to our required format
                dict_val_inner.reverse()    #reversing the list as we got the reversed path
                dict_val=dict_val + [dict_val_inner[-1]] #first value of list is the goal point,which was last value in our list containing the path
                dict_val=dict_val+ [dict_val_inner[0:-1]] #the inner list which contains the path
                dict_val=dict_val+ [len(dict_val_inner)]    #length of our path
        return dict_val

    
'''
Function Name   :main
Input           :image_filename->location of our test images
Output          :occupied_grids->list of all grids which contain either obstacle or our shapes
                 planned_path->dictionary containing the starting point,the end point and path to it,length of the path
Logic           :We devide our 10x10 grid into 100 pieces.Find shape,colour ,area,perimeter of occupied objects in the grids
                 Then store all information in a dictionary with key as the coloured objects and value as a list of above mentioned quantities.
                 Then we match objects on basis of these attributes and find shortest path between them.
                 We have used A* algorithm to achieve the shortest path.
Example Call    :main(image_filename)
'''

def main(image_filename):
        '''
        This function is the main program which takes image of test_images as argument. 
        Team is expected to insert their part of code as required to solve the given 
        task (function calls etc).

        ***DO NOT EDIT THE FUNCTION NAME. Leave it as main****
        Function name: main()

        ******DO NOT EDIT name of these argument*******
        Input argument: image_filename

        Return:
        1 - List of tuples which is the coordinates for occupied grid. See Task2_Description for detail. 
        2 - Dictionary with information of path. See Task2_Description for detail.
        '''

        occupied_grids = []             # List to store coordinates of occupied grid -- DO NOT CHANGE VARIABLE NAME
        planned_path = {}               # Dictionary to store information regarding path planning       -- DO NOT CHANGE VARIABLE NAME
        occupied_dict={}   # A dictionary with key as the position of coloured objects and value as a list of colour ,area,perimeter of occupied objects in the grids

        ##### WRITE YOUR CODE HERE - STARTS

        # cv2.imshow("board_filepath - press Esc to close",cv2.imread(board_filepath))                  - For check - remove
        # cv2.imshow("container_filepath - press Esc to close",cv2.imread(container_filepath))
        image = cv2.imread(image_filename)  #our main image
        (x,y,z) = image.shape
        ##HSV
        img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)    #converted our image to HSV colour code.It gives good control over Hue and Saturation
        lower_red = np.array([0,50,50])
        upper_red = np.array([10,255,255])
        mask0 = cv2.inRange(img_hsv, lower_red, upper_red)  #Range1 of red colour values to be removed
        lower_red = np.array([170,50,50])
        upper_red = np.array([180,255,255])
        mask1 = cv2.inRange(img_hsv, lower_red, upper_red)  #Range2 of red colour values to be removed
        lower_blue = np.array([110,50,50])
        upper_blue = np.array([130,255,255])
        mask2 = cv2.inRange(img_hsv, lower_blue, upper_blue)    #Range of blue colour values to be removed
        lower_green = np.array([50,100,100])
        upper_green = np.array([70,255,255])
        mask3 = cv2.inRange(img_hsv, lower_green, upper_green)  #Range of green colour values to be removed
        lower_yellow = np.array([20,240,240]) 
        upper_yellow = np.array([40,255,255])
        mask4 = cv2.inRange(img_hsv, lower_yellow, upper_yellow)    ##Range of yellow colour values to be removed
        mask=mask4+mask3+mask2+mask1+mask0                  #Final mask

        
        ##Loop for Shape,occupied objets,colour
        ##this loop devides our image into 100 pieces and then evaluate properties of each
        for i in range (10):
                for j in range (10):
                        img1=image[(((j*x)/10)):((((j+1)*x)/10)),(((i*y)/10)):((((i+1)*y)/10)),:]
                        (p,q,s)=img1.shape  #dimentions of the smaller piece
                        (B,G,R)=img1[p/2,q/2]   #Colour of the Center Pixel.
                        if ( not (B>200 and G>200 and R>200)):  #if colour of center pixel is not white,then it is an occupied grid
                                occupied_grids = occupied_grids + [(i+1,j+1)]
                                if (not (B<50 and G<50 and R<50)):  #If it is not Black,then its a coloured object so we find other properties
                                        key =(i+1,j+1)
                                        occupied_dict.setdefault(key, [])    
                                        ##shape
                                        img2=mask[(((j*x)/10)):((((j+1)*x)/10)),(((i*y)/10)):((((i+1)*y)/10))]
                                        #to apply contours,we use the final mask which we obtained and devide it into 100 pieces to find properties of each grid
                                        contours,h = cv2.findContours(img2,1,2)
                                        for cnt in contours:
                                                #We commented this code of finding the shape because we used the fact that if
                                                #area and perimeter of two regular polygons are same,then they will have same shape.So we skipped this step
                                                '''approx = cv2.approxPolyDP(cnt,0.02*cv2.arcLength(cnt,True),True)
                                                if len(approx)==3:
                                                        occupied_dict[key].append('triangle')
                                                elif len(approx)==4:
                                                        occupied_dict[key].append('4-sided')
                                                elif len(approx) > 7:
                                                        occupied_dict[key].append('circle')'''
                                                occupied_dict[key].append(cv2.contourArea(cnt)) #append area of the shape
                                                occupied_dict[key].append(cv2.arcLength(cnt,True))   #append perimeter of the shape
                                        #colour
                                        if (B<50 and G<50 and R>200):
                                                occupied_dict[key].append('red')
                                        if (B<50 and R<50 and G>200):
                                                occupied_dict[key].append('green')
                                        if (G<50 and R<50 and B>200):
                                                occupied_dict[key].append('blue')
                                        if (B<50 and R>200 and G>200):
                                                occupied_dict[key].append('yellow')

        key_list=occupied_dict.keys()   #list of coloured grids
        value_list=occupied_dict.values() #list of area perimeter and colour of corresponding coloured grid
        for i in range(len(key_list)):
                match_dict={}   #Dictionary to keep track of shapes with more than one matching entities
                for j in range(len(key_list)):
                        if value_list[i]==value_list[j] and i!=j:   #matching keys with same values
                                path=path_planner(key_list[i],key_list[j],occupied_grids)   #finding path and path length between them
                                match_dict[key_list[j]]=path[-1]    #storing each matched key with its path length as value
                if len(match_dict)==0:  #if dictionary is empty,it means there is no matching shape
                        path=["NO MATCH",[],0]
                else:
                        val=sorted(match_dict.values()) #we sort the values of this dict
                        mini=val[0] #to obtaint the smallest path length
                        for key,value in match_dict.iteritems():
                                if value == mini:
                                        end=key         #and hence find the grid which will be nearest to the starting point
                        path=path_planner(key_list[i],end,occupied_grids)
                planned_path[key_list[i]]=path

image_filename = "test_images/test_image1.jpg"

main(image_filename)

cv2.waitKey(0)
cv2.destroyAllWindows()
