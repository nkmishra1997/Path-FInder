# Path-FInder
Python Program to find shortest path between matching objects in an image.

# Given:
A set of test images, each containing
* 10x10 grid, making 100 squares 
* Obstacles marked as black square 
* Objects defined by three features, viz. Shape, Sizeand Color.

# The code performs:

* Finds the coordinates of occupied grid.
* For each object in the test_images, finds a matching object which is nearest to it. Object is said to be nearest to another Object, if length of path traversed between two objects is smallest. Traversal is done by moving either horizontally or vertically. The length of the path is determined by the number of moves made during traversal.

# Example:

![capture1](https://cloud.githubusercontent.com/assets/14962324/21743385/128b4e52-d527-11e6-9d83-7582c27686f2.PNG)

For this image,the output for occupied_objects is expected as:
[(1, 2), (1, 3), (1, 4), (1, 5), (2, 2), (2, 5), (3, 2), (3, 8), (4, 9), (5, 2), (5, 5), (5, 6), (6, 7), (7, 7), (7, 8), (8, 5)] 


![capture2](https://cloud.githubusercontent.com/assets/14962324/21743396/3cf97434-d527-11e6-81ab-4a4414ecd7d2.PNG)

For this image,the output is expected as:

{‘(2,5)’: [(5,5), [(3,5), (4,5)], 3], ‘(5,5)’: [(2,5), [(4,5), (3,5)], 3], ‘(3,8)’: [(5,2), [(3,7), (3,6), (3,5), (3,4), (3,3), (4,3), (5,3)], 8], ‘(5,2)’: [(7,7), [(6,2) ,(7,2), (7,3), (7,4), (7,5), (7,6)], 7] }
