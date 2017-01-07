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

Markup : ![picture alt](http://www.brightlightpictures.com/assets/images/portfolio/thethaw_header.jpg "Title is optional")
![Alt text](capture.png?raw=true "Optional Title")
