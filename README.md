# RRT Implementation in 2D/3D
An implementation of RRT for 2D cases, such as with differential drive robots or cars.

## Short about RRT
Rapidly-exploring Random Trees (RRT) is a motion planning for robotics and other automated systems. The basic idea behind RRT is to construct a tree data structure by repeatedly sampling random configurations and adding them to the tree, while also connecting nearby configurations to form edges. RRT is designed to efficiently explore high-dimensional configuration spaces, and can quickly generate feasible paths through complex environments. The algorithm is widely used in autonomous vehicle navigation, industrial automation, and other robotics applications. The Algorithm can be altered a bit to suit the end users specific case, where e.g. goal bias can be used to speed up the search. 

## The implementation 
The implementation of RRT is within the file "..." and is constructed as a class for easy implementation in other projects, where the class can be imported into another program and then be called from there. The Implementation is made for Holonomic robots and there is therefore not a steering function implemented within the code, to use the class for non-holonomic robots a steering function could be implemented within the class where the new node is created.

## Difference between 3D and 2D
There are two implementations for RRT in this repository, where there are only small differences, I would recomend to check through both files to see the differences


## How to use
COMMING
