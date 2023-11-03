
import numpy as np

class RRT2D():
    
    class Node():
        
        def __init__(self,position='',parent=''): # '' could be None istead or False
            self.parent = parent
            self.position = position #np.array([0, 0, 0])
    
    def __init__(self,start,goal,obstacles,stepSize = 0.05, explorationBias = 0.9):
        self.tree = []
        self.solutionPath = []
        self.explore = explorationBias #exploration bias
        self.start = np.array(start)
        self.goal = np.array(goal[0][0:2])
        self.obstacles = obstacles # [[[position],radius],[]]
        self.space = np.array([-4.7,4.7,-4.7,4.7]) #x_min, x_max, y_min, y_max, z_min, z_max
        self.stepSize = stepSize
        self.goalDist = goal[1]
        self.baseRadius = 0.4
        
        
    def qRandomGenerator(self):
        xRand = np.random.uniform(low=self.space[0], high=self.space[1], size=(1,))
        yRand = np.random.uniform(low=self.space[2], high=self.space[3], size=(1,))
        qCords = [xRand[0],yRand[0]]
        
        goalCords = np.array(self.goal)
        
        arrayPick = np.array([0,1])
        arrayChoise = [qCords,goalCords]
        
        rand = np.random.choice(arrayPick, 1, p=[self.explore, 1-self.explore])
        qRand = arrayChoise[rand[0]]
        return qRand
    
    
    def createTree(self):
        if len(self.tree) == 0:
            node = self.Node(position=np.array(self.start))
            self.tree.append(node)
            
        goalReched = False
        
        while not goalReched:
            goalReched = self.addNode()
            #if len(self.tree) > 2000:
                #print(self.findNearest([0,0]).position)
        node = self.tree[-1]
        self.solutionPath.append(node.position)
        
        while node.parent != '':
            node = node.parent
            self.solutionPath.append(node.position)
            
        
    def addNode(self):
        nodeAdded = False
        while not nodeAdded:
            qRand = self.qRandomGenerator()
            parent = self.findNearest(qRand)
            qNew = np.array(((qRand-parent.position)/(np.linalg.norm(qRand-parent.position))))*self.stepSize + np.array(parent.position)
            qNew = np.round(qNew,4)
            if self.collisionCheck(qNew,parent.position):
                node = self.Node(position=qNew,parent=parent)
                self.tree.append(node)
                nodeAdded = True
                dist = self.calculateDistance(qNew,self.goal)
                if dist <= self.goalDist:
                    return True
        return False
        
        
    def findNearest(self,position):
        minDist = 10000
        count = 0
        for i in range(len(self.tree)):
            count += 1
            dist = self.calculateDistance(position,self.tree[i].position)
            if dist < minDist:
                minDist = dist
                idx = i
        parent = self.tree[idx]
        return parent
    
    
    def calculateDistance(self,position1,position2):
        dist = np.sqrt(((position2[0] - position1[0])**2)+\
                       ((position2[1] - position1[1])**2))
                            
        return dist
          
    
    def collisionCheck(self,position,positionBase):
        if len(self.obstacles) > 0:
            for i in range(len(self.obstacles)):
                position1 = self.obstacles[i][0]
                position2 = positionBase
                if self.calculateDistance(position1,position2) <= (self.obstacles[i][1]+self.baseRadius):
                    return False
        return True
            
        
        