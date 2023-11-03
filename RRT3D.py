import numpy as np
import ikpy.chain

class RRT3D():
    
    class Node():
        
        def __init__(self,position='',parent=''): # '' could be None istead or False
            self.parent = parent
            self.position = position #np.array([0, 0, 0])
    
    def __init__(self,start,ob,goal,obstacles,stepSize = 0.01,explorationBias = 0.6):
        self.ob = ob
        self.tree = []
        self.solutionPath = []
        self.basePosition = np.array([-0.15, 0, 0.487+0.14]) #link0 position relative to true base, from urdf file
        self.explore = explorationBias
        self.start = np.array(start)
        self.goal = np.array(goal[0])-np.array([ob['robot_0']['joint_state']['position'][0],ob['robot_0']['joint_state']['position'][1],0])
        self.obstacles = self.reformatObstacles(obstacles) # [[[position],radius],[]]
        self.space = np.array([-1.,2.,-2.,+2.,-1.,2.]) #x_min, x_max, y_min, y_max, z_min, z_max
        
        self.stepSize = stepSize
        self.goalDist = 0.05#goal[1]
        self.jointsRadius = 0.05
        self.chain = ikpy.chain.Chain.from_urdf_file("../urdfenvs/robots/albert/albert.urdf",\
                                                     base_elements=["mmrobot_link0"],\
                                                     active_links_mask=[True,True,True,True,True,True,True,False,False,False,False])
    def reformatObstacles(self,obstacles):
        for i in range(len(obstacles)):
            obstacles[i][0] = np.array(obstacles[i][0]) - np.array([self.ob['robot_0']['joint_state']['position'][0],self.ob['robot_0']['joint_state']['position'][1],0])
        return obstacles
    
    def qRandomGenerator(self):
        xRand = np.random.uniform(low=self.space[0], high=self.space[1], size=(1,))
        yRand = np.random.uniform(low=self.space[2], high=self.space[3], size=(1,))
        zRand = np.random.uniform(low=self.space[4], high=self.space[5], size=(1,))
        qCords = [xRand[0],yRand[0],zRand[0]]
        
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
            #qNew = qNew + np.array([self.ob['robot_0']['joint_state']['position'][0],self.ob['robot_0']['joint_state']['position'][1],0])
            if self.collisionCheck(qNew):
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
        
    def getJointPositions(self,position):
        jointPositions = []
        jointsMatricies = self.chain.forward_kinematics(self.chain.inverse_kinematics(position),full_kinematics=True)
        for i in range(1,11):
            tmp = [jointsMatricies[i][0][3],jointsMatricies[i][1][3],jointsMatricies[i][2][3]]
            jointPositions.append(tmp)
        return jointPositions
    
    
    def calculateDistance(self,position1,position2):
        dist = np.sqrt(((position2[0] - position1[0])**2)+\
                       ((position2[1] - position1[1])**2)+\
                       ((position2[2] - position1[2])**2))
                            
        return dist
          
    
    def collisionCheck(self,position):
        jointsPositions = self.getJointPositions(position)
        if len(self.obstacles) > 0:
            for i in range(len(self.obstacles)):
                position1 = self.obstacles[i][0]
                for j in range(len(jointsPositions)):
                    position2 = jointsPositions[j]
                    if self.calculateDistance(position1,position2) <= (self.obstacles[i][1]+self.jointsRadius):
                        return False
        return True
            
        
        