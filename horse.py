#Imports
import numpy as np
import pybullet as p
import pyrosim.pyrosim as pyrosim
import constants as c
import random
import os
import time

class HORSE:
    def __init__(self, nextAvailableID):
        os.system("rm brain*.nndf")
        os.system("rm fitness*.txt")
        
        self.myID = nextAvailableID

    
    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.End()


    def Create_HorseBody(self):
        #Define body plan of horse
        self.torsoLinks = np.random.randint(2,6)
        self.numLegs = 2*self.torsoLinks
        print(self.torsoLinks)
        
        #Determine how many sensors and motors
        self.sensor_coins = np.random.randint(0,2, size = self.torsoLinks + self.numLegs)
        self.motor_coins = np.random.randint(0,2, size = self.torsoLinks + self.numLegs- 1)
        
        
        #Start simulation by creating horse's head and first link
        initialZ = 4.0
        
        l_parent = random.uniform(0.1,3.0)
        w_parent = random.uniform(0.1,3.0)
        h_parent = random.uniform(0.0,3.0)
        l_child = random.uniform(0.1,3.0)
        w_child = random.uniform(0.1,3.0)
        h_child = random.uniform(0.1,3.0)
        
        l_parentLeg = random.uniform(0.1,l_parent/2)
        w_parentLeg = random.uniform(0.1,w_parent/2.0)
        h_parentLeg = random.uniform(0.1,h_parent)
        l_childLeg = random.uniform(0.1,l_child/2.0)
        w_childLeg = random.uniform(0.1,w_child/2.0)
        h_childLeg = random.uniform(0.1,h_child)

        pyrosim.Start_URDF("body.urdf")
        

        if self.sensor_coins[0] == 1:
            pyrosim.Send_Cube(name="Torso", pos=[0,0,initialZ] , size=[l_parent,w_parent,h_parent], g_value = 1.0, b_value = 0.0)
        elif self.sensor_coins[0] == 0:
            pyrosim.Send_Cube(name="Torso", pos=[0,0,initialZ] , size=[l_parent,w_parent,h_parent], g_value = 0.0, b_value = 1.0)
        
        
        pyrosim.Send_Joint(name = "Torso_0" , parent= "Torso" , child = "0" , type = "revolute", position = [0,0,initialZ-h_parent/2.0], jointAxis = "0 1 0")
    
        if self.sensor_coins[1] == 1:
            pyrosim.Send_Cube(name="0", pos=[0,w_parent/2.0,-h_parentLeg/2.0] , size=[l_parent/2.0,w_parent/2.0,h_parentLeg], g_value = 1.0, b_value = 0.0)
        elif self.sensor_coins[1] == 0:
            pyrosim.Send_Cube(name="0", pos=[0,w_parent/2.0,-h_parentLeg/2.0] , size=[l_parent/2.0,w_parent/2.0,h_parentLeg], g_value = 0.0, b_value = 1.0)
        
        
        pyrosim.Send_Joint(name = "Torso_1" , parent= "Torso" , child = "1" , type = "revolute", position = [0,0,initialZ-h_parent/2.0], jointAxis = "0 1 0")

        if self.sensor_coins[2] == 1:
            pyrosim.Send_Cube(name="1", pos=[0,-w_parent/2.0,-h_parentLeg/2.0] , size=[l_parent/2.0,w_parent/2.0,h_parentLeg], g_value = 1.0, b_value = 0.0)
        elif self.sensor_coins[2] == 0:
            pyrosim.Send_Cube(name="1", pos=[0,-w_parent/2.0,-h_parentLeg/2.0] , size=[l_parent/2.0,w_parent/2.0,h_parentLeg], g_value = 0.0, b_value = 1.0)
        
        
        pyrosim.Send_Joint(name = "Torso_2" , parent= "Torso" , child = "2" , type = "revolute", position = [l_parent/2,0,initialZ], jointAxis = "0 1 0")
        
        if self.sensor_coins[3] == 1:
            pyrosim.Send_Cube(name="2", pos=[l_child/2,0,0] , size=[l_child,w_child,h_child], g_value = 1.0, b_value = 0.0)
        elif self.sensor_coins[3] == 0:
            pyrosim.Send_Cube(name="2", pos=[l_child/2,0,0] , size=[l_child,w_child,h_child], g_value = 0.0, b_value = 1.0)
        
        
        pyrosim.Send_Joint(name = "2_3" , parent= "2" , child = "3" , type = "revolute", position = [l_child/2.0,w_child/2.0,-h_child/2.0], jointAxis = "0 1 0")
    
        if self.sensor_coins[4] == 1:
            pyrosim.Send_Cube(name="3", pos=[0,0,-h_childLeg/2.0] , size=[l_child/2.0,w_child/2.0,h_childLeg], g_value = 1.0, b_value = 0.0)
        elif self.sensor_coins[4] == 0:
            pyrosim.Send_Cube(name="3", pos=[0,0,-h_childLeg/2.0] , size=[l_child/2.0,w_child/2.0,h_childLeg], g_value = 0.0, b_value = 1.0)
        
        
        pyrosim.Send_Joint(name = "2_4" , parent= "2" , child = "4" , type = "revolute", position = [l_child/2.0,-w_child/2.0,-h_child/2.0], jointAxis = "0 1 0")

        if self.sensor_coins[5] == 1:
            pyrosim.Send_Cube(name="4", pos=[0,0,-h_childLeg/2.0] , size=[l_child/2.0,w_child/2.0,h_childLeg], g_value = 1.0, b_value = 0.0)
        elif self.sensor_coins[5] == 0:
            pyrosim.Send_Cube(name="4", pos=[0,0,-h_childLeg/2.0] , size=[l_child/2.0,w_child/2.0,h_childLeg], g_value = 0.0, b_value = 1.0)
        
        
        
        #Create additional links
        if self.torsoLinks > 2:
            for linkNumber in range(3,3*(self.torsoLinks-1),3):
                l_parent = l_child
                w_parent = w_child
                h_parent = h_child
                l_child = random.uniform(0.1,3.0)
                w_child = random.uniform(0.1,3.0)
                h_child = random.uniform(0.1,3.0)
                
                l_parentLeg = l_childLeg
                w_parentLeg = w_childLeg
                h_parentLeg = h_childLeg
                l_childLeg = random.uniform(0.1,l_child/2.0)
                w_childLeg = random.uniform(0.1,w_child/2.0)
                h_childLeg = random.uniform(0.1,h_child)
                
                
                
                if self.sensor_coins[linkNumber+3] == 1:
                    pyrosim.Send_Joint(name = str(linkNumber-1) + "_" + str(linkNumber+2) , parent= str(linkNumber-1) , child = str(linkNumber+2) , type = "revolute", position = [l_parent,0,0], jointAxis = "0 1 0")
                    pyrosim.Send_Cube(name=str(linkNumber+2), pos=[l_child/2,0,0] , size=[l_child,w_child,h_child], g_value = 1.0, b_value = 0.0)
                    
                else:
                    pyrosim.Send_Joint(name = str(linkNumber-1) + "_" + str(linkNumber+2) , parent= str(linkNumber-1) , child = str(linkNumber+2) , type = "revolute", position = [l_parent,0,0], jointAxis = "0 1 0")
                    pyrosim.Send_Cube(name=str(linkNumber+2), pos=[l_child/2,0,0] , size=[l_child,w_child,h_child], g_value = 0.0, b_value = 1.0)
        
                
                if self.sensor_coins[linkNumber+4] == 1:
                    pyrosim.Send_Joint(name = str(linkNumber+2) + "_" + str(linkNumber+3) , parent= str(linkNumber+2) , child = str(linkNumber+3) , type = "revolute", position = [l_child/2.0,w_child/2.0,-h_child/2.0], jointAxis = "0 1 0")
                    pyrosim.Send_Cube(name=str(linkNumber+3), pos=[0,0,-h_childLeg/2.0] , size=[l_child/2.0,w_child/2.0,h_childLeg], g_value = 1.0, b_value = 0.0)
                
                else:
                    pyrosim.Send_Joint(name = str(linkNumber+2) + "_" + str(linkNumber+3) , parent= str(linkNumber+2) , child = str(linkNumber+3) , type = "revolute", position = [l_child/2.0,w_child/2.0,-h_child/2.0], jointAxis = "0 1 0")
                    pyrosim.Send_Cube(name=str(linkNumber+3), pos=[0,0,-h_childLeg/2.0] , size=[l_child/2.0,w_child/2.0,h_childLeg], g_value = 0.0, b_value = 1.0)
                    
                
                if self.sensor_coins[linkNumber+5] == 1:
                    pyrosim.Send_Joint(name = str(linkNumber+2) + "_" + str(linkNumber+4) , parent= str(linkNumber+2) , child = str(linkNumber+4) , type = "revolute", position = [l_child/2.0,-w_child/2.0,-h_child/2.0], jointAxis = "0 1 0")
                    pyrosim.Send_Cube(name=str(linkNumber+4), pos=[0,0,-h_childLeg/2.0] , size=[l_child/2.0,w_child/2.0,h_childLeg], g_value = 1.0, b_value = 0.0)
                
                else:
                    pyrosim.Send_Joint(name = str(linkNumber+2) + "_" + str(linkNumber+4) , parent= str(linkNumber+2) , child = str(linkNumber+4) , type = "revolute", position = [l_child/2.0,-w_child/2.0,-h_child/2.0], jointAxis = "0 1 0")
                    pyrosim.Send_Cube(name=str(linkNumber+4), pos=[0,0,-h_childLeg/2.0] , size=[l_child/2.0,w_child/2.0,h_childLeg], g_value = 0.0, b_value = 1.0)

        
        #End
        pyrosim.End()
        
        
    def Create_HorseBrain(self):      
        #Start neural network
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
        
        
        #Count sensors and motors
        number_sensors = np.count_nonzero(self.sensor_coins)
        number_motors = np.count_nonzero(self.motor_coins)
        
        sensors = []
        motors = []
        
        for i, sensor in enumerate(self.sensor_coins):
            if sensor == 1:
                sensors.append(i)
        
        for j, motor in enumerate(self.motor_coins):
            if motor == 1:
                motors.append(j)
                
        print(sensors)
        print(motors)
        
        #Establish weights for synapses        
        self.weights = np.random.rand(number_sensors,number_motors)
        self.weights = 2*self.weights - 1
        
        
        #Send sensors
        for linkNumber in range(0,self.torsoLinks + self.numLegs):
            if self.sensor_coins[linkNumber] == 1:
                if linkNumber == 0:
                    pyrosim.Send_Sensor_Neuron(name = linkNumber, linkName = "Torso")
                else:
                    pyrosim.Send_Sensor_Neuron(name = linkNumber, linkName = str(linkNumber-1))
        
        #Send motors
        for jointNumber in range(0,self.torsoLinks + self.numLegs - 1,3):
            if self.motor_coins[jointNumber] == 1:
                if jointNumber == 0:
                    pyrosim.Send_Motor_Neuron(name = jointNumber + number_sensors, jointName = "Torso_0")
                    pyrosim.Send_Motor_Neuron(name = jointNumber + number_sensors + 1, jointName = "Torso_1")
                    pyrosim.Send_Motor_Neuron(name = jointNumber + number_sensors + 2, jointName = "Torso_2")
                if jointNumber > 0:
                    pyrosim.Send_Motor_Neuron(name = jointNumber + number_sensors, jointName = str(jointNumber-1) + "_" + str(jointNumber))
                    pyrosim.Send_Motor_Neuron(name = jointNumber + number_sensors + 1, jointName = str(jointNumber-1) + "_" + str(jointNumber+1))


        #Send synapses
        for currentRow, source in enumerate(sensors):
            for currentColumn, target in enumerate(motors):
                pyrosim.Send_Synapse(sourceNeuronName = source, targetNeuronName = target+number_sensors, weight = self.weights[currentRow][currentColumn])  

    
        #End
        pyrosim.End()

     
    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        self.Create_HorseBody()
        self.Create_HorseBrain()
        
        os.system("python3 simulate.py " + str(directOrGUI) + " " + str(self.myID) + ' 2>&1')
        