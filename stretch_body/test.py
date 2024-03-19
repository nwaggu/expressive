#perlin setup
import itertools
import random
import math
import time
import matplotlib.pyplot as plt 
import sys 
import numpy as np 
from waiting import wait 
import math
from scipy.interpolate import interp1d
#robot control
import stretch_body.robot

sys.path.insert(1, '/home/hello-robot/catkin_ws/src/internal_expression/src/stretch_body/perlin-1d')
from perlin_noise import PerlinNoise, Interp

#Random Setup
random.seed(time.time())

# define constants
WIDTH, HEIGHT = (10, 600)
FONT_SIZE = 14
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)

# init pygame
# pygame.init()
# screen = pygame.display.set_mode((WIDTH, HEIGHT))
# pygame.display.set_caption('1D Perlin Noise') 
# font = pygame.font.Font(pygame.font.get_default_font(), FONT_SIZE)

# set initial parametrs for program
seed = random.randint(0, 2**32)
amplitude = 0.12
frequency = 0.3
octaves = 1
# number of integer values on screen
# (implicit frequency)
segments = 40
interpolation = Interp.CUBIC
interp_iter = itertools.cycle((Interp.LINEAR, Interp.CUBIC, Interp.COSINE))
offset = 0
offset_speed = 2
show_marks = False

running = True

# create PerlinNoise object


# print information about program parametrs


class ExpControl():
    
    def __init__(self):
        self.robot=stretch_body.robot.Robot()
        self.robot.startup()
        self.noise = PerlinNoise(seed, amplitude, frequency, octaves, interpolation)
        #h = stretch_body.head.Head()


    def reset(self):
        #self.robot.startup()
        self.robot.lift.motor.enable_pos_pid()
        self.robot.arm.motor.enable_pos_pid()
        self.robot.arm.move_to(0.1)
        self.robot.lift.move_to(0.8)
        self.robot.push_command()
        time.sleep(5)
        #self.robot.end_of_arm.pose('joint_wrist_roll', 0)
        self.robot.end_of_arm.move_to('wrist_yaw', math.pi/2)
        self.robot.end_of_arm.move_to('wrist_roll', 0)
        self.robot.end_of_arm.move_to('wrist_pitch', 0)
        print(self.robot.end_of_arm.get_joint_configuration())


    def _speedPerlin(self,times, set, i):
        h = 0.00000000001
        if i == len(set)-1:
            return 0
        else: 
            top = set[i+1] - set[i]
            bottom = times[i+1] - times[i]
            slope = top / bottom    # Returns the slope to the third decimal
            if slope > 0.15:
                return 0.149
            elif slope < -0.15:
                return -0.149
            elif abs(slope) < 0.01:
                return 0.01*(slope/abs(slope))
            else:
                return float("%.3f" % slope)
    
    def createPerlinNoise(self, linspace, reference):
        #Stored y values
        points = []
        #Generate perlin y values
        for var in linspace:
            # get perlin noise value
            y = self.noise.get(var) + reference
            # check is x value integer in Perlin noise coordinates
            #real_x = x * noise.frequency
            points.append(y)
        plt.plot(linspace, points)
        plt.show()
        return linspace,points

    def position_in_tolerance(self, position, target):
        if abs(target-position) <= 0.05:
            return True
        return False

    

    def secondaryMotion(self, reference):
        #Define the waypoints
        x = np.linspace(0,50,500)
        times, positions = self.createPerlinNoise(x, reference)
        #Move to tarting position
        self.robot.lift.motor.enable_pos_pid()
        self.robot.lift.move_to(reference)
        self.robot.push_command()
        self.robot.lift.wait_until_at_setpoint()
        #Switch motor mode
        self.robot.lift.motor.enable_vel_traj()
        current_target = positions[0]
        for i in range(len(times)):
            speed = self._speedPerlin(times,positions,i)
            self.robot.lift.set_velocity(speed)
            self.robot.push_command()
            time.sleep(1/80)
            wait(lambda: self.position_in_tolerance(self.robot.lift.status['pos'], current_target), sleep_seconds=0.1)
            
            if i != len(times)-1:
                current_target = positions[i+1]
        #Turn off the velocity :)
        self.robot.lift.set_velocity(0)
    
    def generateSine():
        cycles = 2 # how many sine cycles
        resolution = 25 # how many datapoints to generate

        length = np.pi * 2 * cycles
        my_wave = np.sin(np.arange(0, length, length / resolution))
        return my_wave



    def limping(self):
        self.robot.lift.motor.enable_vel_traj()
        top = 0.7
        bottom = 0.5
        fast = -0.15
        slow = 0.05
        bit = 0
        flag =0
        position = self.robot.lift.status['pos']
        position_list = []

        self.robot.base.translate_by(10)

        conversion = interp1d([bottom, top],[math.pi/5,-math.pi/3])
        self.robot.end_of_arm.move_to('wrist_pitch', 0)
        time_start = time.time()

        while time.time() <= time_start + 120:
            if bit == 0 and not flag:
                self.robot.lift.set_velocity(-0.15)
                self.robot.push_command()
                flag = 1
             
            elif bit ==1 and not flag:
                self.robot.lift.set_velocity(0.05)
                self.robot.push_command()
                flag = 1

            if flag:  
                if bit == 1: 
                    if self.position_in_tolerance(self.robot.lift.status['pos'], top):
                        bit = 0
                        flag = 0
                else: 
                    if self.position_in_tolerance(self.robot.lift.status['pos'], bottom):
                        bit =1
                        flag=0
            position_list.append(self.robot.lift.status['pos'])
            if time.time() >= time_start + 0.3:
                pos = position_list.pop()
                if pos > top:
                    pos = top
                if pos < bottom: 
                    pos = bottom
                
                angle = conversion(pos)
                print(angle)
                self.robot.end_of_arm.move_to('wrist_pitch', angle)
            time.sleep(1/80)    
            #self.robot.lift.set_velocity(-0.04)
        self.robot.lift.set_velocity(0)
    

    def simpleLowVelocity(self):
        conversion = interp1d([1, 0],[0,-math.pi/4])
        conversion2 = interp1d([0, 1],[0,math.pi/2])
        self.robot.base.set_translate_velocity(0.1)
        #self.robot.base.translate_by(0.1)
        #self.robot.push_command()
        #self.robot.lift.tra
        #times = [0.0, 1.0, 8.0]
        #positions = [self.robot.lift.status['pos'], self.robot.lift.status['pos']+0.03, 0.1]
        #velocities = [self.robot.lift.status['vel'], 0.0, 0.0]
        self.robot.lift.motor.enable_vel_traj()
        self.robot.lift.set_velocity(0.02)
        self.robot.push_command()
        
        wait(lambda: self.position_in_tolerance(self.robot.lift.status['pos'], 0.9), sleep_seconds=1/80)
        self.robot.lift.set_velocity(-0.13)
        self.robot.push_command()
        pos = self.robot.lift.status['pos']
        angle = conversion(pos)
        self.robot.end_of_arm.move_to('wrist_pitch', angle)
        angle2 = conversion2(pos)
        self.robot.end_of_arm.move_to('wrist_yaw', angle2)

        wait(lambda: self.position_in_tolerance(self.robot.lift.status['pos'], 0.85), sleep_seconds=1/80)
        self.robot.lift.set_velocity(-0.1)
        self.robot.push_command()
        pos = self.robot.lift.status['pos']
        angle = conversion(pos)
        self.robot.end_of_arm.move_to('wrist_pitch', angle)
        angle2 = conversion2(pos)
        self.robot.end_of_arm.move_to('wrist_yaw', angle2)

        wait(lambda: self.position_in_tolerance(self.robot.lift.status['pos'], 0.75), sleep_seconds=1/80)
        self.robot.lift.set_velocity(-0.04)
        self.robot.push_command()
        pos = self.robot.lift.status['pos']
        angle = conversion(pos)
        self.robot.end_of_arm.move_to('wrist_pitch', angle)
        angle2 = conversion2(pos)
        self.robot.end_of_arm.move_to('wrist_yaw', angle2)

        wait(lambda: self.position_in_tolerance(self.robot.lift.status['pos'], 0.2), sleep_seconds=1/80)
        self.robot.lift.set_velocity(-0.02)
        self.robot.push_command()
        pos = self.robot.lift.status['pos']
        angle = conversion(pos)
        self.robot.end_of_arm.move_to('wrist_pitch', angle)
        angle2 = conversion2(pos)
        self.robot.end_of_arm.move_to('wrist_yaw', angle2)

        wait(lambda: self.position_in_tolerance(self.robot.lift.status['pos'], 0.11), sleep_seconds=1/80)
        pos = self.robot.lift.status['pos']
        angle = conversion(pos)
        self.robot.end_of_arm.move_to('wrist_pitch', angle)
        angle2 = conversion2(pos)
        self.robot.end_of_arm.move_to('wrist_yaw', angle2)
        self.robot.stop()
        
    def moveForward(self, speed):
        
        self.robot.base.set_velocity(speed,0)
        self.robot.push_command()
        time.sleep(3)
        self.robot.base.set_velocity(0,0)



        #Create the spline trajectory
        #for waypoint in zip(times, positions,velocities):
        #    self.robot.lift.trajectory.add(waypoint[0], waypoint[1],waypoint[2])

        #self.robot.lift.follow_trajectory()
        #while self.robot.lift.is_trajectory_active():
        #    print('Execution time: %f'%self.robot.lift.get_trajectory_time_remaining())
        #    time.sleep(0.1)
       #self.robot.base.stop()
        #print("???")
       # self.robot.base.startup()
        
        

   


    #def __del__(self):
    #    print("Destructor Called Program Ending")
        

exp = ExpControl()

exp.reset()
#exp.moveForward(0.5)
#time.sleep(5)
exp.secondaryMotion(0.7)


#Create the spline trajectory
# for waypoint in zip(times, positions):
#     robot.lift.trajectory.add(waypoint[0], waypoint[1])
#     print("added")
# #robot.arm.trajectory.add(x_m=positions,t_s=times)

# #Begin execution
# robot.lift.follow_trajectory()

# #Wait unti completion
# while robot.lift.is_trajectory_active():
#     print('Execution time: %f'%robot.lift.get_trajectory_time_remaining())
#     time.sleep(0.1)



# draw lines and update display
#pygame.draw.lines(screen, BLACK, False, points)
#pygame.display.flip()

# move Perlin noise
#offset += offset_speed

#pygame.quit()