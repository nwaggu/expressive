#perlin setup
import itertools
import random
import math
import time
import matplotlib.pyplot as plt 
import sys 
import numpy as np 
from waiting import wait 
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
amplitude = 0.1
frequency = 0.4
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
    
    

    def simpleLowVelocity(self):
        #self.robot.base.set_translate_velocity(0.4)
        #self.robot.base.translate_by(0.1)
        #self.robot.push_command()
        #self.s
        #self.robot.lift.tra
        #times = [0.0, 1.0, 8.0]
        positions = [self.robot.lift.status['pos'], self.robot.lift.status['pos']+0.03, 0.1]
        velocities = [self.robot.lift.status['vel'], 0.0, 0.0]
        self.robot.lift.motor.enable_vel_traj()
        self.robot.lift.set_velocity(0.02)
        self.robot.push_command()
        wait(lambda: self.position_in_tolerance(self.robot.lift.status['pos'], 0.9), sleep_seconds=0.1)
        
       
        self.robot.lift.set_velocity(-0.15)
        self.robot.push_command()
        wait(lambda: self.position_in_tolerance(self.robot.lift.status['pos'], 0.1), sleep_seconds=0.1)
        
        
        

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
        
        

   


    def __del__(self):
        print("Destructor Called Program Ending")
        self.robot.stop()

exp = ExpControl()

exp.simpleLowVelocity()


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