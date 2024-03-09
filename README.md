# Expressing Internal States For Robots 


Representing the internal states of robots is important for HRC. Particularly, it can ensure some level of knowledge of the robot's current task or operation so that people working with it are on the same page. This is especially important when working in close proximity considering the possibility that unawareness of the robot's state could lead to incorrect assessments resulting in injury or damaging of robotic equipment. Motion is chosen over other modes of communication beacause of sound limitations often placed in industrial workspaces\cite{}

## Robot Communication Paradigm

\citet{cha_dissertation} talks about a paradigm for understanding how and when to expressive different robot behaviors depending on the sitution. This was originally created when thinking about robotic sound. Cha describes 4 different robot responses on a necessary versus urgent graph. 


| Type        | Definition                                                                      | Example Situation    |
| :---        | :----                                                                           |        ---: |
| Informative | Conveying information for the user that does not require direct intervention.   | Robot idling, indicating task completetion or panning to indicate planning |
| Beneficial  | Conveying information that would aid the robot directly in its task.            | Conveying next action to avoid user collisions |
| Necessary   | Intervention is needed by the user at some future point in time, doesn't interrupt operation.            | The robot is running low on battery, needs to be charged    |
| Urgent      | User intervention is needed immediately. | The robot failure has some potentional to cause harm or large discrepancies from target task or generally robot is unable to accomplish its task!  |

### Examples
Because the paradigm is previously associated with robot sound there's some translation that has to be made. Examples of the different types we could look to implement relative to the types above are listed below.

Informative:
* Completed Planning
* Panning/Searching Behavior
    * Turn and Quick Pans w/ Camera
* ==Idling/ "Breathing"/ "On"==
    * Simple Perlin Noise Implementation on Lift

Beneficial:
* ==Intent of Future Motions (IoFM)==
    * Camera Following Rotation
    * Turn Signals With a combination of the arm and extension
* Recoverable Task Failures(?) - Asking For Help
    * State of the art says try to mimic what u want to do without moving end goal [cite Kwon] but that's actually impossible with the arm of the Stretch


Neccessary: 
* ==Low Battery==
    * We're moving to point then switching and continuing
* ==Routine Maintenance(?)== 
    * Thinking Swapping Roomba Vaccuum Cartridge/ Cleaning Brushes

Urgent:
* ==Localization(?) Failure==
* Sensor(?) Failure

### Pilot Study Insights
See the full paper for the specific results of the study. Most of the existing insights however are collected here (some not present in the original document),


#### Transitions
I think it might be important to have some level of transitions when representing one state to the next. One of the major things from the pilot study was this notion that it wasn't even really clear when certain behaviors were starting/happening. The distinctions between them were not clear at all. Another insight from the study was the need for urgent behaviors to **completely** stop the current task. People made comments about how they kind of assumed things were okay with the limping motion because of this. 

#### Camera
The camera's movement's aren't very noticeable if you're not looking for it. It may be important to on some level try to emphasize it. I was thinking that a good way might be to stage the task in such a way that Stretch can 'look' at participants during necessary behaviors to emphasize this eventual need to do something about it. Otherwise, maybe we could move the lift up and use the wrist to mimic/enforce its movement? Idk quite yet.

#### 'Overlay' vs 'Add-On'
One thing that's particularly interesting is understanding how the some behaviors actually augement characteristics of a robot's mode of operation. For example, there are specific behaviors like low battery that augments beavhior (e.g. making everything slower, sluggish) and ones that we've modeled being essentially "add-ons" with the camera movement of the IoFM.  



### Technical Details

There's a few things we'd need to setup or get in place in roder to ensure we can actually run this study. Notably with the eye contact stuff, we'd need openpose to be able to run on the system. Otherwise, if we're using Python figuring out how we can transalte some of that existing path planning. 