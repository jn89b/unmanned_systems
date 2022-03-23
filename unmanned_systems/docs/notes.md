# Actual World Settings
- 14ft x 14ft world
- 4.27m x 4.27m
1m = 0.3048m

# Goal 
start = [0,0]
goal = [3,2]
    

# Obstacle list
#this is in meters
obstacle_list = [[0,1], [0.5,1], [1,1],
                [2,0], [2,0.5], [2,1],
                [1,2], [1.5,1.5], [2,2]]


# Set up
- Position obstacles in freedom units
- Map the position obstacles to metric units
- 1 foot is 0.3048m
- Send waypoints to UAVS



# Set up?
- [] Map Turtlebot to meters from feet input 
- [] User publishes feet 
    - [] Turtlebot subscribes to freedom unit topic 
    - [] Turtlebot then converts freedom units to meters and sends to position command

# Situation
- Turtlebot at [0,0]
- Turtlebot goal at [8,10]
- Obstacle list: []


# SSH
Leonardo - 
    ssh ubuntu@192.168.1.127
    password: raspberry

Donatello - 
    ssh donatello@192.168.1.146
    password: raspberry


## How to log in to turtlebot
### On MASTER COMPUTER
- ROS master ip of laptop
    -export ROS_MASTER_URI=http://192.168.1.110:11311
- ROS HOSTNAME AS IP OF laptop
    -export ROS_HOSTNAME=192.168.1.110

- set node namespace in your publisher and subscriber   

### On Turtlebot bashrc
- ROS master ip of laptop:
    -export ROS_MASTER_URI=http://192.168.1.110:11311
- ROS hostname as turtlebot ip:
    -export ROS_HOSTNAME= turtlebot_ip 

#

