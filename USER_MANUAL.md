Last update: 2019-09-10

#### Table of Contents
[Command Flags](#command-flags)  
[Mouse Control](#mouse-control)  
[Keyboard Control](#keyboard-control)  
[Interactive Command Line Control](#interactive-command-line-control)

Note that unless the flag name has only one character, otherwise there are two leading dash symbols.

## Command Flags
Run the command with following flag options.
'''batch
python simulator.py
'''

| Flags         | Descriptions                                |
| ------------- |---------------------------------------------| 
| --help        | Show all the flags’ usages.                 |

Commands for general control  
  
| Flags               | Descriptions                                    |
| ------------------- |-------------------------------------------------| 
| -c                  | Start a closed environment free of any obstacle, in which no port and no task is assigned to agents. User needs to manually specify a path to each agent.                            |
| -d                  | Run the simulator with agents' details showed.  |
| -t < float >         | Running for some minutes without showing up the graphical simulator. PPH (package per hour) is printed every 1-minute in the simulated world.                                              |
| --gp < string >      | Set the global planners of all agents, choices include: LayeredAStar, RRTStar, or your customized algorithm if applicable.                                                       |
| --lp < string >      | Set the local planners of all agents, choices include: DullPlanner, VirtualForcePlanner, FLCPlanner, or your customized algorithm if applicable.                                          |
| --title < string >   | Give the simulator a title to display at the header bar. |

Commands for changing config.JSON data  
  
| Flags         | Descriptions                                |
| ------------- |---------------------------------------------| 
|--agent < int >            | Set the number of agents.  |
|--port < loading int > < unloading int >   | Set the number of loading and unloading ports.  |
|--size < width int > < height int >  | Set the width and height (in meter) of the environment.  |
|--resolution < int > | Set the resolution of the grid (number of grids per meter).  |
|--speed < float >  |Set the cruise speed of the agents.  |
|--step < int >  | Set the number of steps to be simulated per second.  |
|--default | Set all settings back to default.  |


Commands for obstacle generation  
  
| Flags         | Descriptions                                |
| ------------- |---------------------------------------------| 
|--obstacle < a string of (top_left_x,top_left_y,width) tuples separated by comma >           | User specified list of square obstacles. For instance, ```--obstacle '(5,5,1),(6,7,2)'``` gives two obstacles: the first is a 1-meter-width square which has its top-left corner at (5,5); and the second is a 2-meter-width square which has its top-left corner at (6,7). Note that if an obstacle collides with any existing port (or obstacle), it will be discarded. |
|--obstacle < float between 0.0 and 0.7 >  | The simulator randomly generates obstacles upto some percentage of congestion level.  |
|--seed < int>  | The seed used in random generation of obstacles; a fixed seed gives a fixed obstacle map.  |
|--save < filename >  | Save the obstacle map generated to a file under MAP folder for future reuse. |
|--import < filename > | Import an obstacle map from MAP folder. |

##### Examples
```batch
python simulator.py -c --default --obstacle 0.5 --save map1 --gp RRTStar --step 10 --agent 2 -d
```
Start a free control scenario; randomly create 50% obstacles and save the map as file “map1”; use RRTStar as global planners; each agent examines the world 10 times per second.

```batch
python simulator.py --import map1 --gp LayeredAStar --lp VirtualForcePlanner --agent 4 --resolution 2 -d
```
Start a delivery scenario; use map “map1”; use global planner LayeredAStar and local planners VirtualForcePlanner; the number of agents is 4; the resolution of grid map is 2 (2-grid-cell per meter).

```batch
python simulator.py --default --size 12 15 --port 3 2 --title dorabot1
```
Start a delivery scenario; the size of the environment is 12 by 15 meter; the numbers of loading and unloading ports are 3 and 2 respectively; the simulator has a title named “dorabot1”.

## Mouse Control
| Actions         | Mouse Control                                |
| ------------- |---------------------------------------------| 
| Move an agent         | Left-click on an agent and drag to a new position. Release mouse button to drop the agent.                              |
| Select (an) agent(s)        | Left-click on a non-agent spot and drag the mouse. All agents in the rectangle region will be selected. Note that no immediate change is shown to tell if an agent is selected or not.         |
| Give path(s) to (an) agent(s)*, **       |After some agent(s) get selected , right-click on the map to give a sequence of waypoints.           |
| Get an agent’s / a port’s information        | Right-click on an agent / a port. The information is printed in terminal.     |  

\* If the simulator is running, the agent will move immediately right after it receives its first waypoint. Use keyboard P to pause the simulator if you want to assign paths to multiple agents and execute the paths at the same time.

** If the global planner is grid-based (A* family), the waypoints are wrapped to nearest grid points. If is continuous-space-based (RRT family), the waypoints are left as input.

## Keyboard Control
| Key         | Event                               |
| ------------- |---------------------------------------------| 
| q/Q         | Exit the program.                              |
| p/P        | Pause / resume running of the simulator                            |
| g/G        | Show / hide the grid map              |
| r/R        |Show / hide the control range of ports                           |
| d/D        | Show / hide the agents’ details                           |
| s/S        | Show / hide the sensor range                           |
## Interactive Command Line Control
We also provide interactive command line control to ease the testing and evaluation of new algorithms. To start the control panel, run:
```batch
python control.py
```
You will see the following input prompt:
```batch
(cmd)
```
Use ```help``` to get usage help.

#### Available commands
* ```new```  
Same as [command flags](#command-flags), but allow ```Tab``` auto-completion for global planner and local planner input. You can run multiple simulators at the same time with different names, and give commands to a specific simulator using ```--title``` flag
   ```batch
   (cmd) help new
   (cmd) new --title dorabot1 --agent 4 --port 2 2 --size 20 20 --gp RRTStar --lp VirtualForcePlanner -d
   ```
   A simulator titled "dorabot1" has 4 agents equipped with RRTStar and VirtualForcePlanner running in a 20x20-grid, 2x2-port environment.
   Note the output message:
   ```batch
   Current active simulator: dorabot1
   ```
   
* ```control```  
Change planners and paths of part of, or all of the agents in a simulator. ```Tab``` auto-completion is provided for ```--title``` ```--gp``` ```--lp``` flags.If no ```--title``` is given 
   ```batch
   (cmd) help control
   (Cmd) control --title dorabot1 --gp LayeredAStar --lp DullPlanner --agent 0 -p (5,5),(7,7),(10,10)
   ```
   Control the agent 0 from the simulator titled "dorabot1"; change its planner to LayeredAStar and DullPlanner, and its path to start from (5,5), passing (7,7), to (10,10)
   * If no ```--title``` specified, current active simulator will receive the command.
   * If no ```--agent``` specified, all agents from the simulator will receive the command.
   * If no ```""``` around the path tuples, no space is allowed; e.g. (5,5),(10,10) [Valid], (5,5), (10, 10) [Invalid], "(5,5), (10, 10)" [Valid].


* ```activate```  
Give a title of a simulator to set it as current active simulator. ```Tab``` auto-completion is available.
   ```batch
   (cmd) activate dorabot1
   ```

* ```log```  
The log of each simulator is suppressed by default. Give a simulator title to show its log. ```Tab``` auto-completion is available.
   ```batch
   (cmd) log dorabot1
   ```
   

* ```s``` / ```silent```  
Stop all simulators' log.
   ```batch
   (cmd) s
   ```
   
* ```close```  
Close a simulator with given title. ```Tab``` auto-completion is available.
   ```batch
   (cmd) close dorabot1
   ```
   
* ```template```  
Create a new file from a global planner or local planner's template. It provides the skeleton of the planner data structure.
   ```batch
   (cmd) help template
   (cmd) template --magp MyMultiagentGlobal
   (cmd) template --gp MyGlobal
   (cmd) template --lp MyLocal
   ```
   * The input argument should be the classname without "Planner" as suffix, and in CamelCase. ```template --gp MyGlobal``` creates a file named ```my_global_planner.py``` with a class named ```MyGlobalPlanner``` under the folder ```global_planners/user```.
   * The control panel will try to open the file in your current editor; if no applicable, please go to the following folders to manually open and fill the files:
     * --mapg, stands for "Multiagent Global Planner", customized files are stored in ```multiagent_global_planners/user```.
     * --gp, "Global Planner", ```global_planners/user```.
     * --lp, "Local Planner", ```local_planners/user```.
   
* ```rm```  
Remove the customized algorithm files. This is provided for easy clean-up. ```Tab``` auto-completion is available.
   ```batch
   (cmd) rm MyMultiagentGlobal
   ```
 
  
* ```get```  
Retrieve some data structure log, and store into local variable. ```Tab``` auto-completion is available for ```--title```.
   ```batch
   (cmd) help get
   (cmd) get -a 0 agent_zero -s 1 agent_one_sensor
   ```
   * The above example gets the data log of agent 0 into local variable ```agent_zero```; and get agent 1's sensor data log into ```agent_one_sensor```. Since no ```--title``` is specified in this example, the current active simulator will proceed the command by default.
  
* ```print```  
After getting back logs into local variables, details can be printed. ```Tab``` auto-completion is available.
   ```batch
   (cmd) print agent_zero
   (cmd) print agent_zero.sensor
   (cmd) print agent_zero.sensor.location.x
   ```
   
* ```q``` / ```quit```  
Close all simulators as well as the control panel.
   ```batch
   (cmd) q
   ```

   
* ```py```  
Start python shell.