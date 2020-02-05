# UoM COMP90054 AI Planning for Autonomy - Pacman capture the flag by PacmanGhosts

**Monte carlo and Heuristic based Pacman AI**

# Demo
The final presentation and te demo is available at the below link:
youtube : https://youtu.be/SkTKRRxKAh8

#Requirements
Python 2.7 or higher

# Usage
The file contains implementation of the pacman agent in 3 different algorith and the main comptetion file.
1. The **myTeam.py** is the implementation of the combination of best af all the implementations. 
It uses the monte carlo tree search and the methods of heuristic algorithm for the offensove agent.
2. The **gametheory.py** is the minimax and hidden marcov based implementation of the pacman AI. 
3. The **montecarloTeam.py** is the monte carlo based implementation of the pacman AI.
4. The **heuristicTeam.py** uses the linear regression heuristic concepts for the offensove agent to predict the future actions.

# How to Execute
The pacman agent can  be executred as a blue team or a red team :

python capture.py -r myTeam.py -b baselineTeam.py -- this command be used to execute the agent as a red team and if to be executed as a blue team enter the 
file name after the -b in the command. To execute the other implementations change the files names accordingly.

# Layouts

The pacman game had 2 types of layouts :
1. Fixed layouts -- this can be implemented by using the above command 
2. Random layouts -- This can be implemented by using : python capture.py -r myTeam.py -b baselineTeam.py -l RANDOM2 where the number 2 is the map number.

# Acknoledgements

Pac-man implementation by UC Berkeley:

The Pac-man Projects - UC Berkeley (http://ai.berkeley.edu/project_overview.html)

**The complete detailed documentation of the algorithms used and the analysis of the algorithms can be found in the below link.**
https://gitlab.eng.unimelb.edu.au/vduggi/comp90054-pacman/wikis/Comp90054-Pacman-Project-929924