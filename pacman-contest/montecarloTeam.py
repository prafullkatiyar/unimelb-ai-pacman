# myTeam.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


from captureAgents import CaptureAgent
import random, time, util
from game import Directions
import game
from util import nearestPoint

#################
# Team creation #
#################


def createTeam(firstIndex, secondIndex, isRed,
               first='AttackerAgent', second='DefenderAgent'):
    """
    This function should return a list of two agents that will form the
    team, initialized using firstIndex and secondIndex as their agent
    index numbers.  isRed is True if the red team is being created, and
    will be False if the blue team is being created.
    As a potentially helpful development aid, this function can take
    additional string-valued keyword arguments ("first" and "second" are
    such arguments in the case of this function), which will come from
    the --redOpts and --blueOpts command-line arguments to capture.py.
    For the nightly contest, however, your team will be created without
    any extra arguments, so you should make sure that the default
    behavior is what you want for the nightly contest.
    """
    return [eval(first)(firstIndex), eval(second)(secondIndex)]


##########
# Agents #
##########

class Actions():
    """
    A base class for all the actions that can be used by both attacker and defender.
    """

    def getSuccessor(self, gameState, action):
        """
        Finds the next successor which is a grid position (location tuple).
        """
        successor = gameState.generateSuccessor(self.index, action)
        pos = successor.getAgentState(self.index).getPosition()
        if pos != nearestPoint(pos):
            # Only half a grid position was covered
            return successor.generateSuccessor(self.index, action)
        else:
            return successor

    def evaluate(self, gameState, action):
        """
        Computes a linear combination of features and feature weights
        """
        features = self.evaluateAttackParameters(gameState, action)
        weights = self.getCostsOfAttackParamters(gameState, action)
        
        return features * weights

    def evaluateAttackParameters(self, gameState, action):
         
        """
        Returns a counter of features for the state
        """
        features = util.Counter()
        successor = self.getSuccessor(gameState, action)
        print successor
        features['successorScore'] = self.agent.getScore(successor)
        print features['sucessorScore']
        return features

    def getCostsOfAttackParamters(self, gameState, action):
        """
        Normally, weights do not depend on the gamestate.  They can be either
        a counter or a dictionary.
        """
        return {'successorScore': 1.0}

class getDefensiveActions():
    def __init__(self, agent, index, gameState):
        self.index = index
        self.agent = agent
        self.defendCoordinates = {}

        if self.agent.red:
            mazeHorizontalHalf = (gameState.data.layout.width-2) / 2
        else:
            mazeHorizontalHalf = ((gameState.data.layout.width-2) / 2) + 1
        self.mazeVerticalBoundary = []
        for coordinateY in range(1, gameState.data.layout.height - 1):
            if not gameState.hasWall(mazeHorizontalHalf, coordinateY):
                self.mazeVerticalBoundary.append((mazeHorizontalHalf, coordinateY))
                
        #print self.mazeVerticalBoundary
        self.enemyCoordinates = None
        self.foodLeft = None
        self.possibleDefensiblePosition(gameState)

    def possibleDefensiblePosition(self, gameState):
        """ 
        Get the distances to the closest food from all the possible coordinates on the mazeVerticalBoundary  
        """
        for position in self.mazeVerticalBoundary:
            foodCoordinates = self.agent.getFoodYouAreDefending(gameState).asList()
            distanceToClosetFood=min(self.agent.getMazeDistance(position,f) for f in foodCoordinates)
            if distanceToClosetFood == 0:
                distanceToClosetFood = 1
            self.defendCoordinates[position] = distanceToClosetFood
        #print ("Possible Defensible points",self.defendCoordinates)    
    
    def selectBestDefenisblePosition(self):
        """ 
        Get the best defensible position by selecting the coordinates from the possible defenisble positions having the minimum distance to food.  
        """
        minDistance=min(self.defendCoordinates[x] for x in self.defendCoordinates.keys())
        bestDefensivePosition = filter(lambda x: self.defendCoordinates[x] == minDistance, self.defendCoordinates.keys())
        #print ("Best Defensive Position",bestDefensivePosition)
        return random.choice(bestDefensivePosition)

    def chooseAction(self, gameState):
        # find all the food that are left
        foodLeft=self.agent.getFoodYouAreDefending(gameState).asList();
        
        #find all the visible enemies
        enemyStates=[gameState.getAgentState(state) for state in self.agent.getOpponents(gameState)]
        visibleEnemy = filter(lambda enemy: enemy.isPacman and enemy.getPosition() != None,enemyStates)
        
        pacmanCurrentCoordinates = gameState.getAgentPosition(self.index)
        if pacmanCurrentCoordinates == self.enemyCoordinates:
            self.enemyCoordinates = None
        
        actions = gameState.getLegalActions(self.index)
        
        # if food left is less than 5 then attack and try eat all food and power capsule without worrying about defense 
        if len(foodLeft) < 5:
            food = foodLeft + self.agent.getCapsulesYouAreDefending(gameState)
            self.enemyCoordinates = random.choice(food)
        #else if for all enemies that are visible find the distance between our pacman and the enemies and follow the nearest one and kill it
        elif len(visibleEnemy)>0:        
            distanceToVisibleEnemy,stateOfVisibleEnemy = min([(self.agent.getMazeDistance(pacmanCurrentCoordinates,enemy.getPosition()), enemy) for enemy in visibleEnemy])
            self.enemyCoordinates=stateOfVisibleEnemy.getPosition()
        # else defend the best defensible position    
        else:
            self.enemyCoordinates = self.selectBestDefenisblePosition()
             
        
        
        
        possibleActions = []
        distanceFromNewPosition = []
        
        """
        Iterate through all possible actions. If actions is not stop and action doesn't lead to a state 
        where our agent become pacman then its a valid possible actions
        """
        for tempAction in actions:
            nextState = gameState.generateSuccessor(self.index, tempAction)
            if not tempAction == Directions.STOP and not nextState.getAgentState(self.index).isPacman:
                newPosition = nextState.getAgentPosition(self.index)
                possibleActions.append(tempAction)
                distanceFromNewPosition.append(self.agent.getMazeDistance(newPosition, self.enemyCoordinates))
       
        """ from all the possible actions select the one having the shortest path to reach 
        the defenisve position as soon as possible
        """
        minDistance = min(distanceFromNewPosition)
        minDistanceList = filter(lambda x: x[0] == minDistance, zip(distanceFromNewPosition, possibleActions))
        
        return random.choice(minDistanceList)[1]
        

        


class getAttackingActions(Actions):
    def __init__(self, agent, index, gameState):
        self.agent = agent
        self.index = index
        self.agent.distancer.getMazeDistances()
        self.counter = 0

        if self.agent.red:
            mazeHorizontalHalf = (gameState.data.layout.width - 2) / 2
        else:
            mazeHorizontalHalf = ((gameState.data.layout.width - 2) / 2) + 1
        self.mazeVerticalBoundary = []
        for coordinateY in range(1, gameState.data.layout.height - 1):
            if not gameState.hasWall(mazeHorizontalHalf, coordinateY):
                self.mazeVerticalBoundary.append((mazeHorizontalHalf, coordinateY))


    def evaluateAttackParameters(self, gameState, action):
        """
        Get features used for state evaluation.
        """
        attackParameters = util.Counter()
        successor = self.getSuccessor(gameState, action)
        attackParameters['gameScore'] = self.agent.getScore(successor)
        
        # get current position of the agent
        currentCoordinates = successor.getAgentState(self.index).getPosition()
        #calculate the nearest boundary
        attackParameters['nearestBoundary']=min(self.agent.getMazeDistance(currentCoordinates, self.mazeVerticalBoundary[i]) for i in range(len(self.mazeVerticalBoundary)))
        #print ("nearestBoundary",attackParameters['nearestBoundary'])
        attackParameters['foodInHand'] = successor.getAgentState(self.index).numCarrying
        
        
        #calculate the distance to nearest food.
        foodLeft = self.agent.getFood(successor).asList()
        if len(foodLeft) > 0:   
            attackParameters['distanceToNearestFood']=min(self.agent.getMazeDistance(currentCoordinates, food) for food in foodLeft)
       
        #calculate the distance to nearest power capsule.
        powerCapsuleLeft = self.agent.getCapsules(successor)
        if len(powerCapsuleLeft) > 0:
            attackParameters['distanceToNearestPowerCapsule']=min(self.agent.getMazeDistance(currentCoordinates, capsule) for capsule in powerCapsuleLeft)

       #minimum distance to closest ghosts
        enemyStates = []
        for state in self.agent.getOpponents(successor):
           enemyStates.append(successor.getAgentState(state))   
        visibleEnemy = filter(lambda enemy: enemy.isPacman and enemy.getPosition() != None,enemyStates)    
        if len(visibleEnemy)>0:
            enemyPosition = [enemy.getPosition() for enemy in visibleEnemy]
            closestEnemy = min(enemyPosition, key=lambda x: self.agent.getMazeDistance(currentCoordinates, x))
            distanceToClosestGhost = self.agent.getMazeDistance(currentCoordinates, closestEnemy)
            if distanceToClosestGhost <6:
                attackParameters["distanceToClosestGhost"] = distanceToClosestGhost        
        else:
            normalDistance = []
            for state in self.agent.getOpponents(successor):
                normalDistance.append(successor.getAgentDistances()[state])
            attackParameters['distanceToClosestGhost'] = min(normalDistance)
        
        """
            Kill the enemy if its a ghost and is in range that is distanceToClosestEnemy < 5
        """
       
        enemyPacman = [successor.getAgentState(state) for state in self.agent.getOpponents(successor)]
        visibleRange = filter(lambda enemy: enemy.isPacman and enemy.getPosition() != None, enemyPacman)
        if len(visibleRange)>0:
            enemyPosition =[enemy.getPosition() for enemy in visibleRange]
            closestEnemy = min(enemyPosition, key=lambda y: self.agent.getMazeDistance(currentCoordinates, y))
            distanceToClosestEnemy= self.agent.getMazeDistance(currentCoordinates, closestEnemy)
            if distanceToClosestEnemy <5:
                attackParameters['distanceToEnemyPacman'] = distanceToClosestEnemy
        else:
            attackParameters['distanceToEnemyPacman'] = 0
            
        return attackParameters






    def getCostsOfAttackParamters(self, gameState, action):
        
        """
        Costs of all the attacking parameters.
        """    
        successor = self.getSuccessor(gameState, action)
        foodCarrying = successor.getAgentState(self.index).numCarrying
        enemy = [successor.getAgentState(i) for i in self.agent.getOpponents(successor)]
        visibleEnemy = filter(lambda x: not x.isPacman and x.getPosition() != None, enemy)
        if len(visibleEnemy) > 0:
            for enemy in visibleEnemy:
                """ If power capsule has been eaten and then 
                        1. don't worry about the distanceToClosestGhost and set it to negative value.
                        2. set the value of nearest boundary based on the value to the nearest food. Try to increase the number of food carrying. 
                        3. set distanceToPowerCapsule as negative or zero.
                        4. When the scaredTime is less then try and eat maximum food before returning.
                """
                if enemy.scaredTimer > 0:
                    if enemy.scaredTimer > 12:
                        return {'gameScore': 110, 'distanceToNearestFood': -10, 'distanceToEnemyPacman': 0,
                                'distanceToClosestGhost': -1, 'distanceToNearestPowerCapsule': 0, 'nearestBoundary': 10-3*foodCarrying, 'foodInHand': 350}
                        
                    elif 6 < enemy.scaredTimer < 12 :
                        return {'gameScore': 110+5*foodCarrying, 'distanceToNearestFood': -5, 'distanceToEnemyPacman': 0,
                                'distanceToClosestGhost': -1, 'distanceToNearestPowerCapsule': -10, 'nearestBoundary': -5-4*foodCarrying,
                                'foodInHand': 100}

                # Visible and not scared
                else:
                    return {'gameScore': 110, 'distanceToNearestFood': -10, 'distanceToEnemyPacman': 0,
                            'distanceToClosestGhost': 20, 'distanceToNearestPowerCapsule': -15, 'nearestBoundary': -15,
                            'foodInHand': 0}

        self.counter += 1
      
        return {'gameScore': 1000+foodCarrying*3.5, 'distanceToNearestFood': -7, 'distanceToClosestGhost': 0, 'distanceToEnemyPacman': 0,
                'distanceToNearestPowerCapsule': -5, 'nearestBoundary': 5-foodCarrying*3, 'foodInHand': 350}
        
        


    def chooseAction(self, gameState):
       

        # Get valid actions. Randomly choose a valid one out of the best (if best is more than one)
        actions = gameState.getLegalActions(self.agent.index)
        actions.remove(Directions.STOP)
        possibleActions = []
        for a in actions:
            value = 0
            value = self.monteCarloSimulation(2, gameState.generateSuccessor(self.agent.index, a))
            possibleActions.append(value)

        bestPossibleAction = max(possibleActions)
        allPossibleActions = filter(lambda x: x[0] == bestPossibleAction, zip(possibleActions, actions))
        return random.choice(allPossibleActions)[1]
    
    def monteCarloSimulation(self, treeDepth, gameState):
        nextPossibleState = gameState.deepCopy()
        #Simulate to choose uniform random moves 
        while treeDepth > 0:
          # get all possible actions that are valid
          possibleActions = nextPossibleState.getLegalActions(self.index)
          # Remove Direction.STOP from possible actions so that our pacman agent doesn't stop during simulation
          possibleActions.remove(Directions.STOP)
          # Remove Direction.REVERSE from possible actions so that our pacman agent doesn't use reverse direction during simulation 
          reversed_direction = Directions.REVERSE[nextPossibleState.getAgentState(self.index).configuration.direction]
          if reversed_direction in possibleActions and len(possibleActions) > 1:
            possibleActions.remove(reversed_direction)
          # Randomly chooses a valid action
          action = random.choice(possibleActions)
          
          
          nextPossibleState = nextPossibleState.generateSuccessor(self.index, action)
          treeDepth -= 1
        # return the best possible action from the result of simulation.
        return self.evaluate(nextPossibleState, Directions.STOP)


class DefenderAgent(CaptureAgent):
   
    def registerInitialState(self, gameState):
        CaptureAgent.registerInitialState(self, gameState)
        self.OffenceStatus = getAttackingActions(self, self.index, gameState)
        self.DefenceStatus = getDefensiveActions(self, self.index, gameState)

    def chooseAction(self, gameState):
        return self.DefenceStatus.chooseAction(gameState)
    
    



class AttackerAgent(CaptureAgent):
    def __init__(self, index):
        CaptureAgent.__init__(self, index)

    def registerInitialState(self, gameState):
        CaptureAgent.registerInitialState(self, gameState)
        self.DefenceStatus = getDefensiveActions(self, self.index, gameState)
        self.OffenceStatus = getAttackingActions(self, self.index, gameState)

    def chooseAction(self, gameState):
        self.enemies = self.getOpponents(gameState)
        enemyInavders = [invader for invader in self.enemies if gameState.getAgentState(invader).isPacman]


        if  self.getScore(gameState) >= 13:
            return self.DefenceStatus.chooseAction(gameState)
        else:
            return self.OffenceStatus.chooseAction(gameState)