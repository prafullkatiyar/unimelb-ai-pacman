# myTeam.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


from captureAgents import CaptureAgent
import random, time, util, sys
from game import Directions
import game
import distanceCalculator
from util import nearestPoint

#################
# Team creation #
#################


def createTeam(firstIndex, secondIndex, isRed, first='OffenderAgent',
                 second='DefenderAgent'):
    """
    This function returns a list of two agents that will form the
    team, initialized using firstIndex and secondIndex as their agent
    index numbers.  isRed is True if the red team is being created, and
    will be False if the blue team is being created.
    """

    # The following line is an example only; feel free to change it.
    return [eval(first)(firstIndex), eval(second)(secondIndex)]


##########
# Agents #
##########

class HelpingAgent(CaptureAgent):
    def registerInitialState(self, gameState):
        CaptureAgent.registerInitialState(self, gameState)

        # FInd the initial position, the mid point of the board, the position
        #where the agents can move, the maze distance and the position of all teh 4 agents.
        self.start = gameState.getInitialAgentPosition(self.index)
        self.widthCenter = gameState.data.layout.width/2
        #self.legalPos = [p for p in gameState.getWalls().asList(False) if p[1] > 1]
        self.legalPos = []
        for pos in gameState.getWalls().asList(False):
            if pos[1] > 1:
                self.legalPos.append(pos)
        self.distancer.getMazeDistances()
        self.team = self.getTeam(gameState)
        self.enemies = self.getOpponents(gameState)

        # Initialize the likely position to be 1 at the initial position for each of the
        # opposition agents. create a dictionary that holsd the likely position of each agent.
        self.Likelypos = {}
        for e in self.enemies:
            self.Likelypos[e] = util.Counter()
            self.Likelypos[e][gameState.getInitialAgentPosition(e)] = 1.


    def initializeLikelypos(self, enemybot):
        """
        This is to get the likely available position of the agent.
        """

        self.Likelypos[enemybot] = util.Counter()

        for p in self.legalPos:
            self.Likelypos[enemybot][p] = 1.0

        self.Likelypos[enemybot].normalize()


    def elapseTime(self, enemybot, gameState):
        """
        This is to know where the pacman can move.

        """
        new_likelypos = util.Counter()

        for previousPosition in self.legalPos:
            # Get the new probability distribution.
            newProbDistribution = util.Counter()

            # The next possible cannot be determined by moving one step forward in horizontal 
            #or vertical direction as some of these moves may be illegal. 

            for i in [-1,0,1]:
                for j in [-1,0,1]:
                    if not (abs(i) == 1 and abs(j) == 1):
                        possiblePositions = (previousPosition[0]+i,previousPosition[1]+j)

            # If the position is legal then add the position to the new probability distribution list of positions.
            for p in possiblePositions:
                if p in self.legalPos:
                    newProbDistribution[p] = 1.0
                else:
                    pass

            # normalisation is done to make the vaues uniform.
            newProbDistribution.normalize()

            # Once we have the list of the legal positions and the probability distribution 
            # now find the probability distribution for the new likely positions
            # and also update the probability distribution of every position. 
            for newPos, prob in newProbDistribution.items():
                new_likelypos[newPos] += prob * self.Likelypos[enemybot][previousPosition]

        # update the new likely position list.
        new_likelypos.normalize()
        self.Likelypos[enemybot] = new_likelypos


    def observe(self, enemy, observation, gameState):
        """
        This is a clsss for finding the position of the agent and the enemy agent.
        Planning to use the Hidden Markov model cpncepts to get the correct distance from the 
        predicted noisy distacne. 
        """

        # Get the expected distance for the current enemy.
        # Get the position of the calling agent.
         # Creating a dictionary of new likely positions for the enemy.

        expectedDistance = observation[enemy]
        currentPosition = gameState.getAgentPosition(self.index)
        new_likelypos = util.Counter()

        # For each of the legal positions get the new new likely position.
        # Calculating correct  distance to the position.
        # the probability of the correct distance is calculated for current position.
        # Few moves cannot be the possible moves ofr the agent so thi spart of code
        # is to check whether the move is possible by the agent or not.
        for p in self.legalPos:
            correctDistance = ((currentPosition[0] - p[0]) ** 2 + (currentPosition[1] - p[1]) ** 2 ) ** 0.5
            correctDist_prob = gameState.getDistanceProb(correctDistance, expectedDistance)
            if self.red:
                pac = p[0] < self.widthCenter
            else:
                pac = p[0] > self.widthCenter

            # If the true distance is less than 5 then it is the actual distance 
            # and the new_likelypos will be 0 and this distacen can be determined
            # accurately and only when the distance is above 5 it is not exact.
            if correctDistance <= 5:
                new_likelypos[p] = 0.
            elif pac != gameState.getAgentState(enemy).isPacman:
                new_likelypos[p] = 0.
            else:
                # P(x_t|e_1:t) = P(x_t|e_1:t) * P(e_t:x_t) is the equation for online new likely position update.
                new_likelypos[p] = self.Likelypos[enemy][p] * correctDist_prob

        if new_likelypos.totalCount() == 0:
            self.initializeLikelypos(enemy)
        else:
            # Normalize and set the new belief.
            new_likelypos.normalize()
            self.Likelypos[enemy] = new_likelypos


    def chooseAction(self, gameState):
        """
        In this function we begin by updating our Likelypos
        and elapsing time for the Likelypos. We also show our Likelypos on the
        screen by using the provided debugging function.
        """

        currentPosition = gameState.getAgentPosition(self.index)
        possibleDist = gameState.getAgentDistances()
        newState = gameState.deepCopy()

        for enemy in self.enemies:
            enemypresentPos = gameState.getAgentPosition(enemy)
            if enemypresentPos:
                new_likelypos = util.Counter()
                new_likelypos[enemypresentPos] = 1.0
                self.Likelypos[enemy] = new_likelypos
            else:
                self.elapseTime(enemy, gameState)
                self.observe(enemy, possibleDist, gameState)

        # most probable position is used to update the game state.
        # Expectimax can be used only once we know the set of positions where
        # the enemy is starting.
        for enemy in self.enemies:
            probablePosition = self.Likelypos[enemy].argMax()
            config = game.Configuration(probablePosition, Directions.STOP)
            newState.data.agentStates[enemy] = game.AgentState(config, newState.isRed(probablePosition) != newState.isOnRedTeam(enemy))

        action = self.minimaxAlgoExt(newState, height=2)[1]

        return action


    def minimaxAlgoExt(self, gameState, height):
        """
         This function is mainly used to accuratrly predict the expected
         moves of the enemy agent.
        """

        # end of game check .
        if height == 0 or gameState.isOver():
            return self.evaluationFunction(gameState), Directions.STOP

        # find the next game states for future moves.
        actions = gameState.getLegalActions(self.index)
        actions.remove(Directions.STOP)
        futureGameStates = [gameState.generateSuccessor(self.index, action)
                                 for action in actions]

        # Getting the expected scores of enemy moves.
        scores = [self.minimaxAlgo(successorGameState, self.enemies[0], height)[0]
                    for successorGameState in futureGameStates]

        bestIndices = []
        for i in range(len(scores)):
            if scores[i] == max(scores):
                bestIndices.append(i)

        chosenIndex = random.choice(bestIndices)

        return max(scores), actions[chosenIndex]


    def minimaxAlgo(self, gameState, enemy, height):
        """
       The implementation of the expectimax function for the Berkley problem 2.
        """
        if height == 0 or gameState.isOver():
            return self.evaluationFunction(gameState), Directions.STOP
        actions = gameState.getLegalActions(enemy)
        futureGameStates = []
        for action in actions:
            try:
                futureGameStates.append(gameState.generateSuccessor(enemy, action))
            except:
                pass

        # If there is another ghost, then call the expecti function for the
        # next ghost, otherwise call the max function for pacman.
        if enemy < max(self.enemies):
            scores = [self.minimaxAlgo(successorGameState, enemy + 2, height)[0]
                        for successorGameState in futureGameStates]
        else:
            scores = [self.minimaxAlgoExt(successorGameState, height - 1)[0]
                        for successorGameState in futureGameStates]

        # Calculate the expected value.
        bestScore = sum(scores) / len(scores)

        return bestScore, Directions.STOP


    def enemyDistances(self, gameState):
        """
        If we know  the exact location of enemy agent position then we calculate the distance 
        from our agent to the enemy agent. If agent is not in sight then we calculate the 
        distance by assuming where the probability of agent presence is highest and 
        calculate the distance.

        """
        # Get the appromixamte predicted position if the exact position is not known.
        dists = []
        for i in self.enemies:
            currentPosition = gameState.getAgentPosition(self.index)
            enemypresentPos = gameState.getAgentPosition(i)
            if enemypresentPos: 
                pass
            else:  
                enemypresentPos = self.Likelypos[i].argMax()
            dists.append((i, self.distancer.getDistance(currentPosition, enemypresentPos)))
        return dists


    def evaluationFunction(self, gameState):
        """
        Evaluate the utility of a game state.
        """
        util.raiseNotDefined()


class OffenderAgent(HelpingAgent):
    """
    This directly goes to ea the food on the enemy side and when the enemy is 
    near to the agent it retreats. After collecting some food them the agent comes into the 
    home to drop the food.
    """

    def registerInitialState(self, gameState):
        HelpingAgent.registerInitialState(self, gameState)
        self.retreating = True


    def chooseAction(self, gameState):
        timetoMove = [gameState.getAgentState(enemy).scaredTimer for enemy in self.enemies]
        score = self.getScore(gameState)
        
        if len(self.getFood(gameState).asList()) > 2:
            self.retreating = False

        else:
            if min(timetoMove) > 5: 
                self.retreating = False
            else:
                self.retreating = True
        
        return HelpingAgent.chooseAction(self, gameState)


    def evaluationFunction(self, gameState):
        # Get the current position.
        currentPosition = gameState.getAgentPosition(self.index)

        # Get the food on the board.
        nextFood = self.getFood(gameState).asList()

        # Get the closest distance to the middle of the board.
        closestdist_middle = min([self.distancer.getDistance(currentPosition, (self.widthCenter, i))
                                 for i in range(gameState.data.layout.height)
                                 if (self.widthCenter, i) in self.legalPos])

        # Finding the enemy ghosts.
        enemyDist = []
        for e in self.enemies:
            if not gameState.getAgentState(e).isPacman:
                enemypresentPos = gameState.getAgentPosition(e)
                if enemypresentPos != None:
                    enemyDist.append(self.distancer.getDistance(currentPosition, enemypresentPos))

        # find how far the gost is to kill our agent.
        nearestEnemy = min(enemyDist) if len(enemyDist) else 0
        if nearestEnemy >= 4:
            nearestEnemy = 0

        # power pill condition.
        foodColour = None
        if self.red:
            foodColour = gameState.getBlueCapsules()
        else:
            foodColour = gameState.getRedCapsules()

        # distance and minimum distance to the capsule.
        foodColourDistances = [self.distancer.getDistance(currentPosition, capsule) for capsule in
                                    foodColour]
        minCapsuleChasingDistance = min(foodColourDistances) if len(foodColourDistances) else 0

        # Time to go back to safety, or trying to find food still.
        if self.retreating:
            # Want to get back to the other side at this point. Weight is on
            # staying safe and getting back to the halfway point.
            return - 2 * closestdist_middle + 500 * nearestEnemy
        else:
            # Look for the nearest food.
            foodDistances = [self.distancer.getDistance(currentPosition, food) for
                             food in nextFood]
            nearestFood = min(foodDistances) if len(foodDistances) else 0
            timetoMove = [gameState.getAgentState(enemy).scaredTimer for enemy
                             in self.enemies]

            if min(timetoMove) <= 6 and nearestEnemy < 4:
                nearestEnemy *= -1

            return 2 * self.getScore(gameState) - 100 * len(nextFood) - \
                   3 * nearestFood - 10000 * len(foodColour) - \
                   5 * minCapsuleChasingDistance + 100 * nearestEnemy

class getDefensiveActions():
    def __init__(self, agent, index, gameState):
        self.index = index
        self.agent = agent
        self.defendCoordinates = {}

        if self.agent.red:
            mazeHorizontalHalf = (gameState.data.layout.width) / 2
        else:
            mazeHorizontalHalf = ((gameState.data.layout.width) / 2) + 1
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
        

        

class DefenderAgent(CaptureAgent):
   
    def registerInitialState(self, gameState):
        CaptureAgent.registerInitialState(self, gameState)
        self.DefenceStatus = getDefensiveActions(self, self.index, gameState)

    def chooseAction(self, gameState):
        return self.DefenceStatus.chooseAction(gameState)
    
    



class DummyAgent(CaptureAgent):
  """
  A Dummy agent to serve as an example of the necessary agent structure.
  You should look at baselineTeam.py for more details about how to
  create an agent as this is the bare minimum.
  """

  def registerInitialState(self, gameState):
    """
    This method handles the initial setup of the
    agent to populate useful fields (such as what team
    we're on).

    A distanceCalculator instance caches the maze distances
    between each pair of positions, so your agents can use:
    self.distancer.getDistance(p1, p2)

    IMPORTANT: This method may run for at most 15 seconds.
    """

    '''
    Make sure you do not delete the following line. If you would like to
    use Manhattan distances instead of maze distances in order to save
    on initialization time, please take a look at
    CaptureAgent.registerInitialState in captureAgents.py.
    '''
    CaptureAgent.registerInitialState(self, gameState)

    '''
    Your initialization code goes here, if you need any.
    '''


  def chooseAction(self, gameState):
    """
    Picks among actions randomly.
    """
    actions = gameState.getLegalActions(self.index)

    '''
    You should change this in your own agent.
    '''

    return random.choice(actions)
