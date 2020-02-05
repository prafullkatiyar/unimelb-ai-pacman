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



# import required libraries
from captureAgents import CaptureAgent
import random, time, util, operator
from util import nearestPoint
from game import Directions, Actions

# power mode, allowed steps
GODMODE = 120


#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first='BiasedUp', second='BiasedDown'):
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

    # The following line is an example only; feel free to change it.
    return [eval(first)(firstIndex), eval(second)(secondIndex)]


##########
# Agents #
##########

class ReflexCaptureAgent(CaptureAgent):

    def __init__(self, gameState):
        CaptureAgent.__init__(self, gameState)
        self.expectation = [None] * 4
        self.godmodeClock = 0

        # step time calculation
        self.timerA = []
        self.timerB = []
        self.counter = 0
        self.foodNum = 0.0

    # check starting position, red or blue
    def registerIndices(self, gameState):
        self.redIndices = gameState.getRedTeamIndices()
        self.blueIndices = gameState.getBlueTeamIndices()

        if not self.red:
            CaptureAgent.registerTeam(self, self.blueIndices)
        else:
            CaptureAgent.registerTeam(self, self.redIndices)

    # initial state, variables and function definition + declaration
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
        self.registerIndices(gameState)

        '''
        Your initialization code goes here, if you need any.
        '''
        # utility variables declaration
        self.utilityVariable(gameState)

        # constant value used for finding defensive co ordinates
        const = self.constants()

        # on a specific X, all Y's with no wall, for pacman to patrol
        i = 0
        while i != self.heightGameSpace-1:
            if self.walls[self.centreX + const][i]:
                g = 0
            else:
                self.entryPoints.append((self.centreX + const, i))
            i = i + 1


        maxIndiceRed = 0
        maxIndiceBlue = 0
        for i in range(len(self.redIndices)):
            if self.redIndices[i] > maxIndiceRed:
                maxIndiceRed = self.redIndices[i]
            if self.blueIndices[i] > maxIndiceBlue:
                maxIndiceBlue = self.redIndices[i]


        # making agent biased, 1 agent moves in top location, other in bottom location
        if self.index == maxIndiceRed:
            x, y = self.entryPoints[3 * len(self.entryPoints) / 4]
        elif self.index == maxIndiceBlue:
            x, y = self.entryPoints[3 * len(self.entryPoints) / 4]
        else:
            x, y = self.entryPoints[1 * len(self.entryPoints) / 4]
        self.target = (x, y)


        #enemy location based on prior belief
        global prior
        prior = [util.Counter()] * self.agentCount

        #initial state, agent at starting points
        for i, val in enumerate(prior):
            for opp in self.getOpponents(gameState):
                if i == opp:
                    index1 = gameState.getInitialAgentPosition(i)
                    prior[i][index1] = 1.0
        self.takePosition(gameState)

    def utilityVariable(self, gameState):

        walls = gameState.getWalls()             # get walls as T & F grid
        walllist = list(walls)                   # True False list of list
        self.walls = walllist

        self.widthGameSpace = len(walllist)      # maze width
        self.heightGameSpace = len(walllist[0])  # maze height

        self.possibleMoves = gameState.getWalls().asList(False)
        self.entryPoints = []                                     # for patrolling points (defence mode)
        self.centreX = (self.widthGameSpace - 1) / 2              # for patrolling and start mode
        self.centreY = (self.heightGameSpace - 1) / 2
        self.agentCount = gameState.getNumAgents()

    #constant, used for finding defence co ordinated
    def constants(self):
        if self.red:
            horizontalFactor = -3
        else:
            horizontalFactor = 4
        return horizontalFactor

    # if enemy is visible: get maze distance b/w enemy and agent
    def distanceToEnemy(self, gameState):
        pos = [gameState.getAgentPosition(enemy) for enemy in self.getOpponents(gameState)
               if gameState.getAgentPosition(enemy) != None]
        if len(pos) > 0:
            miniDistance = float('inf')
            for i in pos:
                distance = self.getMazeDistance(i, gameState.getAgentPosition(self.index))
                if distance < miniDistance:
                    miniDistance = distance
            return miniDistance

    #distance b/w allies
    def allyDistance(self, gameState):
        if self.index == self.agentsOnTeam[0]:
            allyDistance = None
        else:
            allyDistance = self.getMazeDistance(gameState.getAgentState(self.index).getPosition(),
                                                gameState.getAgentState(self.agentsOnTeam[0]).getPosition())
            if allyDistance == 0:
                allyDistance = 0.3
        return allyDistance

    # posterior belief, after observing evidence
    def poterior(self, gameState):
        for agent, b in enumerate(prior):
            if agent in self.getOpponents(gameState):
                posterior = util.Counter()
                pos = gameState.getAgentPosition(agent)
                #visibe points
                if pos != None:
                    posterior[pos] = 1.0
                else:
                    # iterating through prior: prior beliefs
                    for p in b:
                        # check for legal position (non wall)
                        if p in self.possibleMoves and b[p] > 0:
                            probableAct = [(p[0] - 1, p[1]), (p[0] + 1, p[1]), (p[0], p[1] - 1), (p[0], p[1] + 1),
                                          (p[0], p[1])]
                            newActions = []


                            for j in range(len(probableAct)):
                                actions = probableAct[j]
                                for i in range(len(self.possibleMoves)):
                                    if actions == self.possibleMoves[i]:
                                        newActions.append(actions)

                            distance = util.Counter()
                            for i in range(len(newActions)):
                                actions = newActions[i]
                                distance[actions] = 1
                            changedPosDistribution = distance
                            # new probability is, previous value * probability of current location from iteration
                            for x, y in changedPosDistribution:
                                posterior[x, y] += b[p] * changedPosDistribution[x, y]

                    if len(posterior) == 0:
                        previousState = self.getPreviousObservation()
                        if previousState != None and previousState.getAgentPosition(agent) != None:  # just ate an enemy
                            posterior[previousState.getInitialAgentPosition(agent)] = 1.0
                        else:
                            for p in self.possibleMoves: posterior[p] = 1.0
                prior[agent] = posterior

    # locate enemy
    def find(self, agent, noisyDistance, gameState):
        myPos = gameState.getAgentPosition(self.index)
        allPossible = util.Counter()
        # existing prob
        for j in range(len(self.possibleMoves)):
            p = self.possibleMoves[j]
            trueDistance = util.manhattanDistance(p, myPos)
            allPossible[p] += gameState.getDistanceProb(trueDistance, noisyDistance)
        #new prior becomes =  previous prob values(prior) * new prob values
        i = 0
        while i != len(self.possibleMoves):
            p = self.possibleMoves[i]
            i = i + 1
            prior[agent][p] *= allPossible[p]


    # return best move
    # contains function definitions
    def chooseAction(self, gameState):
        # code to find out time interval b.w. 2 steps

        start = time.time()
        if self.index == 1:
           self.timerA.append(start)
           if len(self.timerA) > 2:
               g=0
               time1 = self.timerA[len(self.timerA)-1] - self.timerA[len(self.timerA)-2]
        else:
            self.timerB.append(start)

        self.counter += 1

        # fetch opponents
        opponents = self.getOpponents(gameState)

        # distance that is based on noise
        unknownDistance = gameState.getAgentDistances()

        # for all enemy, use find function to get noise distance
        for i in range(len(opponents)):
            agent = opponents[i]
            self.find(agent, unknownDistance[agent], gameState)

        # entry points are pre selected legal positions for petrolling
        self.positions = [self.entryPoints[len(self.entryPoints) / 2]] * self.agentCount
        for i, blf in enumerate(prior):
            highestVal = 0
            match = 0
            for val in prior[i]:
                if blf[val] == highestVal:
                    if highestVal > 0:
                        match += 1
                elif blf[val] > highestVal:
                    highestVal = blf[val]
                    self.positions[i] = val
            if match > 5:
                self.positions[i] = self.target

        # posterior being normalize(), get pick location with high expectation (max)
        for i in range(len(opponents)):
            agent = opponents[i]
            prior[agent].normalize()
            self.expectation[agent] = max(prior[agent].iteritems(), key=operator.itemgetter(1))[0]
        self.poterior(gameState)
        gameMode = 'offence'

        # if not reached centre, mode = begin, centre = self.takeposition
        if self.takePosition == False:
            gameMode = 'begin'

        # if agent current pos = cetre, change mode to offence
        if gameState.getAgentPosition(self.index) == self.center and self.takePosition == False:
            self.takePosition = True
            gameMode = 'offence'

        # if enmy is a pacman i.e. in our region. killll!!!!
        for i in range(len(opponents)):
            enm = opponents[i]
            if gameState.getAgentState(enm).isPacman:
                gameMode = 'kill'

        # visible enemy positions, visible are enemy in radius of 5
        pos1 = [(invader, gameState.getAgentPosition(invader)) for invader in self.getOpponents(gameState)
                if gameState.getAgentPosition(invader) != None]
        # if enemy visible and agent no pacman go defensive
        if len(pos1) > 0:
            for enemy, pos in pos1:
                if self.getMazeDistance(gameState.getAgentPosition(self.index), pos) < 5 \
                        and not gameState.getAgentState(self.index).isPacman:
                    gameMode = 'defensive'
                    break

        # legal actions = N S E W Stop
        actions = gameState.getLegalActions(self.index)

        # most important part of code: send actions for evaluation
        vals = [self.evaluate(gameState, a, gameMode) for a in actions]

        # based on cost of evaluation select best action
        maxVal = max(vals)
        listActions = [act for act, v in zip(actions, vals) if v == maxVal]
        return random.choice(listActions)

    # baselineTeam code for getting successor
    def getSuccessor(self, gameState, action):
        successor = gameState.generateSuccessor(self.index, action)
        pos = successor.getAgentState(self.index).getPosition()
        if pos != nearestPoint(pos):
            # Only half a grid position was covered
            return successor.generateSuccessor(self.index, action)
        else:
            return successor

    # baselineTeam methodology for creating controller
    def evaluate(self, gameState, action, gameMode):
        if gameMode == 'offence':
            features = self.getFeaturesOffence(gameState, action)
            weights = self.getWeightsOffence()
        elif gameMode == 'defensive':
            features = self.getFeaturesDefensive(gameState, action)
            weights = self.getWeightsDefensive()
        elif gameMode == 'begin':
            features = self.getFeatures(gameState, action)
            weights = self.getWeights()
        elif gameMode == 'kill':
            features = self.getFeaturesKill(gameState, action)
            weights = self.getWeightKill()

        # linear regression logic implementation
        return features * weights

    # based on game mode, this function returns features for action evaluation
    def getFeaturesKill(self, gameState, action):

        # utility variables
        features = util.Counter()
        successor = self.getSuccessor(gameState, action)
        myState = successor.getAgentState(self.index)
        myPos = myState.getPosition()
        opponents = self.getOpponents(gameState)

        # finding and classifying enemies as attackers
        # if enemy is PacMan = attacker
        attackers = []
        for i in range(len(opponents)):
            bot = opponents[i]
            if successor.getAgentState(bot).isPacman:
                attackers.append(bot)

        # number of attackers, heuristic
        features['attackersCount'] = len(attackers)
        ed = 0
        for i in range(len(attackers)):
            bot = attackers[i]
            enemyPos = self.expectation[bot]
            ed = self.getMazeDistance(myPos, enemyPos)


        # distance between agent and enemy heoristic
        features['attackerDistance'] = ed


        # distance betweem 2 agents (allies)
        if successor.getAgentState(self.index).isPacman:
            if self.allyDistance(successor) != None:
                features['friendDistance'] = 1.0 / self.allyDistance(successor)

        # reverse and stop action heuristic
        if action == Directions.STOP:
            features['stop'] = 1
        rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
        if action == rev:
            features['reverse'] = 1

        return features

    # params: action, gameState
    # return: feature values
    def getFeaturesOffence(self, gameState, action):

        # utility variables
        features = util.Counter()                                # baseline
        successor = self.getSuccessor(gameState, action)         # baseline
        myState = successor.getAgentState(self.index)            # baseline
        myPos = myState.getPosition()                            # baseline
        listFood = self.getFood(successor).asList()

        # successor co ordinate heuristic
        features['successorScore'] = self.getScore(successor)

        # distance to close enemy food
        if len(listFood) > 0:
            shortestPath = min([self.getMazeDistance(myPos, yummyBerries) for yummyBerries in listFood])
            features['foodDistance'] = shortestPath

        # food avaialble heuristic
        features['collectFood'] = -len(listFood)

        # current game score heuristic
        features['gameScore'] = 100*self.getScore(successor)

        # distance to invader
        invaderMazeDist = self.distanceToEnemy(successor)
        if not invaderMazeDist:
            pass
        else:
            if invaderMazeDist == 0.5:
                features['risk'] = 8
            elif invaderMazeDist == 1:
                features['risk'] = 4
            if invaderMazeDist == 2:
                features['risk'] = 2
            elif invaderMazeDist <= 4:
                features['risk'] = 1
            else:
                features['risk'] = 0

        # god mode heuristic
        powerCaps = self.getCapsules(successor)
        if (len(powerCaps) > 0):
            minCapsuleDist = min([self.getMazeDistance(myPos, caps) for caps in powerCaps])
            features['getCapsule'] = -len(powerCaps)
        else:
            minCapsuleDist = .1
        features['distanceToCapsule'] = 1.0 / minCapsuleDist

        # Holding food heuristic
        if myPos in self.getFood(gameState).asList():
            self.foodCount += 1.0

        # priximity detection
        pos = gameState.getAgentPosition(self.index)
        agentType = self.index % 2
        if agentType == 1:
            # red side
            if pos[0] < self.widthGameSpace / 2:
                val1 = 1.0
            else:
                val1 = 0.0
        else:
            # blue side
            if pos[0] > self.widthGameSpace / 2 - 1:
                val1 = 1.0
            else:
                val1 = 0.0

        if val1 == 0.0:
            self.foodCount = 0.0

        # food holiding in hand heuristic
        list = []
        p = []
        for i in range(1, self.heightGameSpace):
            if not gameState.hasWall(self.widthGameSpace / 2, i):
                p = (self.widthGameSpace / 2, i)
                list.append(self.distancer.getDistance(myPos, p))
        val = self.foodCount*min(list)
        features['foodInHand'] = val

        # drop food back to safe zone heuristic, based on food available
        features['dropFood'] = self.foodCount * val1

        # powered up - capsule mode
        if myPos in self.getCapsules(gameState):
            self.godmodeClock = GODMODE

        # manage timer
        if self.godmodeClock > 0:
            self.godmodeClock -= 1

        # check powerd up mode; heuristic
        if self.godmodeClock > 0:
            features['isPowered'] = self.godmodeClock / GODMODE
            features['foodInHand'] = 0.0
            features['collectFood'] = 100 * features['collectFood']
        else:
            features['isPowered'] = 0.0

        # distance to ally heuristic
        if successor.getAgentState(self.index).isPacman:
            distanceToAlly = self.allyDistance(successor)
            if distanceToAlly != None:
                features['friendDistance'] = 1.0 / distanceToAlly

        # trap detectio heuristic, where agent can get stuc
        actions = gameState.getLegalActions(self.index)
        if (len(actions) <= 2):
            features['trap'] = 1.0
        else:
            features['trap'] = 0.0

        # pause heuristic
        if action != Directions.STOP:
            features['pause'] = 0.0
        else:
            features['pause'] = 1.0


        return features

    # feature values for defensive heuristic
    def getFeaturesDefensive(self, gameState, action):

        features = util.Counter()
        successor = self.getSuccessor(gameState, action)

        myState = successor.getAgentState(self.index)
        myPos = myState.getPosition()

        # total visible enemies
        enemies = []
        for i in self.getOpponents(successor):
            enemies.append(successor.getAgentState(i))

        # find attacking enemies
        invaders=[]
        for enemy in enemies:
            if enemy.isPacman and enemy.getPosition() != None:
                invaders.append(enemy)

        # number of attackers heuristic
        features['attackersCount'] = len(invaders)
        if len(invaders) > 0:
            enemyDist = []
            for enemy in invaders:
                enemyDist.append(self.getMazeDistance(myPos, enemy.getPosition()))
            features['attackerDistance'] = min(enemyDist)

        # Compute distance to enemy
        distEnemy = self.distanceToEnemy(successor)
        if (distEnemy <= 5):
            features['risk'] = 1
            scaredTimer = gameState.getAgentState(self.index).scaredTimer
            if distEnemy <= 1 and scaredTimer > 0:
                features['risk'] = -1
        else:
            features['risk'] = 0

        # distance to ally heuristic
        if successor.getAgentState(self.index).isPacman:
            distanceToAlly = self.allyDistance(successor)
            if distanceToAlly != None:
                features['friendDistance'] = 1.0 / distanceToAlly

        if action == Directions.STOP:
            features['stop'] = 1
        rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
        if action == rev: features['reverse'] = 1

        return features

    # game start mode
    # param:  gameState, action
    # returns; features
    def getFeatures(self, gameState, action):
        features = util.Counter()
        successor = self.getSuccessor(gameState, action)

        myState = successor.getAgentState(self.index)
        myPos = myState.getPosition()

        # distance to centre position heuristic
        dist = self.getMazeDistance(myPos, self.center)
        features['distToCenter'] = dist

        if myPos == self.center:
            features['atCenter'] = 1
        return features

    # returns weigts for for offence features
    def getWeightsOffence(self):
        return {'foodDistance': -11,  'trap': -199, 'successorScore': 799,
                'getCapsule': 4999, 'collectFood': 3999, 'distanceToCapsule': 699,
                'pause': -999, 'distToAlly': -5999, 'isPowered': 4999999,
                'dropFood': 99, 'foodInHand': -20, 'gameScore': 3999, 'risk': -999}


    # returns weights for killll!! mode features
    def getWeightKill(self):
        return {'attackersCount': -99, 'attackerDistance': -9, 'stop': -4999,
                'reverse': -4999, 'friendDistance': -2499}

    # returns weights for defense mode features.
    def getWeightsDefensive(self):
        return {'attackersCount': -9999, 'attackerDistance': -499, 'stop': -4999,
                'reverse': -199, 'risk': 2999, 'friendDistance': -3999}

    # weight to reach centre, at the start of game
    def getWeights(self):
        return {'distToCenter': -1, 'atCenter': 999}

'''
bot the agents work offensive and defensive, the agent starts by taking 
centre position.
'''
class BiasedUp(ReflexCaptureAgent):

    def takePosition(self, gameState):
        positions = []
        self.takePosition = False
        centreX = gameState.getWalls().width / 2
        centreY = gameState.getWalls().height / 2
        # 0 to x-1 and x to width
        if self.red:
            centreX -= 1
        # centre marked at half of maze width and height
        self.center = (centreX, centreY)
        maxHeight = gameState.getWalls().height

        i = 0
        while i < (maxHeight - centreY):
            if not gameState.hasWall(centreX, centreY):
                positions.append((centreX, centreY))
            centreY = centreY + 1
            i += 1

        # agents current position
        myPos = gameState.getAgentState(self.index).getPosition()
        leastDistance = float('inf')
        minPos = None

        # new target position from possible positions selected before
        for i in range(len(positions)):
            location = positions[i]
            distance = self.getMazeDistance(myPos, location)
            if distance > leastDistance:
                pass
            else:
                leastDistance = distance
                minPos = location

        self.center = minPos


class BiasedDown(ReflexCaptureAgent):

    def takePosition(self, gameState):
        positions = []
        self.takePosition = False
        centreX = gameState.getWalls().width / 2
        centreY = gameState.getWalls().height / 2
        if self.red:
            centreX = centreX - 1
        # Set where the centre is
        self.center = (centreX, centreY)
        for i in range(centreY):
            if not gameState.hasWall(centreX, centreY):
                positions.append((centreX, centreY))
            centreY = centreY - 1

        # current agent position
        myPos = gameState.getAgentState(self.index).getPosition()
        leastDistance = float('inf')
        minPos = None

        # new target position from possible positions selected before
        for i in range(len(positions)):
            location = positions[i]
            dist = self.getMazeDistance(myPos, location)
            if dist > leastDistance:
                pass
            else:
                leastDistance = dist
                minPos = location

        self.center = minPos