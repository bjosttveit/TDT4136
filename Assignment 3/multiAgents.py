# multiAgents.py
# --------------
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


from util import manhattanDistance
from game import Directions
import random, util

from game import Agent

class ReflexAgent(Agent):
    """
    A reflex agent chooses an action at each choice point by examining
    its alternatives via a state evaluation function.

    The code below is provided as a guide.  You are welcome to change
    it in any way you see fit, so long as you don't touch our method
    headers.
    """


    def getAction(self, gameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {NORTH, SOUTH, WEST, EAST, STOP}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices) # Pick randomly among the best

        "Add more of your code here if you want to"

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState, action):
        """
        Design a better evaluation function here.

        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.

        Print out these variables to see what you're getting, then combine them
        to create a masterful evaluation function.
        """
        # Useful information you can extract from a GameState (pacman.py)
        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        newGhostStates = successorGameState.getGhostStates()
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

        "*** YOUR CODE HERE ***"
        return successorGameState.getScore()

def scoreEvaluationFunction(currentGameState):
    """
    This default evaluation function just returns the score of the state.
    The score is the same one displayed in the Pacman GUI.

    This evaluation function is meant for use with adversarial search agents
    (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
    This class provides some common elements to all of your
    multi-agent searchers.  Any methods defined here will be available
    to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

    You *do not* need to make any changes here, but you can if you want to
    add functionality to all your adversarial search agents.  Please do not
    remove anything, however.

    Note: this is an abstract class: one that should not be instantiated.  It's
    only partially specified, and designed to be extended.  Agent (game.py)
    is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
    """
    Your minimax agent (question 2)
    """

    def maxValue(self, depth, gameState):
        #Check for terminal state
        if gameState.isWin() or gameState.isLose():
            return gameState.getScore(), None
        
        v = float('-inf')
        a = None

        #Iterate over all possible actions
        actions = gameState.getLegalActions(0)
        for action in actions:
            #Generate successor state for action
            successor = gameState.generateSuccessor(0, action)
            v2, _ = self.minValue(depth - 1, successor, 1)
            #If new utility is greater, update
            if v2 > v:
                v, a = v2, action
        return v, a
    
    #Gets run for each ghost by specifying 'agent'
    def minValue(self, depth, gameState, agent):
        #Check for terminal state
        if gameState.isWin() or gameState.isLose():
            return gameState.getScore(), None

        v, v2 = float('inf'), 0
        numAgents = gameState.getNumAgents()

        #Iterate over all possible actions
        actions = gameState.getLegalActions(agent)
        for action in actions:
            #Generate successor state for action
            successor = gameState.generateSuccessor(agent, action)

            #If this is the last ghost
            if agent == numAgents - 1:
                #If reached depth-limit, return utility
                if depth == 0:
                    v2 = self.evaluationFunction(successor)
                #Else it is pacmans turn again
                else:
                    v2, _ = self.maxValue(depth, successor)
            #If there are more ghosts, run minValue again with same depth but next ghost
            else:
                v2, _ = self.minValue(depth, successor, agent + 1)
            
            #If new utility is less, update
            v = min(v, v2)

        return v, None

    def getAction(self, gameState):
        """
        Returns the minimax action from the current gameState using self.depth
        and self.evaluationFunction.

        Here are some method calls that might be useful when implementing minimax.

        gameState.getLegalActions(agentIndex):
        Returns a list of legal actions for an agent
        agentIndex=0 means Pacman, ghosts are >= 1

        gameState.generateSuccessor(agentIndex, action):
        Returns the successor game state after an agent takes an action

        gameState.getNumAgents():
        Returns the total number of agents in the game

        gameState.isWin():
        Returns whether or not the game state is a winning state

        gameState.isLose():
        Returns whether or not the game state is a losing state
        """
        "*** YOUR CODE HERE ***"
        _, a = self.maxValue(self.depth, gameState)
        return a

class AlphaBetaAgent(MultiAgentSearchAgent):
    """
    Your minimax agent with alpha-beta pruning (question 3)
    """

    def maxValue(self, depth, gameState, alpha, beta):
        #Check for terminal state
        if gameState.isWin() or gameState.isLose():
            return gameState.getScore(), None
        
        v = float('-inf')
        a = None

        #Iterate over all possible actions
        actions = gameState.getLegalActions(0)
        for action in actions:
            #Generate successor state for action
            successor = gameState.generateSuccessor(0, action)
            v2, _ = self.minValue(depth - 1, successor, 1, alpha, beta)
            #If new utility is greater, update utility and alpha
            if v2 > v:
                v, a = v2, action
                alpha = max(alpha, v)
            #If utility is greater than beta, return and stop iterating
            if v > beta:
                return v, a
        return v, a
    
    def minValue(self, depth, gameState, agent, alpha, beta):
        #Check for terminal state
        if gameState.isWin() or gameState.isLose():
            return gameState.getScore(), None

        v, v2 = float('inf'), 0
        numAgents = gameState.getNumAgents()

        #Iterate over all possible actions
        actions = gameState.getLegalActions(agent)
        for action in actions:
            #Generate successor state for action
            successor = gameState.generateSuccessor(agent, action)
            
            #If this is the last ghost
            if agent == numAgents - 1:
                #If reached depth-limit, return utility
                if depth == 0:
                    v2 = self.evaluationFunction(successor)
                #Else it is pacmans turn again
                else:
                    v2, _ = self.maxValue(depth, successor, alpha, beta)
            #If there are more ghosts, run minValue again with same depth but next ghost
            else:
                v2, _ = self.minValue(depth, successor, agent + 1, alpha, beta)
            
            #If new utility is less, update utility and beta
            if v2 < v:
                v = v2
                beta = min(beta, v)
            
            #If utility is less than alpha, return and stop iterating
            if v < alpha:
                return v, None

        return v, None

    def getAction(self, gameState):
        """
        Returns the minimax action using self.depth and self.evaluationFunction
        """
        "*** YOUR CODE HERE ***"
        _, a = self.maxValue(self.depth, gameState, float('-inf'), float('inf'))
        return a

class ExpectimaxAgent(MultiAgentSearchAgent):
    """
      Your expectimax agent (question 4)
    """

    def getAction(self, gameState):
        """
        Returns the expectimax action using self.depth and self.evaluationFunction

        All ghosts should be modeled as choosing uniformly at random from their
        legal moves.
        """
        "*** YOUR CODE HERE ***"
        util.raiseNotDefined()

def betterEvaluationFunction(currentGameState):
    """
    Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
    evaluation function (question 5).

    DESCRIPTION: <write something here so we know what you did>
    """
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

# Abbreviation
better = betterEvaluationFunction
