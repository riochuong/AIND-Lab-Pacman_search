# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
"""

import util
from util import PriorityQueue


class SearchProblem:
  """
  This class outlines the structure of a search problem, but doesn't implement
  any of the methods (in object-oriented terminology: an abstract class).

  You do not need to change anything in this class, ever.
  """

  def getStartState(self):
    """
    Returns the start state for the search problem 
    """
    util.raiseNotDefined()

  def isGoalState(self, state):
    """
      state: Search state

    Returns True if and only if the state is a valid goal state
    """
    util.raiseNotDefined()

  def getSuccessors(self, state):
    """
      state: Search state

    For a given state, this should return a list of triples, 
    (successor, action, stepCost), where 'successor' is a 
    successor to the current state, 'action' is the action
    required to get there, and 'stepCost' is the incremental 
    cost of expanding to that successor
    """
    util.raiseNotDefined()

  def getCostOfActions(self, actions):
    """
     actions: A list of actions to take

    This method returns the total cost of a particular sequence of actions.  The sequence must
    be composed of legal moves
    """
    util.raiseNotDefined()


def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return [s, s, w, s, w, w, s, w]


# helper to add action string to the list 
def add_action(actions, act_str):
  if act_str == "West":
    actions.append(w)
  if act_str == "South":
    actions.append(s)
  if act_str == "East":
    actions.append(e)
  if act_str == "North":
    actions.append(n)
  return actions



def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first
  [2nd Edition: p 75, 3rd Edition: p 87]

  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm 
  [2nd Edition: Fig. 3.18, 3rd Edition: Fig 3.7].

  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:

  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  """
  "*** YOUR CODE HERE ***"
  # get directions symbol
  from game import Directions
  from util import Stack 
  
  LOC = 0
  ACTION = 1
  COST = 2
  COME_FROM = 3
  """
    Dfs does all the traversing work
  """
          

  #print "Start:", problem.getStartState()
  #print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  #print "Start's successors:", problem.getSuccessors(problem.getStartState())
  # states of the search
  current_state = problem.getStartState()
  # adding another element for keep tracking of where we come from
  current_state = (current_state, None,0,None)
  frontiers = Stack()
  frontiers.push(current_state)
  explored = set()
  destination = None
  actions = []
  
  # go through each frontier
  while not frontiers.isEmpty():
    new_frontier = frontiers.pop()
    # skip frontier if it's been explored
    if (new_frontier[LOC] in explored):
      continue

    # mark as explored
    explored.add(new_frontier[LOC])
    
    # check if we hit goal state
    if problem.isGoalState(new_frontier[LOC]):
      destination = new_frontier
      break

    # add new frontiers
    for next_frontier in problem.getSuccessors(new_frontier[LOC]):
      if not (next_frontier[LOC] in explored):
        # mark where it came from
        # print("push frontier ", next_frontier[LOC], "from ", new_frontier[LOC])
        # print("explored ",explored)
        # print("")
        next_frontier += (new_frontier,)
        frontiers.push(next_frontier)

    # if we found destination reconstruct the path
  if destination:
    print(destination)
    while destination[COME_FROM]:
      actions.append(destination[ACTION])
      destination = destination[COME_FROM]
    actions = [act for act in reversed(actions)]

 # print("solution ",actions)
  return actions

  util.raiseNotDefined()


def breadthFirstSearch(problem):
  """
  Search the shallowest nodes in the search tree first.
  [2nd Edition: p 73, 3rd Edition: p 82]
  """

  from util import Queue
  LOC = 0
  ACTION = 1
  COST = 2
  COME_FROM = 3
  "*** YOUR CODE HERE ***"
  initial_state = problem.getStartState()
  if (problem.isGoalState(initial_state)):
    print("We are at the solution")
    return []
  initial_state = (initial_state,None,0,None)
  frontiers = Queue()
  frontiers.push(initial_state)
  explored = set()
  actions = []
  last_frontier = None
  found_solution = False
  
  while(not frontiers.isEmpty()):
    frontier = frontiers.pop()
    # there might be case frontier has two same values
    if (frontier[LOC] in explored):
      continue
    if (problem.isGoalState(frontier[LOC])):
          last_frontier = frontier
          found_solution = True
          break
    # explore next
    explored.add(frontier[LOC])
    for new_frontier in problem.getSuccessors(frontier[LOC]):
      if not (new_frontier[LOC] in explored):
        new_frontier += (frontier,)
        # update new front tier
        frontiers.push(new_frontier)

  # back track the path
  if (last_frontier):
    actions.append(last_frontier[ACTION])
    parent = last_frontier[COME_FROM]
    # keep extracting
    while parent[COME_FROM]:
      actions.append(parent[ACTION])
      parent = parent[COME_FROM]

  return [act for act in reversed(actions)]

  util.raiseNotDefined()


LOC = 0
ACTION = 1
COST = 2
COME_FROM = 3
PATH_COST = 4

class Frontier :
  def __init__(self):
    self.queue = PriorityQueue()
    self.ht = {}

  def pop(self):
    while len(self.ht):
      val = self.queue.pop()
      # delete from ht also
      #print(self.ht)
      #print("pop:", val, " val loc",val[LOC])
      
      # discard item that is not in the dict
      if (val[LOC] not in self.ht.keys()):
        continue
      
      del self.ht[val[LOC]]
      # clean up queue 
      if (not len(self.ht)):
        while (not self.queue.isEmpty()):
          self.queue.pop()

      print("pop val", val)
      return val
    else:
      # clean up and pop queue
      while (not self.queue.isEmpty()):
        self.queue.pop()
    return None

  def push(self, item):
    # push item to queue with priority as cost
    #print("push", item, "loc ",item[LOC])
    self.queue.push(item, item[COST])
    # keep track item cost in frontier 
    self.ht[item[LOC]] = item[COST]

  def getItemCost(self, key):
    if (key in self.ht):
      return self.ht[key]

  def isEmpty(self):
    return (len(self.ht) == 0)

  # check if key is in frontier 
  def isInFrontier(self, key):
    return key in self.ht

def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  
  # this class represent a frontier 
  # to support member ship check and 
  # retreived lowest cost path 


  # start the project here 
  
  initial_state = problem.getStartState()
  initial_state = (initial_state, None, 0, None)
  frontiers = Frontier()
  frontiers.push(initial_state)
  explored = set()
  destination = None
  actions = []

  # main loop 
  while (not frontiers.isEmpty()):
    next_frontier = frontiers.pop()
    #print("Next Frontier", next_frontier)

    # skip to next 
    if (next_frontier[LOC] in explored):
      continue
    # check goal state
    if (problem.isGoalState(next_frontier[LOC])):
      #print("found destination", next_frontier[LOC])
      destination = next_frontier
      break

    # add to explored
    explored.add(next_frontier[LOC])

    # update frontiers
    for new_frontier in problem.getSuccessors(next_frontier[LOC]):
      # upate cost for the new possible frontier
      #print("new frontier", new_frontier)
      new_cost = new_frontier[COST] + next_frontier[COST]
      # hacky way to update the cost
      convert_obj = list(new_frontier)
      convert_obj[COST] = new_cost
      new_frontier = tuple(convert_obj)
      # update to know where they come from 
      new_frontier += (next_frontier,)
      if (not new_frontier[LOC] in explored) and (not frontiers.isInFrontier(new_frontier[LOC])):
        frontiers.push(new_frontier)
      # check if we found a shorter path 
      elif frontiers.isInFrontier(new_frontier[LOC]) \
        and (new_frontier[COST] < frontiers.getItemCost(new_frontier[LOC])):
        # update the frontier in the tree 
        frontiers.push(new_frontier)

  # now reconstruct the path 
  # back track the path
  if destination:
    print("Destination", destination)
    while destination[COME_FROM]:
      actions.append(destination[ACTION])
      destination = destination[COME_FROM]
    actions = [act for act in reversed(actions)]
  
  return actions

  util.raiseNotDefined()


def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0


def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"
  "Search the node of least total cost first. "
  
  # this class represent a frontier 
  # to support member ship check and 
  # retreived lowest cost path 


  # start the project here 
  
  initial_state = problem.getStartState()
  initial_state = (initial_state, None, heuristic(initial_state,problem), None,0)
  #print('initial state', initial_state)
  frontiers = Frontier()
  frontiers.push(initial_state)
  explored = set()
  destination = None
  actions = []

  # main loop 
  while (not frontiers.isEmpty()):
    next_frontier = frontiers.pop()
    #print("Next Frontier", next_frontier)

    # skip to next 
    if (next_frontier[LOC] in explored):
      continue
    # check goal state
    if (problem.isGoalState(next_frontier[LOC])):
      #print("found destination", next_frontier[LOC])
      destination = next_frontier
      break

    # add to explored
    explored.add(next_frontier[LOC])

    # update frontiers
    for new_frontier in problem.getSuccessors(next_frontier[LOC]):
      # upate cost for the new possible frontier
      h_val = heuristic(new_frontier[LOC], problem)
      new_path_cost = new_frontier[COST] + next_frontier[PATH_COST] 
      new_cost = new_path_cost + h_val
      # hacky way to update the cost
      convert_obj = list(new_frontier)
      convert_obj[COST] = new_cost
      new_frontier = tuple(convert_obj)
      # update to know where they come from 
      new_frontier += (next_frontier,)
      new_frontier += (new_path_cost,)
      if (not new_frontier[LOC] in explored) and (not frontiers.isInFrontier(new_frontier[LOC])):
        frontiers.push(new_frontier)
      # check if we found a shorter path 
      elif frontiers.isInFrontier(new_frontier[LOC]) \
        and (new_frontier[COST] < frontiers.getItemCost(new_frontier[LOC])):
        # update the frontier in the tree 
        frontiers.push(new_frontier)

  # now reconstruct the path 
  # back track the path
  if destination:
    print("Destination", destination)
    while destination[COME_FROM]:
      actions.append(destination[ACTION])
      destination = destination[COME_FROM]
    actions = [act for act in reversed(actions)]
  
  return actions

  util.raiseNotDefined()
  util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
