# Missionaries and Cannibals problem
# By Lauren Dennedy & Matthew Stanford
# gr5743 & gd9687

# We are using a Tree structure to represent the states and 
# actions in our AI, as well as using a depth-limited search with
# an average diameter of 12 to search for solutions

# Node class provided by "Artificial Intelligence: A Modern Approach"
# This class is used in main to create the Tree for the searching algorithm

# Code is from https://github.com/aimacode/aima-python/blob/master/search.py
# Starting on line 68 to line 130

class Node:
    """A node in a search tree. Contains a pointer to the parent (the node
    that this is a successor of) and to the actual state for this node. Note
    that if a state is arrived at by two paths, then there are two nodes with
    the same state. Also includes the action that got us to this state, and
    the total path_cost (also known as g) to reach the node. Other functions
    may add an f and h value; see best_first_graph_search and astar_search for
    an explanation of how the f and h values are handled. You will not need to
    subclass this class."""

    def __init__(self, state, parent=None, action=None, path_cost=0):
        """Create a search tree Node, derived from a parent by an action."""
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node {}>".format(self.state)

    def __lt__(self, node):
        return self.state < node.state

    def expand(self, problem):
        """List the nodes reachable in one step from this node."""
        return [self.child_node(problem, action)
                for action in problem.actions(self.state)]

    def child_node(self, problem, action):
        """[Figure 3.10]"""
        next_state = problem.result(self.state, action)
        next_node = Node(next_state, self, action, problem.path_cost(self.path_cost, self.state, action, next_state))
        return next_node

    def solution(self):
        """Return the sequence of actions to go from the root to this node."""
        return [node.action for node in self.path()[1:]]

    def path(self):
        """Return a list of nodes forming the path from the root to this node."""
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))

    # We want for a queue of nodes in breadth_first_graph_search or
    # astar_search to have no duplicated states, so we treat nodes
    # with the same state as equal. [Problem: this may not be what you
    # want in other contexts.]

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __hash__(self):
        # We use the hash value of the state
        # stored in the node instead of the node
        # object itself to quickly search a node
        # with the same state in a Hash Table
        return hash(self.state)

# Problem class provided by "Artificial Intelligence: A Modern Approach"
# This class is used in main as a superclass structure for defining our problem

# Code is from https://github.com/aimacode/aima-python/blob/master/search.py
# Starting on line 15 to line 62

class Problem:
    """The abstract class for a formal problem. You should subclass
    this and implement the methods actions and result, and possibly
    __init__, goal_test, and path_cost. Then you will create instances
    of your subclass and solve them with the various search functions."""

    def __init__(self, initial, goal=None):
        """The constructor specifies the initial state, and possibly a goal
        state, if there is a unique goal. Your subclass's constructor can add
        other arguments."""
        self.initial = initial
        self.goal = goal

    def actions(self, state):
        """Return the actions that can be executed in the given
        state. The result would typically be a list, but if there are
        many actions, consider yielding them one at a time in an
        iterator, rather than building them all at once."""
        raise NotImplementedError

    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""
        raise NotImplementedError

    #the original code had to be modified, as is_in was giving an error
    def goal_test(self, state):
        """Return True if the state is a goal. The default method compares the
        state to self.goal or checks for state in self.goal if it is a
        list, as specified in the constructor. Override this method if
        checking against a single self.goal is not enough."""
        if isinstance(self.goal, list):
            if (state == self.goal):
                return True
            return False
        else:
            return state == self.goal 

    def path_cost(self, c, state1, action, state2):
        """Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2. If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path."""
        return c + 1

    def value(self, state):
        """For optimization problems, each state has a value. Hill Climbing
        and related algorithms try to maximize this value."""
        raise NotImplementedError

# Below is the implementation of our own Problem subclass

# Encoding method for the data of the missionaries and cannibals

# A node that contains {m, c, b} as the number of 
# missionaries on the wrong side, number of cannibals on 
# the wrong side, and if the boat is on the right or wrong 
# side. Initial state is {3, 3, 1}, goal is {0, 0, 0}

from operator import add

class MCProblem(Problem):
    def __init__(self, initial, goal=None):
        super().__init__(initial, goal)

    def actions(self, state):
        # This function will return actions that can be executed given the current state

        # This is used in results
        actions = []

        if (state[2] == 1):     # If boat is on wrong side
            # Two possible actions based on this condition

            # A missionary and a cannibal going together:
            # Δ [-1, -1, -1]
            if (state[0] >= state[1]):      # m and c are equal
                print("A missionary and a cannibal sail to the right side of the shore.")
                actions.append([-1, -1, -1])
                print("Encoding Change: ", actions[0])

            # 2 cannibals going together:
            # Δ [0, -2, -1]
            elif (state[0] < state[1]):     # m and c are not equal
                print("Two cannibals sail to the right side of the shore.")
                actions.append([0, -2, -1])
                print("Encoding Change: ", actions[0])
                
            
        elif (state[2] == 0): # If boat is on right side
            # Three possible actions based on this condition

            if (state[1] < 2):      # If 2 cannibals just went:
                # Action is to leave one cannibal and bring the other back
                # Δ [0, +1, +1]
                print("A cannibal is left behind at the right side of the shore, and a cannibal sails back to the wrong side of the shore.")
                actions.append([0, 1, 1])
                print("Encoding Change: ", actions[0])
                
            else:                   # If a missionary and a cannibal just went
                if (state[0] == state[1]):
                    # If missionaries on right side outnumber cannibals, 
                    # leave cannibal
                    # Δ [+1, 0, +1]
                    print("A cannibal is left behind at the right side of the shore, and a missionary sails back to the wrong side of the shore.")
                    actions.append([1, 0, 1])
                    print("Encoding Change: ", actions[0])
                    
                else:
                    # If cannibals on right side == missionaries on right 
                    # side, leave missionary
                    # Δ [0, +1, +1]
                    print("A missionary is left behind at the right side of the shore, and a cannibal sails back to the wrong side of the shore.")
                    actions.append([0, 1, 1])
                    print("Encoding Change: ", actions[0])

        else:
            # This is an error because the boat will be a value other than 1 or 0, we'll throw some exception here if there's an error
            raise ValueError("The boat is neither on the right or wrong side.")

        return actions
    
    def result(self, state, action):
        # Returns the state that results from executing the given action in the given state. The action must be one of the self.actions(state)

        # The way the encoding method is implemented, as [1, 2, 1] for example, allows for changes like this:

        # [1, 2, 1]
        # [0, -1, -1]
        # + _______
        # [1, 1, 0]

        # This line performs the list arithmetic as shown above
        result = list(map(add, state, action))

        # Printing data for the resulting state to show steps in the solution
        print("New State: ", result)
        print()
        print(result[0], "missionary(ies) on the wrong side of the shore,",
                3-result[0], "missionary(ies) on the right side of the shore.")

        print()
        print(result[1], "cannibal(s) on the wrong side of the shore,",
                3-result[1], "cannibal(s) on the right side of the shore.")

        print()

        if (result[2]==0):
            print("The boat is on the right side of the shore.")

        elif (result[2]==1):
            print("The boat is on the wrong side of the shore")

        else:
            print("The boat is lost in space and time.")

        print()
        print("----------------------------------------------------------")
        print()
        return result
    
    def goal_test(self, state):
        return super().goal_test(state)

    def path_cost(self, c, state1, action, state2):
        return super().path_cost(c, state1, action, state2)

# Creating an instance/object of our MCProblem class

MCProblem = MCProblem([3, 3, 1], [0, 0, 0])

# Starting output
print("3 missionaries and 3 cannibals are on one side (the wrong side) of a river.")
print("Current state: [3, 3, 1]")
print()
print("----------------------------------------------------------")
print()

# Depth Limited Searching algorithm from 
# https://github.com/aimacode/aima-python/blob/master/search.py
# Starting on line 295 to line 314

def depth_limited_search(problem, limit=12):
    def recursive_dls(node, problem, limit):
        if problem.goal_test(node.state): # Here's the goal test
            return node
        elif limit == 0: # This is if it runs out of moves
            print("The algorithm couldn't find a solution within 12 steps.")
            return 'cutoff'
        else:
            cutoff_occurred = False
            for child in node.expand(problem):
                result = recursive_dls(child, problem, limit - 1)
                if result == 'cutoff':
                    cutoff_occurred = True
                elif result is not None:
                    return result
            return 'cutoff' if cutoff_occurred else None

    # Body of depth_limited_search:
    return recursive_dls(Node(problem.initial), problem, limit)

# Finally, we call the search on our custom problem object
depth_limited_search(MCProblem)