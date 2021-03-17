from queue import PriorityQueue
from bitarray import bitarray

# list of agents all possible actions
array_of_actions = ['patrol', 'approach_to_shoot', 'approach_to_hit', 'hit', 'aim', 'shoot', 'load', 'searchgun']
# list of agents all possible values of state
array_of_values = ['hasgun', 'enemyvisible', 'enemyinshootrange', 'nearenemy', 'weaponloaded', 'enemylinedup', 'enemyalive', 'alive']

# each action has dicts of preconditions and postconditions (key in array_of_values, value is bool) and cost (default 1)
class Action:
	def __init__(self, pre_conds, post_conds, cost = 1):
		self.pre_conds = pre_conds
		self.post_conds = post_conds
		self.cost = cost

# values of paricular worldstate are organized as bits in bitarray
# care_mask - mask which values are relevant for the worldstate (because without it each value should be True, False or Irrelevant)
class WorldState:
	def __init__(self):
		self.values = bitarray(len(array_of_values))
		self.values.setall(False)
		self.care_mask = bitarray(len(array_of_values))
		self.care_mask.setall(False)
	def set(self, value_name, value):
		index = array_of_values.index(value_name)
		self.values[index] = value
		self.care_mask[index] = True
	def __hash__(self):
		return hash((self.values.to01(), self.care_mask.to01()))
	def __eq__(self, other):
		return self.values & other.care_mask == other.values & other.care_mask
	def __lt__(self, other):
		return (self.values & other.care_mask).count() < (other.values & other.care_mask).count()
	def print(self):
		for i in range(len(array_of_values)):
			if i != len(array_of_values) - 1:
				print(f"{array_of_values[i]} : {self.values[i]}", end="; ") 
			else:
				print(f"{array_of_values[i]} : {self.values[i]}")

# actionplanner, which can be initialied with add_pre_cond, add_post_cond and set_cost methods
# get_possible_transitions and do_transition are required for A* algorithm
class ActionPlanner:
	def __init__(self):
		self.actions = {action_name : Action({}, {}) for action_name in array_of_actions}
	def add_pre_cond(self, action, pre_cond, value):
		self.actions[action].pre_conds[pre_cond] = value
	def add_post_cond(self, action, post_cond, value):
		self.actions[action].post_conds[post_cond] = value
	def set_cost(self, action, cost):
		self.actions[action].cost = cost
	def get_possible_transitions(self, ws):
		possible_transitions = []
		for action_name, action in self.actions.items():
			count = 0
			for name, pre_cond in action.pre_conds.items():
				if pre_cond == ws.values[array_of_values.index(name)]:
					count += 1
			if count == len(action.pre_conds):
				possible_transitions.append(action_name)
		return possible_transitions
	def do_transition(self, ws, action_name):
		new_ws = WorldState()
		new_ws.values = ws.values.copy()
		new_ws.care_mask = ws.care_mask.copy()
		for post_cond_name, value in self.actions[action_name].post_conds.items():
			new_ws.set(post_cond_name, value)
		return new_ws

# heuristic for A* algorithm
def heuristic(current, goal):
	return (current.values & goal.care_mask).count(True) - (goal.values & goal.care_mask).count(True)

def a_star_search(action_planner, begin, goal):
	frontier = PriorityQueue()
	frontier.put(begin, 0)
	came_from = {}
	cost_so_far = {}
	came_from[begin] = None
	cost_so_far[begin] = 0
	while not frontier.empty():
		current = frontier.get()

		if current == goal:
			temp_node = came_from.pop(current, None)
			temp_cost = cost_so_far.pop(current, None)
			came_from[goal] = temp_node
			cost_so_far[goal] = temp_cost
			break
			
		for next in action_planner.get_possible_transitions(current):
			temp_ws = action_planner.do_transition(current, next)
			new_cost = cost_so_far[current] + action_planner.actions[next].cost
			if temp_ws not in cost_so_far or new_cost < cost_so_far[temp_ws]:
				cost_so_far[temp_ws] = new_cost
				priority = new_cost + heuristic(temp_ws, goal)
				frontier.put(temp_ws, priority)
				came_from[temp_ws] = current
    
	return came_from, cost_so_far

# needed to reconstruct sequence of agents actions in right order 
def reconstruct_path(came_from, begin, goal):
    current = goal
    path = []
    while current != begin:
        path.append(current)
        current = came_from[current]
    path.append(begin) 
    path.reverse() 
    return path


if __name__ == "__main__":
	ap = ActionPlanner()

	ap.add_pre_cond("patrol", "nearenemy", False)
	ap.add_pre_cond("patrol", "enemyinshootrange", False)
	ap.add_post_cond("patrol", "enemyvisible", True)

	ap.add_pre_cond("approach_to_shoot", "enemyvisible", True)
	ap.add_pre_cond("approach_to_shoot", "hasgun", True)
	ap.add_post_cond("approach_to_shoot", "enemyinshootrange", True)

	ap.add_pre_cond("aim", "enemyvisible", True)
	ap.add_pre_cond("aim", "enemyinshootrange", True)
	ap.add_pre_cond("aim", "weaponloaded", True)
	ap.add_post_cond("aim", "enemylinedup", True)

	ap.add_pre_cond("shoot", "enemylinedup", True)
	ap.add_post_cond("shoot", "enemyalive", False)

	ap.add_pre_cond("searchgun", "hasgun", False)
	ap.add_pre_cond("searchgun", "enemyvisible", False)
	ap.add_post_cond("searchgun", "hasgun", True)

	ap.add_pre_cond("load", "hasgun", True)
	ap.add_post_cond("load", "weaponloaded", True)

	ap.add_pre_cond("approach_to_hit", "enemyvisible", True)
	ap.add_pre_cond("approach_to_hit", "hasgun", False)
	ap.add_post_cond("approach_to_hit", "nearenemy", True)

	ap.add_pre_cond("hit", "nearenemy", True)
	ap.add_post_cond("hit", "enemyalive", False)


	begin = WorldState()
	begin.set("enemyvisible", False)
	begin.set("hasgun", False)
	begin.set("weaponloaded", False)
	begin.set("enemylinedup", False)
	begin.set("enemyalive", True)
	begin.set("enemyinshootrange", False)
	begin.set("nearenemy", False)
	begin.set("alive", True)

	goal = WorldState()
	goal.set("enemyalive", False)



	came_from, cost_so_far = a_star_search(ap, begin, goal)

	results = reconstruct_path(came_from, begin, goal)

	for ws in results:
		ws.print()