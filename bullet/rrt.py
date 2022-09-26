import numpy as np
from robosuite.utils.mjcf_utils import BLUE
import random
import torch
# from ..rts.envs.robots.load_robot import load_sinlge_robot_arm_params


class Node:

    def __init__(self, pos, parent=None):
        self.pos = np.array(pos)
        self.parent = parent

    def euclidean_distance(self, node):
        d = (node.pos-self.pos + np.pi) % (2*np.pi) - \
            np.pi  # Wrap between -pi and pi
        return np.linalg.norm(d)

    def __str__(self):
        return f"pos: {self.pos}\nparent:{self.parent.__repr__()}"

    def __hash__(self):
        return hash(self.pos.data.tobytes)

    def __eq__(self, other):
        return self.euclidean_distance(other) <= 0.03


class BuildRRT:

    def __init__(self, env, start, goal, goal_bias, step):
        self.env = env
        self.start = Node(start, None)
        self.goal = Node(goal, None)
        self.step_size = step
        self.current = self.start
        self.goal_bias = goal_bias
        self.solution = []

        self.dof_limits = []
        for joint in range(7):
            self.dof_limits.append(np.array([self.env.ll[joint], self.env.ul[joint]]))
        self.dof_limits = np.array(self.dof_limits)
        self.tree_list = [self.current]

        print("******** RRT PARAMS *********")
        print(f"Start QPos: {self.start.pos}")
        print(f"Goal QPos: {self.goal.pos}")
        print(f"Goal Bias: {self.goal_bias}")
        print(f"Step Size: {self.step_size}")

    '''
    def _load_robots(self):
        """
        Instantiates robots and stores them within the self.robots attribute
        """
        self.robot_params = []
        params,_ = load_sinlge_robot_arm_params('Kinova3')
        self.robot_params.append(params)
    '''

    def _check_if_node_in_free_cspace(self, node):
        if not self._check_within_limits(node):
            return False
        pos = torch.tensor(node.pos, dtype=torch.float)
        return not self.env.zonopy.arm3d.collision_check(pos)

    def _check_within_limits(self, node):
        # Joint limits with same lower and upper bounds have no limits
        joints_with_limits = self.dof_limits[:, 0] != self.dof_limits[:, 1]
        return (
            (node.pos[joints_with_limits] >=
                self.dof_limits[joints_with_limits, 0]
             ).all() and
            (node.pos[joints_with_limits] <=
             self.dof_limits[joints_with_limits, 1]
             ).all()
        )

    def _pick_random_node(self):
        if random.random() < self.goal_bias:
            return self.goal
        while True:
            joints_with_limits = self.dof_limits[:, 0] != self.dof_limits[:, 1]
            new_node = Node([random.uniform(self.dof_limits[i, 0], self.dof_limits[i, 1]) if joints_with_limits[i] else (random.uniform(-np.pi, np.pi) if i != 0 else random.uniform(-np.pi/2, np.pi/2))
                            for i in range(7)], parent=None)
            if self._check_if_node_in_free_cspace(new_node):
                return new_node

    def _find_closest_node(self, node):
        # print "Finding closest node to ", node.pos
        min_distance = np.Inf
        min_node = self.start
        for curr in self.tree_list:
            d = node.euclidean_distance(curr)
            if d < min_distance:
                min_distance = d
                min_node = curr
        return min_node

    def _goal_test(self, node):
        return node == self.goal

    def expand_to_node(self, node):
        r = (node.pos-self.current.pos + np.pi) % (2*np.pi) - np.pi
        if np.linalg.norm(r) == 0:
            return self._goal_test(self.current)
        r_hat = r / np.linalg.norm(r)

        # Loop till node is moving farther from random node
        # prev_distance = np.Infinity
        # distance = np.Infinity
        # while distance <= prev_distance:
        while True:
            new_node = Node(self.current.pos + (r_hat * self.step_size), None)
            if not self._check_if_node_in_free_cspace(new_node):
                break
            # Update Tree
            new_node.parent = self.current
            self.tree_list.append(new_node)
            self.current = new_node

            '''
            ee_pos = self.env.sim.data.site_xpos[self.robot.eef_site_id].copy()
            DebugHelper.add_new_marker_to_env(
                self.env, pos=ee_pos, size=0.005, rgba=BLUE, persist=True)
            '''
            # self.env.render()
            # DebugHelper.update_after_render(self.env)

            r = (node.pos-self.current.pos + np.pi) % (2*np.pi) - np.pi
            # prev_distance = distance
            # distance = np.linalg.norm(r)

        return self._goal_test(self.current)

    def reset(self):
        print("Resetting")
        n = self._find_closest_node(self.goal)
        print(
            f"Distance to goal from closest: {n.euclidean_distance(self.goal)}")
        self.current = self.start
        self.solution = []
        self.tree_list = [self.current]

    def build_tree(self):
        count = 0
        iter = 0
        print("Building Tree")
        success = True
        while iter < 50:
            q_rand = self._pick_random_node()
            self.current = self._find_closest_node(q_rand)
            # if (q_rand.euclidean_distance(self.current) < self.step_size):
            #     print("skipping")
            #     continue

            if count == 100:
                print(len(self.tree_list))
                n = self._find_closest_node(self.goal)
                if n == self.goal:
                    print("Reached Goal")
                    self.goal.parent = n
                    break
                print(
                    f"Distance to goal from closest: {n.euclidean_distance(self.goal)}")
                print(f"Tree Size {len(self.tree_list)}")
                count = 0
                iter += 1

            if self.expand_to_node(q_rand):
                print("Reached Goal")
                self.goal.parent = self.current
                break
            count += 1

            if len(self.tree_list) > 3000:
                self.reset()
        
        if iter == 50:
            success = False
            print("failed to build tree")
        
        return success

    def _expand_between_smoothing_nodes(self, st, end):
        curr = st
        # np.array([(end.pos[i] - st.pos[i]) for i in range(7)])
        # r = (end.pos-st.pos + np.pi) % (2*np.pi) - np.pi
        r = end.pos - st.pos
        r_hat = r / np.linalg.norm(r)
        new_path = [curr]
        while True:
            new_node = Node(curr.pos + (r_hat * self.step_size), None)
            try:
                if self._check_if_node_in_free_cspace(new_node):
                    new_node.parent = curr
                    new_path.append(new_node)
                    curr = new_node
                    if new_node == end:
                        return new_path
                else:
                    break
            except Exception as e:
                print(e)
                break
        return []

    def shortcut_smoothing(self):
        for i in range(150):
            try:
                i1 = random.randint(0, len(self.solution)-1)
                i2 = random.randint(i1, len(self.solution)-1)
                if (i2 - i1) < 2:
                    i -= 1
                    continue
                n1 = self.solution[i1]
                n2 = self.solution[i2]

                new_path = self._expand_between_smoothing_nodes(n1, n2)
                if len(new_path) > 1:
                    sub_start = i1
                    sub_end = i2
                    if not (sub_end+1 == len(self.solution)):
                        self.solution[sub_end+1].parent = new_path[-1]
                        end = self.solution[sub_end+1]
                        self.solution[sub_start: sub_end] = new_path
                        self.solution[sub_start + len(new_path)] = end
            except IndexError:
                i -= 1
                continue

    def backtrace(self):
        self.solution = []
        curr = self.goal
        while curr is not self.start:
            self.solution.append(curr)
            curr = curr.parent
        self.solution.append(self.start)
        self.solution.reverse()

    def save(self, filename):
        solution_np = np.array([n.pos for n in self.solution])
        solution_np.tofile(filename)
        print(
            f"Saved solution to {filename} with {len(self.solution)} waypoints")

    def load(self, filename):
        solution_np = None
        solution_np = np.fromfile(filename).reshape((-1, 7))
        self.solution = [Node(pos) for pos in solution_np]
        print(
            f"Loaded solution from {filename} with {len(self.solution)} waypoints")
