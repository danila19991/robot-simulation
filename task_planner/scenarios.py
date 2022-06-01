from task_planner.fsm_state import ChooseMultiState, BaseFsmState
from task_planner.composite_task import *
from robots.two_wheel.fsm_states import TwoWheelState
from robots.manipuator.fsm_elements import ManipulatorState
from task_planner.fsm_state import *
from robots.two_wheel.simulation import TwoWheelRobot
from robots.manipuator.simulation import ManipulatorRobot


class ExampleScenario:

    def __init__(self, cart_lengths, manip_lengths, ctx):
        self.cart_lengths = cart_lengths
        self.manip_lengths = manip_lengths
        self.cart_starts = list()
        self.cart_robs = list()
        self.cart_end = EndState()
        self.cart_end.name += '-CartEnd'
        for i, _ in enumerate(cart_lengths):
            s = StartState()
            r = TwoWheelRobot(ctx)
            s.name += f'-Cart{i}'
            r.state = s
            self.cart_starts.append(s)
            self.cart_robs.append(r)

        self.manip_starts = list()
        self.manip_robs = list()
        self.manip_end = EndState()
        self.manip_end.name += '-ManipEnd'
        for i, _ in enumerate(manip_lengths):
            s = StartState()
            r = ManipulatorRobot(ctx)
            s.name += f'-Manip{i}'
            r.state = s
            self.manip_starts.append(s)
            self.manip_robs.append(r)


        self.cart_mid_state = TwoWheelState(0, 0, 0, 0) # todo correct
        self.cart_mid_state.name += '-CartToManip'
        self.choose_manip = ChooseMultiState([self.cart_mid_state])
        self.choose_manip.name += '-CartToManip-Makro'

        for i, cart_start in enumerate(self.cart_starts):
            move_1_1 = CartMove(cart_start, self.cart_mid_state, self.choose_manip)
            cart_start.default = move_1_1.first_action()
            cart_start.composite = move_1_1
            move_1_1.t1[0] = cart_lengths[i]

        self.cart_before_manip_states = list()
        self.choose_cube = list()
        for i, manip_length in enumerate(manip_lengths):
            s2 = TwoWheelState(0, 0, 0, 0) # todo correct
            s2.name += f'-CartNearManip{i}'
            l2 = ChooseMultiState([s2, self.manip_starts[i]])
            l2.name += f'-Cube{i}-Makro'
            self.cart_before_manip_states.append(s2)
            self.choose_cube.append(l2)

            move_2_1 = CartMove(self.cart_mid_state, s2, l2)
            self.choose_manip.states[0].next[i] = move_2_1.first_action()
            self.choose_manip.next[i] = move_2_1
            move_2_1.t1[0] = manip_lengths[i]

            move_3_1 = CubeTransition1(s2, self.cart_end, self.manip_starts[i], self.manip_end, None, i)
            l2.states[0].next[0] = move_3_1.first_action_c()
            l2.states[1].next[0] = move_3_1.first_action_m()
            l2.next[0] = move_3_1
            move_3_2 = CubeTransition2(s2, self.cart_end, self.manip_starts[i], self.manip_end, None, i)
            l2.states[0].next[1] = move_3_2.first_action_c()
            l2.states[1].next[1] = move_3_2.first_action_m()
            l2.next[1] = move_3_2


