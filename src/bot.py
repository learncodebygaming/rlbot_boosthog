import math

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

from util.orientation import Orientation
from util.vec import Vec3


class MyBot(BaseAgent):
    BOOSTS_COLLECTED_THRESHOLD = 6
    BALL_CHASE_DURATION = 5

    def initialize_agent(self):
        # This runs once before the bot starts up
        self.controller_state = SimpleControllerState()
        self.action_display = None
        self.has_first_touch_happened_yet = False
        self.boosts_collected = 0
        self.boost_counted = False
        self.chase_ball_game_time = 99999

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        info = self.get_field_info()
        ball_location = Vec3(packet.game_ball.physics.location)
        my_car = packet.game_cars[self.index]
        car_location = Vec3(my_car.physics.location)

        # if a goal occurred or overtime, reset the kickoff flag
        if packet.game_info.is_kickoff_pause:
            self.has_first_touch_happened_yet = False

        # update how many boosts our bot has collected
        self.update_boosts_collected(my_car, packet)
        
        # behavior logic is here. it decides what our target should be
        behavior_mode = "Unknown"
        if not self.has_first_touch_happened_yet:
            behavior_mode = "Kickoff"
            # during kickoff, go for the ball
            target_location = ball_location

            # check to see if kickoff has occurred by seeing if the ball position
            # has left the middle of the field
            if ball_location.flat() != Vec3(0, 0, 0):
                self.has_first_touch_happened_yet = True
        elif packet.game_info.game_time_remaining > self.chase_ball_game_time - self.BALL_CHASE_DURATION:
            behavior_mode = "Chase Ball"
            # go chase the ball
            target_location = ball_location
        else:
            behavior_mode = "Boost Steal"
            # default behavior is to grab the nearest boost
            target_location = get_nearest_boost(info, packet, car_location)

        # set the controller inputs based on the target we picked
        self.controller_state = self.action_goto(my_car, car_location, target_location)

        # additional debug text to draw in the corner
        #print(packet.game_info.game_time_remaining)
        corner_debug = "Time Remaining: {}\n".format(packet.game_info.game_time_remaining)
        corner_debug += "Boosts Collected: {}\n".format(self.boosts_collected)
        corner_debug += "Second Boost Location: {}\n".format(info.boost_pads[1].location)
        corner_debug += "Nearest Boost Location: {}\n".format(target_location)
        corner_debug += "Current Mode: {}\n".format(behavior_mode)

        draw_debug(self.renderer, my_car, target_location, self.action_display, corner_debug)

        return self.controller_state

    def action_goto(self, my_car, car_location, target_location):
        controller = SimpleControllerState()

        if target_location is None:
            return controller
        car_to_target = target_location - car_location

        # Find the direction of our car using the Orientation class
        car_orientation = Orientation(my_car.physics.rotation)
        car_direction = car_orientation.forward

        steer_correction_radians = find_correction(car_direction, car_to_target)

        if steer_correction_radians > 0:
            # Positive radians in the unit circle is a turn to the left.
            turn = -1.0  # Negative value for a turn to the left.
            self.action_display = "turn left"
        else:
            turn = 1.0
            self.action_display = "turn right"

        controller.throttle = 1.0
        controller.steer = turn
        controller.boost = True

        return controller

    def update_boosts_collected(self, my_car, packet):
        # mark when 100 boost has been reached
        if my_car.boost >= 100:
            if not self.boost_counted:
                self.boosts_collected += 1
            self.boost_counted = True
        else:
            self.boost_counted = False

        # if boosts collected has crossed threshold, trigger our behavior logic to chase ball
        if self.boosts_collected >= self.BOOSTS_COLLECTED_THRESHOLD:
            self.chase_ball_game_time = packet.game_info.game_time_remaining
            # reset the boost collected count
            self.boosts_collected = 0


def find_correction(current: Vec3, ideal: Vec3) -> float:
    # Finds the angle from current to ideal vector in the xy-plane. Angle will be between -pi and +pi.

    # The in-game axes are left handed, so use -x
    current_in_radians = math.atan2(current.y, -current.x)
    ideal_in_radians = math.atan2(ideal.y, -ideal.x)

    diff = ideal_in_radians - current_in_radians

    # Make sure that diff is between -pi and +pi.
    if abs(diff) > math.pi:
        if diff < 0:
            diff += 2 * math.pi
        else:
            diff -= 2 * math.pi

    return diff


def draw_debug(renderer, car, target, action_display, corner_debug=None):
    renderer.begin_rendering()
    # draw a line from the car to the target
    renderer.draw_line_3d(car.physics.location, target, renderer.white())
    # print the action that the bot is taking
    renderer.draw_string_3d(car.physics.location, 2, 2, action_display, renderer.white())
    # print the corner debug string
    # adjust y position depending on how many lines of text there are
    if corner_debug:
        corner_display_y = 900 - (corner_debug.count('\n') * 20)
        renderer.draw_string_2d(10, corner_display_y, 1, 1, corner_debug, renderer.white())
    renderer.end_rendering()


def get_nearest_boost(info, packet, car_location):
    nearest_boost_loc = None

    # loop over all the boosts
    for i, boost in enumerate(info.boost_pads):
        # only want large boosts that haven't been taken
        if boost.is_full_boost and packet.game_boosts[i].is_active:
            # if we haven't found any boosts yet, use this one
            if not nearest_boost_loc:
                nearest_boost_loc = boost.location
            else:
                # if this boost is closer, save that
                if car_location.dist(Vec3(boost.location)) < car_location.dist(Vec3(nearest_boost_loc)):
                    nearest_boost_loc = boost.location

    # if no large boosts are found, find the nearest small boost
    # CODE SMELL: very similar duplicate code, looping over boost list twice
    if nearest_boost_loc is None:
        for i, boost in enumerate(info.boost_pads):
            # only want large boosts that haven't been taken
            if packet.game_boosts[i].is_active:
                # if we haven't found any boosts yet, use this one
                if not nearest_boost_loc:
                    nearest_boost_loc = boost.location
                else:
                    # if this boost is closer, save that
                    if car_location.dist(Vec3(boost.location)) < car_location.dist(Vec3(nearest_boost_loc)):
                        nearest_boost_loc = boost.location

    # if no boosts are available, target the center of the field
    if nearest_boost_loc is None:
        nearest_boost_loc = Vec3(0, 0, 0)

    # a different possible optimization we could make would be to look at the 
    # packet.game_boosts[i].timer to find boosts that will respawn before our car arrives there

    return Vec3(nearest_boost_loc)

