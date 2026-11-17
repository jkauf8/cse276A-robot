"""ROS node for HW4: the HW2 localization node driving a planned path, with the HW3 spin floor."""
import math

import rclpy
from rclpy.signals import SignalHandlerOptions

from hw2_localization.controller import Controller
from robot_control import velocity_control as vc
from robot_control.localization_control import LocalizationNode


class PlanNode(LocalizationNode):
    def __init__(self):
        super().__init__()
        self.declare_parameter('min_turn_cmd', 0.38)  # smallest wheel command while spinning (lower stalls on carpet)

        def param(name):
            return self.get_parameter(name).value

        # the base class builds its Controller without min_turn_cmd, so rebuild it with the same arguments plus that
        self.ctrl = Controller(self.ctrl.waypoints, vc.wheel_cmds, vc.CRUISE_SPEED, vc.TURN_RATE, vc.PAUSE, vc.TURN_SIGN,
                               min_cmd=param('min_cmd'), deadband=param('deadband'),
                               turn_tol=math.radians(param('turn_tol_deg')), reaim_tol=math.radians(param('reaim_tol_deg')),
                               pos_tol=param('pos_tol'), final_checks=param('final_checks'),
                               min_turn_cmd=param('min_turn_cmd'))


def main(args=None):
    # run until the controller finishes; always stop the wheels at the end, even on Ctrl-C
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PlanNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
