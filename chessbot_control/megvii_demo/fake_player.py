#!/usr/bin/env python

import numpy as np
import copy
import re

from chessbot_controller import *
import utils


class FakePlayer:

    def __init__(self, resample=False):
        self.controller = ChessbotController()
        self.yaml = "config.yaml"
        self.tip_qua = {"h": [0.707,  0,  0.707,  0],
                        "i": [0.669, 0.230, 0.669, -0.230],
                        "v": [0.533, 0.465, 0.532, -0.465]}
        self.map = np.zeros((9, 10), dtype=np.uint8)
        self.pos = [[1, 2], [1, 4], [2, 3], [2, 4], [2, 5], [3, 4], [8, 9]]

        # populate map
        for p in self.pos:
            self.map[p[0], p[1]] = 1

        if resample:
            print("============ Ready to sample corners")
            self.sample_corners()
        else:
            self.corners = utils.read_yaml(self.yaml)
            print("============ Corners loaded")

        # go to initial state
        raw_input("============ Press ENTER to go to the initial state")
        self.reset()
        self.controller.release()

    def sample_corners(self):
        self.corners = []
        for i in range(4):
            raw_input(
                "============ Move to no.{} corner and press ENTER.".format(i+1))
            pose = self.controller.pose_state
            print(pose)
            self.corners.append(pose)

        # save sampled corners to yaml
        utils.save_yaml(self.yaml, self.corners)
        print("Sampling completed.")

    def interp_grid(self, x, y):
        corner_x1 = (1-x/8.) * \
            np.array(self.corners[0]) + x/8. * np.array(self.corners[1])
        corner_x2 = (1-x/8.) * \
            np.array(self.corners[2]) + x/8. * np.array(self.corners[3])
        corner = (1-y/9.) * corner_x1 + y/9. * corner_x2
        corner = corner.tolist()

        # get tip angle
        if x - 1 < 0 and not self.map[x+1, y]:
            k = "h"
        elif x + 1 > 8 and not self.map[x-1, y]:
            k = "h"
        elif not self.map[x-1, y] and not self.map[x+1, y]:
            # horizontal
            k = "h"
        elif y - 1 < 0 and not self.map[x, y+1]:
            k = "v"
        elif y + 1 > 9 and not self.map[x, y-1]:
            k = "v"
        elif not self.map[x, y-1] and not self.map[x, y+1]:
            # vertical
            k = "v"
        else:
            # istatic
            k = "i"
        corner.extend(self.tip_qua[k])
        return corner

    def goto_xy(self, x, y):
        pose_new = self.interp_grid(x, y)
        pose_new_over = self.get_pose_over(pose_new)

        if not self.is_idle():
            pose_old = self.controller.pose_state
            pose_old_over = self.get_pose_over(pose_old)
            self.controller.goto_pose_goal(pose_old_over)

        self.controller.goto_pose_goal(pose_new_over)
        self.controller.goto_pose_goal(pose_new)

    def rotate_wrist(self, rad):
        js = self.controller.joint_state
        js[-1] = rad
        import ipdb
        ipdb.set_trace()
        self.controller.goto_joint_goal(js)

    def get_pose_over(self, pose):
        pose_over = copy.copy(pose)
        pose_over[2] += 0.04
        return pose_over

    def reset(self):
        if not self.is_idle():
            pose_old = self.controller.pose_state
            pose_old_over = self.get_pose_over(pose_old)
            self.controller.goto_pose_goal(pose_old_over)
            self.controller.goto_joint_goal(INITIAL_JOINT)

    def is_idle(self):
        diff = np.array(self.controller.pose_state) - np.array(INITIAL_POSE)
        return abs(diff.sum()) < 0.05

    def parse_code(self, code):
        # code pattern: 2,3-4,6
        assert re.match("[0-8],[0-9]-[[0-8],[0-9]$", code) is not None
        old_pos, new_pos = code.split('-')
        old_pos = map(int, old_pos.split(','))
        new_pos = map(int, new_pos.split(','))
        self.move(old_pos[0], old_pos[1], new_pos[0], new_pos[1])

    def move(self, x1, y1, x2, y2):
        assert 0 <= x1 <= 8 and 0 <= y1 <= 9 and 0 <= x2 <= 8 and 0 <= y2 <= 9
        if not self.map[x1, y1]:
            print("[WARN]\tNo piece found in ({},{})".format(x1, y1))
            return

        if self.map[x2, y2]:
            # remove (x2, y2)
            self.remove(x2, y2)

        self.goto_xy(x1, y1)
        self.controller.grip()
        self.goto_xy(x2, y2)
        self.controller.release()
        self.reset()

        self.map[x1, y1] = 0
        self.map[x2, y2] = 1

    def remove(self, x, y):
        self.goto_xy(x, y)
        self.controller.grip()
        self.reset()
        self.controller.release()
        
        self.map[x, y] = 0


def main():
    try:
        player = FakePlayer()

        import ipdb
        ipdb.set_trace()

        running = True
        while running:
            instr = raw_input("Enter instruction: ")
            if instr == 'q':
                return
            
            player.parse_code(instr)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
