#!/usr/bin/env python

from chessbot_controller import *


def main():
  try:
    cbc = ChessbotController()

    import ipdb
    ipdb.set_trace()

    raw_input("============ Press ENTER to state demo project")

    ############# go to CP 1
    ps0 = cbc.pose_from_point(POINT_0)
    ps1 = cbc.pose_from_point(POINT_1)
    ps2 = cbc.pose_from_point(POINT_2)

    waypoints = []
    waypoints.append(ps0)
    waypoints.append(ps1)
    waypoints.append(ps2)
    plan, fraction = cbc.plan_cartesian_path(waypoints)
    cbc.execute_plan(plan)

    # import ipdb 
    # ipdb.set_trace()

    cbc.grip()

    ############# go to CP 2
    cbc.go_to_point_goal(POINT_1)
    cbc.go_to_point_goal(POINT_3)
    cbc.go_to_point_goal(POINT_4)

    cbc.release()

    ############# go to Initial State
    cbc.go_to_point_goal(POINT_3)
    cbc.go_to_point_goal(POINT_0)
    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == "__main__":
    main()