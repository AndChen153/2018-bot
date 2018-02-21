#!/usr/bin/env python3

import math
import sys
import pickle

import pathfinder as pf

WHEELBASE_WIDTH = 0.7619995885

TRAJECTORIES = {
    'center_right': [
        pf.Waypoint(0, 0, 0),
        pf.Waypoint(2, 1.95, 0),
    ],
    'center_left': [
        pf.Waypoint(0, 0, 0),
        pf.Waypoint(2, -1.95, 0),
    ],
    # 'center_left_backup': [
    #     pf.Waypoint(2, -1.95, 0),
    #     pf.Waypoint(0, 0, 0)
    # ],
    # 'center_left_to_cube': [
    #     pf.Waypoint(0, 0, 0),
    #     pf.Waypoint(1, 1, math.radians(45))
    # ],
    # 'center_left_return_from_second': [
    #     pf.Waypoint(1.5, 0, math.radians(45)),
    #     pf.Waypoint(1, -1, math.radians(45)),
    #     pf.Waypoint(2, -1.95, 0)
    # ]
}

if __name__ == '__main__':

    for key, points in TRAJECTORIES.items():

        info, trajectory = pf.generate(points, pf.FIT_HERMITE_CUBIC,
                                       pf.SAMPLES_HIGH, 0.05, 3.66, 0.5, 60.0)

        modifier = pf.modifiers.TankModifier(trajectory).modify(WHEELBASE_WIDTH)

        print(info)

        pickle.dump(modifier.getLeftTrajectory(), open(sys.argv[1] + '/' + key + '-l', 'wb'))
        pickle.dump(modifier.getRightTrajectory(), open(sys.argv[1] + '/' + key + '-r', 'wb'))
        pf.serialize_csv(sys.argv[1] + '/' + key + '.csv', trajectory)
