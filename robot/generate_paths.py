#!/usr/bin/env python3

import math
import sys
import pickle

import pathfinder as pf

WHEELBASE_WIDTH = 0.7619995885  # meters

# TRAJECTORIES = {
#     'center_right': [
#         pf.Waypoint(0, 0, 0),
#         pf.Waypoint(2, 1.95, 0),
#     ],
#     'center_left': [
#         pf.Waypoint(0, 0, 0),
#         pf.Waypoint(2, -1.95, 0),
#     ]
# ]
#
# TRAJECTORIES = {
#     'center_left': [
#         pf.Waypoint(0.89, 4.11, 0),
#         pf.Waypoint(3.5, 5.5, 0),
#     ],
#     'center_left_reverse': [
#         pf.Waypoint(3.5, 5.5, 0),
#         pf.Waypoint(2.25, 4.75, 0)
#     ],
#     'center_left_return': [
#         pf.Waypoint(2.25, 4.75, 0),
#         pf.Waypoint(3.5, 5.5, 0)
#     ],

#     'center_right': [
#         pf.Waypoint(0.89, 4.11, 0),
#         pf.Waypoint(3.5, 2.72, 0),
#     ],
#     'center_right_reverse': [
#         pf.Waypoint(3.5, 2.72, 0),
#         pf.Waypoint(2.25, 3.47, 0)
#     ],
#     'center_right_return': [
#         pf.Waypoint(2.25, 3.47, 0),
#         pf.Waypoint(3.5, 2.72, 0)
#     ]
# }


TRAJECTORIES = {
    'side_forward': [
        pf.Waypoint(0, 0, 0),
        pf.Waypoint(3.75, 0, 0)
    ],
    'side_return_right': [
        pf.Waypoint(3.75, 0, 0),
        pf.Waypoint(1.75, -1.75, 0)
    ],
    'center_right': [
        pf.Waypoint(0.89, 4.11, 0),
        pf.Waypoint(3.5, 2.72, 0),
    ],
    'center_right_reverse': [
        pf.Waypoint(3.5, 2.72, 0),
        pf.Waypoint(2.25, 3.47, 0)
    ],
    'center_right_return': [
        pf.Waypoint(2.25, 3.47, 0),
        pf.Waypoint(3.75, 3.25, 0)
    ]
}


# Mirror trajectories
for key, points in list(TRAJECTORIES.items()):
    new_key = key.replace('right', 'left')
    if new_key != key:
        TRAJECTORIES[new_key] = [
            points[0],
            pf.Waypoint(points[1].x, points[0].y +
                        (points[0].y - points[1].y), points[1].angle)
        ]

if __name__ == '__main__':

    for key, points in TRAJECTORIES.items():

        # info, trajectory = pf.generate(points, pf.FIT_HERMITE_CUBIC,
        #                               pf.SAMPLES_HIGH, 0.05, 3.66, 1.5, 60.0)

        info, trajectory = pf.generate(points, pf.FIT_HERMITE_CUBIC,
                                       pf.SAMPLES_HIGH, 0.02, 2.5, 3, 60.0)

        modifier = pf.modifiers.TankModifier(
            trajectory).modify(WHEELBASE_WIDTH)

        print(key, info)

        left_traj = modifier.getLeftTrajectory()
        right_traj = modifier.getRightTrajectory()

        pickle.dump(left_traj, open(
            sys.argv[1] + '/' + key + '-l.pickle', 'wb'))
        pickle.dump(right_traj, open(
            sys.argv[1] + '/' + key + '-r.pickle', 'wb'))
        pickle.dump(trajectory, open(
            sys.argv[1] + '/' + key + '.pickle', 'wb'))
        pf.serialize_csv(sys.argv[1] + '/' + key + '.csv', trajectory)
        pf.serialize_csv(sys.argv[1] + '/' + key + '-l.csv', left_traj)
        pf.serialize_csv(sys.argv[1] + '/' + key + '-r.csv', right_traj)
