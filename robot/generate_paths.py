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
    ],
    'side_right_around_back_to_opposite_side': [
        pf.Waypoint(0, 1.5, 0),
        pf.Waypoint(5.5, 2.2, math.radians(60)),
        pf.Waypoint(5.75, 5.4, math.radians(90))
    ],
    'side_right_reverse_to_same_side_cube': [
        pf.Waypoint(4.25, 2.20, math.radians(90)),
        pf.Waypoint(6.5, 1.25, math.radians(135))
    ],
    'side_right_approach_same_side_cube': [
        pf.Waypoint(6.5, 1.25, math.radians(135)),
        pf.Waypoint(5.5, 2, math.radians(135))
    ],
    'side_right_return_approach_same_side': [
        pf.Waypoint(5.5, 2, math.radians(135)),
        pf.Waypoint(6.5, 1.25, math.radians(135))
    ],
    'side_right_deposit_same_side_cube': [
        pf.Waypoint(6.5, 1.25, math.radians(135)),
        pf.Waypoint(4.25, 2.20, math.radians(90)),
    ]
}


# Mirror trajectories
for key, points in list(TRAJECTORIES.items()):
    new_key = key.replace('right', 'left')
    if new_key != key:
        TRAJECTORIES[new_key] = [
            points[0],
            pf.Waypoint(points[1].x, points[0].y +
                        (points[0].y - points[1].y),
                        -points[1].angle)
        ]

        # Add 3rd point
        if len(points) > 2:
            TRAJECTORIES[new_key].append(
                pf.Waypoint(points[2].x, TRAJECTORIES[new_key][1].y +
                            (points[1].y - points[2].y),
                            -points[2].angle)
            )

if __name__ == '__main__':

    paths = {}

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

        paths[key + '-l'] = left_traj
        paths[key + '-r'] = right_traj
        paths[key] = trajectory

        if len(sys.argv) > 2 and sys.argv[2] == '--dump-csvs':
            pf.serialize_csv(sys.argv[1] + '/' + key + '.csv', trajectory)
            pf.serialize_csv(sys.argv[1] + '/' + key + '-l.csv', left_traj)
            pf.serialize_csv(sys.argv[1] + '/' + key + '-r.csv', right_traj)

    pickle.dump(paths, open(sys.argv[1] + '/paths.pickle', 'wb'))
