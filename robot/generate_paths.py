#!/usr/bin/env python3

import math
import sys
import pickle

import pathfinder as pf

WHEELBASE_WIDTH = 0.7619995885  # meters

# ORIGINAL
# DT = 0.02
# MAX_VELOCITY = 3.66
# MAX_ACCEL = 3
# MAX_JERK = 60.0

# V2 - FRI 3/10
# DT = 0.02
# MAX_VELOCITY = 3.66
# MAX_ACCEL = 9.5
# MAX_JERK = 100.0

# DT = 0.02
# MAX_VELOCITY = 3.66
# MAX_ACCEL = 9.5
# MAX_JERK = 50

# DT = 0.02
# MAX_VELOCITY = 3.66
# MAX_ACCEL = 7
# MAX_JERK = 50

DT = 0.02
MAX_VELOCITY = 3.66
MAX_ACCEL = 4.5
MAX_JERK = 60.0

TRAJECTORIES = {
    'side_forward': [
        pf.Waypoint(0, 0, 0),
        pf.Waypoint(3.75, 0, 0)
    ],
    'side_return_right': [
        pf.Waypoint(3.75, 0, 0),
        pf.Waypoint(1.75, -1.75, 0)
    ],

    # THESE ARE NEW
    'center_right': [
        pf.Waypoint(0.89, 4.11, 0),
        pf.Waypoint(3.5, 3.25, 0)  # + 0.86
    ],
    'center_left': [
        pf.Waypoint(0.89, 4.11, 0),
        pf.Waypoint(3.5, 5.25, 0)  # + 1.14
    ],

    # OG FALLBACK, NO CENTER LEFT SPECIFIC
    # 'center_right': [
    #     pf.Waypoint(0.89, 4.11, 0),
    #     pf.Waypoint(4, 2.75, 0)
    # ],

    'center_right_reverse': [
        pf.Waypoint(3.5, 2.72, 0),
        pf.Waypoint(1.75, 3.47, 0)
    ],
    'center_right_return': [
        pf.Waypoint(1.75, 3.47, 0),
        pf.Waypoint(3.6, 3.2, 0)
    ],
    'center_approach_second_cube': [
        pf.Waypoint(0, 0, 0),
        pf.Waypoint(1.75, 0, 0)
    ],
    'center_return_second_cube': [
        pf.Waypoint(1.25, 0, 0),
        pf.Waypoint(0, 0, 0)
    ],
    'side_right_around_back_to_opposite_side': [
        pf.Waypoint(0, 1.5, 0),
        pf.Waypoint(5.5, 2.2, math.radians(60)),
        pf.Waypoint(5.75, 8.5, math.radians(90))
    ],
    'side_right_reverse_to_same_side_cube': [
        pf.Waypoint(4.25, 2.2, math.radians(90)),
        pf.Waypoint(7, 1.75, math.radians(135))

        # pf.Waypoint(4.25, 2.2, math.radians(90)),
        # pf.Waypoint(5, 1.25, math.radians(150)),
        # pf.Waypoint(6.5, 1.25, math.radians(135))

        # pf.Waypoint(4.25, 2.20, math.radians(90)),
        # pf.Waypoint(6.5, 1.25, math.radians(135))
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
    ],

    'side_right_to_back': [
        pf.Waypoint(0, 1.25, math.radians(0)),
        pf.Waypoint(3.75, 0.75, math.radians(0)),
        pf.Waypoint(5.75, 2.25, math.radians(90))
    ],
    'side_right_back_to_second_cube': [
        pf.Waypoint(5.25, 2.65, math.radians(180)),
        pf.Waypoint(6, 3.7, math.radians(225))
    ]
}

TRAJECTORY_OPTIONS = {
    # 'center_right': {
    #     'max_accel': 4,
    #     'max_jerk': 30,
    #     'max_velocity': 4
    # }
}


# Mirror trajectories
for key, points in list(TRAJECTORIES.items()):
    new_key = key.replace('right', 'left')
    if new_key != key and new_key not in TRAJECTORIES:
        print('Autogenerating %s' % new_key)
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

        # Copy options
        if key in TRAJECTORY_OPTIONS:
            TRAJECTORY_OPTIONS[new_key] = TRAJECTORY_OPTIONS[key]

if __name__ == '__main__':

    paths = {}

    for key, points in TRAJECTORIES.items():

        if key in TRAJECTORY_OPTIONS:
            opts = TRAJECTORY_OPTIONS[key]
        else:
            opts = {}

        max_velocity = opts.get('max_velocity', MAX_VELOCITY)
        max_accel = opts.get('max_accel', MAX_ACCEL)
        max_jerk = opts.get('max_jerk', MAX_JERK)

        info, trajectory = pf.generate(points, pf.FIT_HERMITE_CUBIC,
                                       pf.SAMPLES_HIGH, DT, max_velocity,
                                       max_accel, max_jerk)

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
