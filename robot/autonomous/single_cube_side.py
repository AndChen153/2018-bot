from autonomous.two_cube_side import TwoCubeSide
from components.field import SwitchState


class SingleCubeLeft(TwoCubeSide):

    MODE_NAME = 'Single Cube From Left'
    ONE_CUBE_ONLY = True
    start_side = SwitchState.LEFT


class SingleCubeRight(TwoCubeSide):

    MODE_NAME = 'Single Cube From Right'
    ONE_CUBE_ONLY = True
    start_side = SwitchState.RIGHT
