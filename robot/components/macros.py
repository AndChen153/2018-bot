from components import drivetrain, elevator, grabber


class Macros:

    drivetrain = drivetrain.Drivetrain
    elevator = elevator.Elevator
    grabber = grabber.Grabber

    def setup(self):
        self.components = {
            'drivetrain': self.drivetrain,
            'elevator': self.elevator,
            'grabber': self.grabber
        }

    @property
    def component_names(self):
        return list(self.components.keys())

    def fetch_component_states(self):
        states = {}
        for key, component in self.components.items():
            states[key] = component.get_state()
        return states

    def put_component_states(self, states):
        for key, state in states.items():
            self.components[key].put_state(state)

    def execute(self):
        pass
