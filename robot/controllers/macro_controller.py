import csv
import json
import datetime
from enum import IntEnum
from os import path

from magicbot import StateMachine, state
from wpilib import Timer

from components import macros


class MacroMode(IntEnum):
    DISABLED = 0
    PLAYING = 1
    RECORDING = 2


class MacroController(StateMachine):

    macros = macros.Macros

    def setup(self):
        self.mode = MacroMode.DISABLED
        self.f = None
        self.current_macro = None
        self.prev_step = None

    def is_active(self):
        return self.is_executing

    def play(self, macro_name=None):
        self.mode = MacroMode.PLAYING
        if macro_name:
            self.current_macro = macro_name
        self.engage()

    def record(self, macro_name=None):
        self.mode = MacroMode.RECORDING
        if not self.current_macro:
            self.current_macro = macro_name or \
                datetime.datetime.now().isoformat()
        # print('recording', self.current_macro)
        self.engage()

    @state(first=True)
    def begin(self):
        self.start_time = Timer.getFPGATimestamp()
        if self.mode == MacroMode.PLAYING:
            self._load_current_macro()
            self.next_state('_play_macro')
        elif self.mode == MacroMode.RECORDING:
            self._load_macro_recorder()
            self.next_state('_record_macro')

    def _get_path(self):
        basepath = path.dirname(__file__)
        return path.abspath(path.join(basepath, '../macros',
                                      self.current_macro + '.txt'))

    def _load_current_macro(self):
        self.f = open(self._get_path(), 'r')
        reader = csv.DictReader(self.f)
        macro_steps = []
        for line in reader:
            time = float(line.pop('time'))
            state = {k: json.loads(v) for k, v in line.items()}
            macro_steps.append([time, state])
        self.current_macro_steps = macro_steps

    def _load_macro_recorder(self):
        self.f = open(self._get_path(), 'w')
        self.writer = csv.DictWriter(
            self.f, fieldnames=['time'] + self.macros.component_names)
        self.writer.writeheader()

    @state()
    def _play_macro(self):
        if self.current_macro_steps:
            time, step = self.current_macro_steps[0]
            if Timer.getFPGATimestamp() - self.start_time >= time:
                self.macros.put_component_states(step)
                self.current_macro_steps.pop(0)
                self.prev_step = step
            elif self.prev_step:
                self.macros.put_component_states(self.prev_step)
        else:
            self.done()

    @state
    def _record_macro(self):
        time = Timer.getFPGATimestamp() - self.start_time
        states = self.macros.fetch_component_states()
        row = {k: json.dumps(v) for k, v in states.items()}
        row['time'] = time
        self.writer.writerow(row)

    def done(self):
        self.current_macro = None
        if self.f:
            self.f.close()
            self.f = None

        super().done()
