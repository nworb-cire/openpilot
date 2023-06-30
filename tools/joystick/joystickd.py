#!/usr/bin/env python
import os
import argparse
import threading
from dataclasses import dataclass

from inputs import get_gamepad

import cereal.messaging as messaging
from common.realtime import Ratekeeper
from common.numpy_fast import interp, clip
from common.params import Params
from tools.lib.kbhit import KBHit


class Keyboard:
  def __init__(self):
    self.kb = KBHit()
    self.axis_increment = 0.05  # 5% of full actuation each key press
    self.axes_map = {'w': 'gb', 's': 'gb',
                     'a': 'steer', 'd': 'steer'}
    self.axes_values = {'gb': 0., 'steer': 0.}
    self.axes_order = ['gb', 'steer']
    self.cancel = False

  def update(self):
    key = self.kb.getch().lower()
    self.cancel = False
    if key == 'r':
      self.axes_values = {ax: 0. for ax in self.axes_values}
    elif key == 'c':
      self.cancel = True
    elif key in self.axes_map:
      axis = self.axes_map[key]
      incr = self.axis_increment if key in ['w', 'a'] else -self.axis_increment
      self.axes_values[axis] = clip(self.axes_values[axis] + incr, -1, 1)
    else:
      return False
    return True


@dataclass
class JoystickConfig:
  accel_axis: str
  steer_axis: str
  cancel_button: str


configs = {
  'joystick': JoystickConfig(accel_axis='ABS_Y', steer_axis='ABS_RZ', cancel_button='BTN_TRIGGER'),
  'gamepad': JoystickConfig(accel_axis='ABS_Y', steer_axis='ABS_RX', cancel_button='BTN_NORTH'),
  'switch': JoystickConfig(accel_axis='ABS_Y', steer_axis='ABS_X', cancel_button='BTN_SOUTH'),
}

class Joystick:
  def __init__(self, config: JoystickConfig):
    # TODO: find a way to get this from API, perhaps "inputs" doesn't support it
    self.config = config
    self.min_axis_value = {config.accel_axis: 0., config.steer_axis: 0.}
    self.max_axis_value = {config.accel_axis: 255., config.steer_axis: 255.}
    self.axes_values = {config.accel_axis: 0., config.steer_axis: 0.}
    self.axes_order = [config.accel_axis, config.steer_axis]
    self.cancel = False

  def update(self):
    joystick_event = get_gamepad()[0]
    event = (joystick_event.code, joystick_event.state)
    if event[0] == self.config.cancel_button:
      if event[1] == 1:
        self.cancel = True
      elif event[1] == 0:   # state 0 is falling edge
        self.cancel = False
    elif event[0] in self.axes_values:
      self.max_axis_value[event[0]] = max(event[1], self.max_axis_value[event[0]])
      self.min_axis_value[event[0]] = min(event[1], self.min_axis_value[event[0]])

      norm = -interp(event[1], [self.min_axis_value[event[0]], self.max_axis_value[event[0]]], [-1., 1.])
      self.axes_values[event[0]] = norm if abs(norm) > 0.05 else 0.  # center can be noisy, deadzone of 5%
    else:
      return False
    return True


def send_thread(joystick):
  joystick_sock = messaging.pub_sock('testJoystick')
  rk = Ratekeeper(100, print_delay_threshold=None)
  while 1:
    dat = messaging.new_message('testJoystick')
    dat.testJoystick.axes = [joystick.axes_values[a] for a in joystick.axes_order]
    dat.testJoystick.buttons = [joystick.cancel]
    joystick_sock.send(dat.to_bytes())
    print('\n' + ', '.join(f'{name}: {round(v, 3)}' for name, v in joystick.axes_values.items()))
    if "WEB" in os.environ:
      import requests
      requests.get("http://"+os.environ["WEB"]+":5000/control/%f/%f" % tuple([joystick.axes_values[a] for a in joystick.axes_order][::-1]), timeout=None)
    rk.keep_time()

def joystick_thread(joystick):
  Params().put_bool('JoystickDebugMode', True)
  threading.Thread(target=send_thread, args=(joystick,), daemon=True).start()
  while True:
    joystick.update()

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Publishes events from your joystick to control your car.\n' +
                                               'openpilot must be offroad before starting joysticked.',
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('--keyboard', action='store_true', help='Use your keyboard instead of a joystick')
  parser.add_argument('--gamepad', type=str, choices=['joystick', 'gamepad', 'switch'], default='joystick',
                      help='Type of gamepad to use. Default is joystick.')
  args = parser.parse_args()

  if not Params().get_bool("IsOffroad") and "ZMQ" not in os.environ and "WEB" not in os.environ:
    print("The car must be off before running joystickd.")
    exit()

  print()
  if args.keyboard:
    print('Gas/brake control: `W` and `S` keys')
    print('Steering control: `A` and `D` keys')
    print('Buttons')
    print('- `R`: Resets axes')
    print('- `C`: Cancel cruise control')
  else:
    print('Using joystick, make sure to run cereal/messaging/bridge on your device if running over the network!')

  joystick = Keyboard() if args.keyboard else Joystick(configs[args.gamepad])
  joystick_thread(joystick)
