#!/usr/bin/env python3

# Copyright 2020-2022 Josh Pieper, jjp@pobox.com.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

'''Set servo-level configuration for a quad A1 robot.'''

# python ~/b2_ws/src/experimental/scripts/n_lateral_motor_config.py -v

import argparse
import asyncio
import moteus
import os
import subprocess
import sys
import tempfile
import math

SCRIPT_PATH = os.path.dirname(__file__)

#TODO Read this information from one script files between ROS and this...

# python ~/blue_ws/src/bam_dev/scripts/motor_config.py -v

#IFLIGHT MOTOR INFO:
# https://shop.iflight-rc.com/gimbal-motors-cat44/ipower-motor-gm110-10-brushless-gimbal-motor-pro1297
#https://shop.iflight-rc.com/gimbal-motors-cat44/ipower-motor-gm8112-brushless-gimbal-motor-pro195
#https://shop.iflight-rc.com/gimbal-motors-cat44/ipower-motor-gm6208-150t-brushless-gimbal-motor-pro208

CONFIG_BAM_2 = {

    'motor_position.output.sign' : [
            ('11,12', '1'),
        ],

    'servo.max_power_W' : [
        ('11,12', '80'),
    ],
    'servo.max_current_A' : [ #from blue  motor_current_limits: [2.5, 2.5, 2.5, 2.0, 2.0, 1.0, 1.0, 1.0]
        ('11,12', '5'),
    ],
    'servo.derate_temperature' : [
        ('11,12', '60'),
    ],
    'servo.fault_temperature' : [
        ('11,12', '80'),
    ],
    'servo.pid_position.kp' : [
        ('11,12', '120'),
    ],
    'servo.pid_position.ki' : [
        ('11,12', '0'),
  
    ],
    'servo.pid_position.kd' : [
        ('11,12', '0'),
    ],
    # 'servo.default_timeout_s' : [
    #     ('11,12,13,14,15,16,17,18', math.nan),
    # ],
    'servo.default_velocity_limit' : [
        ('11,12', '10'),
    ],
    'servo.default_accel_limit' : [
        ('11,12', '10'),
    ],
    'servopos.position_min' : [
        ('11,12', '-20'),

    ],
    'servopos.position_max' : [
        ('11,12', '20'),

    ],
    'servo.voltage_mode_control' : [
        ('11,12', '1'),
    ],
}

async def config_servo(args, transport, servo_id):
    if args.verbose:
        print(f"*** SERVO {servo_id} ***")

    c = moteus.Controller(id=servo_id, transport=transport)
    s = moteus.Stream(c, verbose=args.verbose)

    await s.flush_read()

    # CONFIG = CONFIG_A1 if args.a1 else CONFIG_A2
    CONFIG = CONFIG_BAM_2

    for key, data_or_value in CONFIG.items():
        if type(data_or_value) == str:
            value = data_or_value
            await s.command(
                "conf set {} {}".format(key, value).encode('utf8'))
        else:
            for servo_selector, value in data_or_value:
                ids = set([int(x) for x in servo_selector.split(',')])
                if not servo_id in ids:
                    continue
                await s.command(
                    "conf set {} {}".format(key, value).encode('utf8'))

    await s.command(b'conf write')


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', action='store_true')
    # parser.add_argument('--version', action='store_true')

    args = parser.parse_args()

    # if os.geteuid() != 0:
    #     raise RuntimeError('This must be run as root')

    transport = moteus.Fdcanusb()
    #TODO What happens if two FDCAN USB CONNECTED?

    for i in range(1, 3):
        i = i + 10 # ID's for right are 11, 12, 13, etc.
        print(i)
        await config_servo(args, transport, i)

if __name__ == '__main__':
    asyncio.run(main())