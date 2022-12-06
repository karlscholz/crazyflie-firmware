#!/usr/bin/env python3
#
# ,---------,       ____  _ __
# |  ,-^-,  |      / __ )(_) /_______________ _____  ___
# | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
# | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
# Copyright (C) 2020 Bitcraze AB
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, in version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
"""
Send powerdown/up/reboot commands remotely using a Crazyradio
"""
# 20221206 karlscholz, if you are in WSL, use python.exe instad of python3 to just run it in windows (faster, usb stuff, etc)
import sys
#import subprocess
from cflib.utils.power_switch import PowerSwitch
from time import sleep
firstID = int(input('Enter the last two Hex Chars of your first ID: '), base=16)

firstAdress = "radio://0/80/2M/E7E7E7E7"+str(format(firstID, 'x')).upper()
adressList = [firstAdress]


lastID = int(input('Enter the last two Hex Chars of your last ID: '), base=16)

adressCount = lastID-firstID
if adressCount >= 0 and adressCount < 256:
    for i in range(firstID, lastID):
        adressList.append("radio://0/80/2M/E7E7E7E7"+str(format(i+1, 'x')).upper())
else:
    print("Please enter a valid range!")
    sys.exit(1)

print("The Adresses are:")
for i in adressList:
    print(i)

while(True):
    selected = input('\nChoose from the following options:\n   STM32 Power Down:\td\n   STM32 Power Up:\tu\n   STM32 Reboot:     \tr\n   Platform Shutdown:\tshutdown\n\n   Ctrl+C to exit\n')#\n   Upload Firmware:\tfirmware\n   (For upload you have to be in the project folder(or app layer))')
    if selected == "d":
        for a in adressList:
            print(f"Powering down the STM32 on {a}")
            try:
                PowerSwitch(a).stm_power_down()
            except Exception as e:
                print(f"Failed to power down the STM32 on {a}: {e}")

    if selected == "u":
        for a in adressList:
            print(f"Powering up the STM32 on {a}")
            try:
                PowerSwitch(a).stm_power_up()
            except Exception as e:
                print(f"Failed to power up the STM32 on {a}: {e}")

    if selected == "r":
        for a in adressList:
            print(f"Rebooting the STM32 on {a}")
            try:
                PowerSwitch(a).stm_power_cycle()
            except Exception as e:
                print(f"Failed to reboot the STM32 on {a}: {e}")

    if selected == "shutdown":
        for a in adressList:
            print(f"Shutting down the platform on {a}")
            try:
                PowerSwitch(a).platform_power_down()
            except Exception as e:
                print(f"Failed to shutdown the platform on {a}: {e}")

    # if input == "firmware":
    #     for a in adressList:
    #         print(f"Uploading the firmware on {a}")
    #         try:
    #             subprocess.run("python.exe -m cfloader flash build/cf2.bin stm32-fw -w {a}")
    #         except Exception as e:
    #             print(f"Failed to upload the firmware on {a}: {e}")