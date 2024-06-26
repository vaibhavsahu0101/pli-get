"""
pli acquisition
RT, 27 june 2023, 10h53
RT, VS, 14 july 2023, 21h29
RT, 24 july 2023, 22h31
"""

import pathlib
import os
import asyncio
import argparse
import time
import numpy as np
from pypylon import pylon
from skimage import io
import serial
import serial.tools.list_ports

## PLI machine

class DummySerial:
    '''Dummy serial port for testing.'''
    in_waiting = False
    def __init__(self):
        pass
    def write(self, string):
        print(f"DummySerial > {string.strip()}")
    def readline(self):
        return "ready"

class DummyCamera:
    '''Dummy camera for testing.'''
    def __init__(self):
        pass
    def Open(self):
        pass
    def Close(self):
        pass
    def StartGrabbing(self, grab_strategy):
        pass
    def RetrieveResult(self, timeout, timeout_handling):
        # return DummyGrabResult()
        return None
    def StopGrabbing(self):
        pass

class PLI:
    '''PLI machine class.'''

    pli_serial = None # serial connection to the PLI machine
    camera = None # Basler camera
    xy_stage_serial = None # XY stage

    images = [] # array to store all images
    channel = 1 # green, default image channel to capture
    steps = 0 # current number of steps in the PLI machine
    status = "idle"
    motor_busy = [False, False, False]
    debug = False

    n_polarisers = None
    n_stepper_steps = None
    n_large_gear_teeth = None
    n_small_gear_teeth = None

    n_angles = None

    # PLI arduino connection
    def process_message(self, message):
        '''Process a message received from the PLI machine.'''

        print("PLI >", message)

        if message == "ready":
            self.status = "ready"

        arr = message.split(": ")
        if arr[0] == "done":
            motor_number =  int(arr[1])
            self.motor_busy[motor_number] = False
            if self.motor_busy[1] is False and self.motor_busy[2] is False:
                self.status = "ready"
                print("motors ready")

    async def listen_for_xy_stage_messages(self, ser):
        '''Listen for messages from the XY stage.'''

        print("listen for stage messages")
        while True:
            if ser.in_waiting:
                message = ser.readline().decode().strip()
                print(message)
            await asyncio.sleep(0.1)

    async def listen_for_messages(self, ser):
        '''Listen for messages from the PLI machine.'''

        while self.status != "done":
            if ser.in_waiting:
                message = ser.readline().decode().strip()
                self.process_message(message)
            await asyncio.sleep(0.1)

    async def wait_for_ready(self):
        '''Wait for the PLI machine to be ready.'''

        while self.status != "ready":
            await asyncio.sleep(0.1)

    def new_xy_stage(self):
        ports = serial.tools.list_ports.comports()
        xy_stage_port = None
        for port, desc, hwid in sorted(ports):
            if self.debug:
                print(f"{port}: {desc} [{hwid}]")
            if "140" in port:
                xy_stage_port = port
        if self.debug:
            print(f"xy_stage_port: {xy_stage_port}")

        ser = None
        if xy_stage_port is None:
            print("XY stage not found.")
        else:
            ser = serial.Serial(xy_stage_port, 115200)
            for _ in range(2):
                output = ser.readline()
                print(output)
            ser.write(b"$G\n")
            for _ in range(2):
                output = ser.readline()
                print(output)
        return ser

    def xy_stage_write(self, string, sleep=0.01):
        '''Write a string to the xy stage.'''

        bin_cmd = string.encode() # bytes(string, "ascii")

        # add carriage return at the end
        bin_cmd += b"\r\n"

        print(bin_cmd)
        self.xy_stage_serial.write(bin_cmd)
        for _ in range(1):
            output = self.xy_stage_serial.readline()
            print(output)
        if sleep:
            time.sleep(sleep)

    def new_pli(self):
        '''Create a new serial connection to the PLI machine.'''

        ports = serial.tools.list_ports.comports()
        arduino_port = None
        xy_stage_port = None
        for port, desc, hwid in sorted(ports):
            if self.debug:
                print(f"{port}: {desc} [{hwid}]")
            # if "usbmodem" in port or "usbserial" in port:
            if "AB0LS12X" in port:
                arduino_port = port
        if self.debug:
            print(f"arduino_port: {arduino_port}")

        ser = None
        if arduino_port is None:
            print("Arduino not found: returning dummy serial port.")
            ser = DummySerial()
        else:
            ser = serial.Serial(arduino_port, 9600)
        return ser

    def pli_write(self, string, sleep=0.01):
        '''Write a string to the PLI machine.'''
        bin_cmd = bytes(string, "ascii")
        if string.startswith("delay"):
            bin_cmd += b"\r\n"
        self.pli_serial.write(bin_cmd)
        if sleep:
            time.sleep(sleep)

    def move(self, motor_number, delta):
        '''move one motor by delta steps.'''

        direction = "+" if delta > 0 else "-"
        string = f"{motor_number}{direction}{abs(delta)}\n"
        self.pli_write(string)

        if motor_number == 2:
            self.steps -= delta

        self.status = "moving"
        self.motor_busy[motor_number] = True

    def move2(self, delta):
        '''move two motors by delta steps.'''

        self.move(1, delta)
        self.move(2, -delta)

    def move_to(self, pos):
        '''move to a given position.'''

        delta = pos - self.steps
        if self.debug:
            print(f"move_to delta: {delta}")

        if delta == 0:
            return

        self.move(1, delta)
        self.move(2, -delta)

    def move_to_home(self):
        '''move to home position.'''

        if self.debug:
            print(f"move_to_home from {self.steps} to 0")
        self.move_to(0)
    
    def reset_steps(self):
        '''reset the number of steps.'''

        self.steps = 0

    ## Basler camera

    def new_camera(self):
        '''Create a new camera instance.'''

        camera = None
        try:
            camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
            print("Got camera:", camera.GetDeviceInfo().GetModelName())
        except:
            print("Camera not found: returning empty camera.")
        return camera

    def camera_open(self):
        '''Open the camera.'''
        if self.camera is None:
            print("Empty camera: Open")
        else:
            self.camera.Open()

    def camera_close(self):
        '''Close the camera.'''
        if self.camera is None:
            print("Empty camera: Close")
        else:
            self.camera.Close()

    ## Settings

    def set_n_angles(self, n_angles):
        '''Set the number of angles.'''

        self.n_angles = n_angles

    def set_n_polarisers(self, n_polarisers):
        '''Set the number of polarisers.'''
        if n_polarisers != 1 and n_polarisers != 2:
            raise ValueError("n_polarisers must be 1 or 2")
        self.n_polarisers = n_polarisers
    
    def set_n_stepper_steps(self, n_stepper_steps):
        '''Set the number of steps in a whole turn of the stepper motors.'''
        self.n_stepper_steps = n_stepper_steps

    def set_n_large_gear_teeth(self, n_large_gear_teeth):
        '''Set the number of teeth in the large gear.'''
        self.n_large_gear_teeth = n_large_gear_teeth

    def set_n_small_gear_teeth(self, n_small_gear_teeth):
        '''Set the number of teeth in the small gear.'''
        self.n_small_gear_teeth = n_small_gear_teeth

    def set_color_mode(self, color_mode):
        '''Set the color mode of the camera.'''

        if self.camera is None:
            print(f"Empty camera: {color_mode}")
            return

        self.camera.Open()
        if color_mode == "RGB8":
            self.camera.PixelFormat.SetValue("RGB8")
        elif color_mode == "Mono8":
            self.camera.PixelFormat.SetValue("Mono8")
        elif color_mode == "Mono12":
            self.camera.PixelFormat.SetValue("Mono12")
        else:
            raise ValueError("color_mode must be RGB8 or Mono8 or Mono12")
        self.camera.Close()

    def set_gain(self, gain):
        '''Set the gain of the camera.'''

        if self.camera is None:
            print(f"Empty camera: {gain}")
            return

        self.camera.Open()
        self.camera.GainAuto.SetValue("Off")
        self.camera.Gain.SetValue(gain)
        self.camera.Close()
    
    def set_exposure(self, exposure):
        '''Set the exposure of the camera.'''

        if self.camera is None:
            print(f"Empty camera: {exposure}")
            return

        self.camera.Open()
        self.camera.ExposureAuto.SetValue("Off")
        self.camera.ExposureTime.SetValue(exposure)
        self.camera.Close()
    
    def set_gamma(self, gamma):
        '''Set the gamma of the camera.'''

        if self.camera is None:
            print(f"Empty camera: {gamma}")
            return

        self.camera.Open()
        self.camera.Gamma.SetValue(gamma)

    ## Acquisition functions

    # grab one image from the camera
    def grab_image(self, channel=1):
        '''Grab one image from the camera.
        Returns:
            img: numpy array of the image
        '''
        img = None

        if self.camera is None:
            print("Empty camera: Image")
            img = np.zeros((100, 100, 3), dtype=np.uint8)
        else:
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
            grab_result = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if grab_result.GrabSucceeded():
                img = grab_result.Array[:, :]
            grab_result.Release()
            self.camera.StopGrabbing()

        return img

    def grab(self, image_name = "test.png"):
        '''Grab one image from the camera. Appends the image in Image_Array.
        Returns:
            img: numpy array of the image
        '''
        img = self.grab_image()
        self.images.append([img, image_name])
        # print(f"Image shape: {img.shape[0]} x {img.shape[1]}")
        return img

    def save_all_images(self):
        '''Save all images in Image_Array.'''
        print("saving images...")
        _, computed_image_name = self.images[0]
        pathlib.Path(
            os.path.dirname(f"{computed_image_name}")
        ).mkdir(
            parents=True,
            exist_ok=True
        )
        for img, computed_image_name in self.images:
            io.imsave(f"{computed_image_name}", img)
        print("all images saved.")

    ## Autocalibration

    def mean_value_image(self, img=None):
        '''Get the mean value of an image for calibration.'''
        if img is None:
            img = self.grab_image()[::10, ::10]
        else:
            img = img[::10, ::10]
        res = np.mean(img.ravel())
        return res

    async def calibrate(self, initial_step):
        '''Calibrate the background of the PLI machine.
        Parameters:
            initial_step: initial step size
        Returns:
            vals: array of mean values
            pos: position of the PLI machine
        '''
        if self.pli_serial is None or self.camera is None:
            print("Empty PLI and camera: Calibrate")
            return [0], 0
        time.sleep(1)
        self.pli_write("delay 5")
        self.set_gamma(1)
        pos = 0
        dx = initial_step # pylint: disable=C0103
        val0 = self.mean_value_image()
        print("initial value:", val0)
        vals = [val0]
        while dx != 0:
            self.move(1, dx)
            await self.wait_for_ready()
            pos += dx
            val1 = self.mean_value_image()
            vals.append(val1)
            dy = val1 - val0 # pylint: disable=C0103
            if dy > 0:
                dx = -int(dx/2) # pylint: disable=C0103
            val0 = val1
        return vals, pos

    ## Constructor

    def __init__(self):
        self.camera = self.new_camera()
        self.xy_stage_serial = self.new_xy_stage()
        self.pli_serial = self.new_pli()

async def calibrate_task(pli):
    '''Calibrate the PLI machine.'''
    vals, pos = await pli.calibrate(20)
    print(vals)
    print(pos)
    pli.status = "done" # this stops the listener task and ends the script

async def acquire_task(pli, base_path):
    '''Acquire one slice of the sample.'''

    steps_whole_turn = pli.n_large_gear_teeth/pli.n_small_gear_teeth * pli.n_stepper_steps
    print("steps for a whole turn:", steps_whole_turn)

    # angles
    angle_step = steps_whole_turn/pli.n_angles
    print("setps in one angular displacement:", angle_step)

    pli.images = []
    dark_array = []

    await pli.wait_for_ready()
    pli.pli_write("delay 5")

    print("moving home")
    pli.move_to_home()
    await pli.wait_for_ready()
    print("home reached")


    # print("calibrating")
    # vals, pos = pli.calibrate(20)
    # await pli.wait_for_ready()
    # print(pos, vals[-1])

    for iteration in range(pli.n_angles):
        print(f"angle: {iteration}")

        pli.move_to(int(iteration * angle_step))
        await pli.wait_for_ready()

        img = pli.grab(f"{base_path}/{iteration}.tif")

        dark_array.append(pli.mean_value_image(img))

    pli.move_to_home()
    await pli.wait_for_ready()

    print(f"mean dark value: {np.mean(dark_array)}")
    print(f"dark value range: {np.min(dark_array)} - {np.max(dark_array)}")
    # plt.plot(dark_array)

    pli.save_all_images()

    pli.status = "done" # this stops the listener task and ends the script


def main(args):
    '''Main function.'''
    # create a new PLI machine instance,
    # and configure it
    pli = PLI()

    loop = asyncio.get_event_loop()
    listener_task = loop.create_task(pli.listen_for_messages(pli.pli_serial))
    # xy_stage_listener_task = loop.create_task(pli.listen_for_xy_stage_messages(pli.xy_stage_serial))

    if args.calibrate is True:
        commands_task = loop.create_task(calibrate_task(pli))
        tasks = [listener_task, commands_task]
        loop.run_until_complete(asyncio.wait(tasks))
        print("done calibrating")
    elif args.pli_commands:
        commands = args.pli_commands.split(";")
        for command in commands:
            command = command.strip()
            print(f"command: [{command}]")
            if command.startswith("wait"):
                secs = float(command.split(" ")[1])
                print(f"waiting for {secs} seconds")
                time.sleep(secs)
            else:
                pli.pli_write(command)
            time.sleep(1)
    elif args.xy_commands:
        commands = args.xy_commands.split(";")
        for command in commands:
            command = command.strip()
            print(f"command: [{command}]")
            if command.startswith("wait"):
                secs = float(command.split(" ")[1])
                print(f"waiting for {secs} seconds")
                time.sleep(secs)
            else:
                pli.xy_stage_write(command)
            time.sleep(1)
    elif args.acquire is True:
        if args.base_path is None:
            raise ValueError("base_path is required")
        if args.n_angles is None:
            raise ValueError("n_angles is required")
        if args.n_polarisers is None:
            raise ValueError("n_polarisers is required")
        if args.n_stepper_steps is None:
            raise ValueError("n_stepper_steps is required")
        if args.n_large_gear_teeth is None:
            raise ValueError("n_large_gear_teeth is required")
        if args.n_small_gear_teeth is None:
            raise ValueError("n_small_gear_teeth is required")
        if args.color_mode is None:
            raise ValueError("color_mode is required")
        if args.gain is None:
            raise ValueError("gain is required")
        if args.exposure is None:
            raise ValueError("exposure is required")
        if args.gamma is None:
            raise ValueError("gamma is required")
        
        base_path = args.base_path
        n_angles = args.n_angles
        n_polarisers = args.n_polarisers
        n_stepper_steps = args.n_stepper_steps
        n_large_gear_teeth = args.n_large_gear_teeth
        n_small_gear_teeth = args.n_small_gear_teeth
        color_mode = args.color_mode
        gain = args.gain
        exposure = args.exposure
        gamma = args.gamma

        pli.pli_write("delay 5", sleep=0)
        pli.reset_steps()

        pli.set_n_angles(n_angles)
        pli.set_n_polarisers(n_polarisers)
        pli.set_n_stepper_steps(n_stepper_steps)
        pli.set_n_large_gear_teeth(n_large_gear_teeth)
        pli.set_n_small_gear_teeth(n_small_gear_teeth)
        pli.set_color_mode(color_mode)
        pli.set_gain(gain)
        pli.set_exposure(exposure)
        pli.set_gamma(gamma)

        # start the acquisition
        commands_task = loop.create_task(acquire_task(pli, base_path))
        tasks = [listener_task, commands_task]
        loop.run_until_complete(asyncio.wait(tasks))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Acquire PLI data.')
    parser.add_argument('--calibrate', action='store_true', help='auto-calibrate the PLI machine')
    parser.add_argument('--acquire', action='store_true', help='acquire an image')
    parser.add_argument('--base_path', type=str, help='base path for the images')
    parser.add_argument('--n_angles', type=int, help='number of angles')
    parser.add_argument('--n_polarisers', type=int, help='machine number of polarisers')
    parser.add_argument('--n_stepper_steps', type=int, help='number of steps in a whole turn of the stepper motors')
    parser.add_argument('--n_large_gear_teeth', type=int, help='machine number of teeth in the large gear')
    parser.add_argument('--n_small_gear_teeth', type=int, help='machine number of teeth in the small gear')
    parser.add_argument('--color_mode', type=str, help='camera color mode')
    parser.add_argument('--gain', type=float, help='camera gain')
    parser.add_argument('--exposure', type=float, help='camera exposure time')
    parser.add_argument('--gamma', type=float, help='camera gamma')
    parser.add_argument('--pli_commands', type=str, help='raw command to send to the PLI machine')
    parser.add_argument('--xy_commands', type=str, help='raw command to send to the XY stage')
    parser.add_argument('--settings', help='print the current settings')

    main(parser.parse_args())

# my_dict = {
#     "calibrate": False,
#     "pli_commands": False,
#     "xy_commands": False,
#     "acquire": True,
#     "base_path": "~/Desktop/test",
#     "n_angles": 36,
#     "n_polarisers": 1,
#     "n_stepper_steps": 800,
#     "n_large_gear_teeth": 112,
#     "n_small_gear_teeth": 42,
#     "color_mode": "Mono8",
#     "gain": 9.555,
#     "exposure": 11000,
#     "gamma": 3.999
# }
# args = argparse.Namespace(**my_dict)
# main(args)
