"""
pli acquisition
RT, 27 june 2023, 10h53
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

import matplotlib.pyplot as plt

## PLI machine

class DummySerial:
    '''Dummy serial port for testing.'''
    in_waiting = False
    def __init__(self):
        pass
    def write(self, string):
        print(f"DummySerial: {string.strip()}")
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

    images = [] # array to store all images
    channel = 1 # green, default image channel to capture
    steps = 0 # current number of steps in the PLI machine

    debug = True

    status = "idle"
    motor_busy = [False, False, False]

    # PLI arduino connection
    def process_message(self, message):
        '''Process a message received from the PLI machine.'''

        print("Received message:", message)

        if message == "ready":
            self.status = "ready"

        arr = message.split(": ")
        if arr[0] == "done":
            motor_number =  int(arr[1])
            self.motor_busy[motor_number] = False
            if self.motor_busy[1] is False and self.motor_busy[2] is False:
                self.status = "ready"
                print("motors ready")

    async def listen_for_messages(self, ser):
        '''Listen for messages from the PLI machine.'''

        print("listen_for_messages")
        while self.status != "done":
            if ser.in_waiting:
                message = ser.readline().decode().strip()
                self.process_message(message)
            await asyncio.sleep(0.1)

    async def wait_for_ready(self):
        '''Wait for the PLI machine to be ready.'''

        while self.status != "ready":
            await asyncio.sleep(0.1)

    def new_pli(self):
        '''Create a new serial connection to the PLI machine.'''

        ports = serial.tools.list_ports.comports()
        arduino_port = None
        for port, desc, hwid in sorted(ports):
            if self.debug:
                print(f"{port}: {desc} [{hwid}]")
            if "usbmodem" in port or "usbserial" in port:
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

        self.pli_serial.write(bytes(string, "ascii"))
        if sleep:
            time.sleep(sleep)

    def set_n_angles(self, n_angles):
        '''Set the number of angles.'''

        self.n_angles = n_angles
        self.angle_step = self.steps_whole_turn/n_angles
        print("setps in one angular displacement:", self.angle_step)

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
        print("home reached")
    
    def reset_steps(self):
        '''reset the number of steps.'''

        self.steps = 0

    ## Basler camera

    def new_camera(self):
        '''Create a new camera instance.'''

        camera = None
        try:
            camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
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
            raise Exception("color_mode must be RGB or Mono8 or Mono12")
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
        self.camera.GammaSelector.SetValue("User")
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
            print("will grab")
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
            print("grabbed")
            grab_result = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if grab_result.GrabSucceeded():
                print("grab succeeded")
                # img = grab_result.Array[:, :, channel]
                img = grab_result.Array[:, :]
            print("will release")
            grab_result.Release()
            print("will stop grabbing")
            self.camera.StopGrabbing()

        print("returning")
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
        _, computed_image_name = self.images[0]
        pathlib.Path(
            os.path.dirname(f"{computed_image_name}")
        ).mkdir(
            parents=True,
            exist_ok=True
        )
        for img, computed_image_name in self.images:
            img2 = (img - img.min())/(img.max() - img.min())
            io.imsave(f"{computed_image_name}", img2)
            # io.imsave(f"{computed_image_name}", img)
        print("all saved.")

    ## Autocalibration

    def mean_value_image(self, img=None):
        '''Get the mean value of an image for calibration.'''
        # channel 0, 1, 2 is red, green, blue
        if img is None:
            print("grabbing image")
            img = self.grab_image()[::10, ::10]
            print("done grabbing")
        else:
            img = img[::10, ::10]
        res = np.mean(img.ravel())
        return res

    def calibrate(self, initial_step):
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

        pos = 0
        dx = initial_step # pylint: disable=C0103
        print("preparing")
        val0 = self.mean_value_image()
        print("initial value:", val0)
        vals = [val0]
        while dx != 0:
            self.move(1, dx)
            pos += dx
            val1 = self.mean_value_image()
            vals.append(val1)
            dy = val1 - val0 # pylint: disable=C0103
            if dy > 0:
                dx = -int(dx/2) # pylint: disable=C0103
            val0 = val1
        return vals, pos

    def __init__(self, pol_gear_teeth=92, mot_gear_teeth=42, stepper_max=800, n_angles=36):
        # machine characteristics
        self.pol_gear_teeth = pol_gear_teeth
        self.mot_gear_teeth = mot_gear_teeth
        self.stepper_max = stepper_max
        self.steps_whole_turn = pol_gear_teeth/mot_gear_teeth*stepper_max
        print("steps for a whole turn:", self.steps_whole_turn)

        # angles
        self.n_angles = n_angles
        self.angle_step = self.steps_whole_turn/n_angles
        print("setps in one angular displacement:", self.angle_step)

        self.pli_serial = self.new_pli()
        self.camera = self.new_camera()

async def capture_one_slice(pli, base_path="./", name="slice_"):
    '''Capture one slice of the sample.'''

    pli.images = []
    dark_array = []

    print("moving home")
    pli.move_to_home()
    await pli.wait_for_ready()

    print("calibrating")
    vals, pos = pli.calibrate(20)
    print("a")
    await pli.wait_for_ready()
    print("b")
    print(pos, vals[-1])
    print("c")

    print("iterating")
    for iteration in range(pli.n_angles):
        print(f"angle: {iteration}")

        pli.move_to(int(iteration * pli.angle_step))
        await pli.wait_for_ready()

        img = pli.grab(f"{base_path}/{name}{iteration}.tif")

        dark_array.append(pli.mean_value_image(img))

    pli.move_to_home()
    await pli.wait_for_ready()

    print(f"mean dark value: {np.mean(dark_array)}")
    print(f"dark value range: {np.min(dark_array)} - {np.max(dark_array)}")
    # plt.plot(dark_array)

    pli.save_all_images()

    pli.status = "done" # this stops the listener task and ends the script


def main(base_path="./", name="slice_", n_angles=18, color_mode="Mono8"):
    '''Main function.'''
    # create a new PLI machine instance,
    # and configure it
    pli = PLI(pol_gear_teeth=96)
    pli.pli_write("delay 30\n", sleep=0)
    pli.set_color_mode(color_mode)

    #------------------------------------------
    # these values are for Víctor's brain slice
    pli.set_gain(25)
    pli.set_exposure(400000)
    pli.set_gamma(3.999)
    #------------------------------------------

    pli.set_n_angles(n_angles)
    pli.reset_steps()

    # start the acquisition
    loop = asyncio.get_event_loop()
    listener_task = loop.create_task(pli.listen_for_messages(pli.pli_serial))
    commands_task = loop.create_task(capture_one_slice(pli,
        base_path=base_path, name=name))
    tasks = [listener_task, commands_task]
    loop.run_until_complete(asyncio.wait(tasks))

# if __name__ == "__main__":
#     # parse arguments: mandatory arguments are base_path and name,
#     # optional arguments are n_angles and color_mode
#     parser = argparse.ArgumentParser(description='Acquire one slice of PLI data.')
#     parser.add_argument('base_path', type=str, help='base path for the images')
#     parser.add_argument('name', type=str, help='name of the images')
#     parser.add_argument('--n_angles', type=int, default=18, help='number of angles (default 18)')
#     parser.add_argument('--color_mode', type=str, default="RGB8", help='color mode (default RGB8)')
#     args = parser.parse_args()

#     # call main function passing the arguments
#     main(args.base_path, args.name, args.n_angles, args.color_mode)

main("images/sheep-olympus", "slice-1_", 18, "Mono12")