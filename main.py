
########################################################################
#
# Copyright (c) 2022, STEREOLABS.
#
# All rights reserved.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

import keyboard
import sys
import pyzed.sl as sl
from signal import signal, SIGINT
import os

zed_list = []

cameras = sl.Camera.get_device_list()
index = 0

save_path = 'C:/Users/xjack/OneDrive/Pulpit/Magisterka sem 1/stereolab camera/nagrywanie'
path_list = []

init = sl.InitParameters()
init.camera_resolution = sl.RESOLUTION.HD720
init.depth_mode = sl.DEPTH_MODE.NONE
init.camera_fps = 30

for cam in cameras:
    print('opening' + str(cam))
    path_list.append(save_path + '/' + "ZED {}".format(cam.serial_number))
    init.set_from_serial_number(cam.serial_number)
    zed_list.append(sl.Camera())
    status = zed_list[index].open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        zed_list[index].close()
    index = index + 1

    for path in path_list:
        if (os.path.isdir(path) == False):
            os.mkdir(path)

def handler(signal_received, frame):
    cam.disable_recording()
    cam.close()
    sys.exit(0)

signal(SIGINT, handler)

def main():
    #   if not sys.argv or len(sys.argv) != 2:
    #       print("Only the path of the output SVO file should be passed as argument.")
    #      exit(1)

    # for cam in zed_list:
    #   status = cam.open(init)
    #  print('xd3')
    # if status != sl.ERROR_CODE.SUCCESS:
    #    print(repr(status))
    #   exit(1)

    # path_output = sys.argv[1]
    print('starting')

    key = ''
    print('press r to start recording, press s to stop')
    num_of_record=0
    while (key != 'q'):


        while (key != 'r'):
            # print(key)
            key = keyboard.read_key()

        it = 0

        for cam in zed_list:
            recording_param = sl.RecordingParameters(path_list[it] + '/' +str(num_of_record) +'.svo', sl.SVO_COMPRESSION_MODE.H264)
            err = cam.enable_recording(recording_param)
            print(repr(err))
            if err != sl.ERROR_CODE.SUCCESS:
                print(repr(status))
                exit(1)
            it = it + 1

        runtime = sl.RuntimeParameters()
        #print("SVO is Recording, use Ctrl-C to stop.")
        frames_recorded = [0, 0]

        while key != 's':
            for index in range(0, len(zed_list)):
                if zed_list[index].grab(runtime) == sl.ERROR_CODE.SUCCESS:
                    frames_recorded[index] += 1
                print("Frame count for camera " + str(index) + " : " + str(frames_recorded[index]))
            try:  # used try so that if user pressed other than the given key error will not be shown
                if keyboard.is_pressed('s'):  # if key 's' is pressed
                    print('stopping')
                    break  # finishing the loop
            except:
                break  # if user pressed a key other than the given key the loop will break

        for cam in zed_list:
            err = cam.disable_recording()
            print(repr(err))

        #while key == 's':
         #   print('press r to start recording, press s to stop, q to quit')
        key = keyboard.read_key()
           # print(key)
        num_of_record=num_of_record+1

if __name__ == "__main__":
     main()

