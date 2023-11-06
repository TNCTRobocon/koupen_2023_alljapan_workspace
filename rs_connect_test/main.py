import pyrealsense2 as rs2
import time

current_devices = []

def on_devices_changed(info):
    global current_devices
    devs = info.get_new_devices()
    print(len(devs))
    print(devs)
    print(devs[0])
    print(info.was_added(devs[0]))
    print(info.was_removed(devs[0]))
        
    

def main():
    ctx = rs2.context()
    ctx.set_devices_changed_callback(on_devices_changed)
    print("Please connect/disconnect your RealSense camera.")
    while True:
        pass

if __name__ == '__main__':
    main()