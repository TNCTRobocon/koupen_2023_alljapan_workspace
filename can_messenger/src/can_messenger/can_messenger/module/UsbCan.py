import os
import can

class UsbCan:
    """constructor"""
    def __init__(self,debug_mode) :
        self.__state = False
        self.__debug_mode = debug_mode
        return
    
    """destructor"""
    def __del__(self):
        if self.__debug_mode:
            return
        if self.__state == True:
            os.system('sudo ifconfig can0 down')
        return
    
    """open can port"""
    def open(self):
        if self.__debug_mode:
            return
        self.__state = True
        try:
            os.system('sudo ip link set can0 type can bitrate 500000')
            os.system('sudo ifconfig can0 up')
            os.system('sudo ifconfig can0 txqueuelen 10000') #バッファオーバーフロー対策 
            self.__can0 = can.interface.Bus(channel = 'can0', bustype = 'socketcan')
        except OSError:
            print('OS Error')
        return
    
    """close can port"""
    def close(self):
        if self.__debug_mode:
            return
        self.__state = False
        os.system('sudo ifconfig can0 down')
        
        return
    
    """send data"""
    def send(self,msg):
        # try:
        if self.__debug_mode:
            return
        if self.__state == True:
            self.__can0.send(msg)
        return
        # except:
            # time.sleep(3)
            # print("can translate error")
            # print("reconnected...")
            # self.close()
            # self.open()
            
    