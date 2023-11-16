import customtkinter as ct
import tkinter
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

from enum import Enum

class Preset(Enum):
    DOWN = 1
    UP = 2
    BELT_ON = 2
    BELT_OFF = 1

class App(ct.CTk):
    FONT_TYPE = "meiryo"
    config_keeper = [1, 1, 1, 1, 1, 1]
    last_limit = [1] * 6
    num_of_config = [3, 2, 2, 2, 2, 2]
    config2 = [0,0,0,0,0,0,0,0]
    
    color_config = ["#bf3a7a", "#3a7ebf"]
    color_hover_config = ["#823275", "#325882"]
    
    button_obj_keeper = []
    
    now_preset = 0
    
    preset_config = [[Preset.DOWN.value, Preset.DOWN.value], [Preset.UP.value, Preset.DOWN.value], [Preset.UP.value, Preset.UP.value], [Preset.DOWN.value, Preset.UP.value], [Preset.DOWN.value, Preset.DOWN.value]]
    def __init__(self):
        super().__init__()
        rclpy.init()
        self.ros_gui = RosGui()
        
        self.fonts = (self.FONT_TYPE, 25)
        self.geometry("590x700")
        self.title("GUI")
        
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)
        self.grid_columnconfigure(2, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)
        self.grid_rowconfigure(2, weight=1)
        self.grid_rowconfigure(3, weight=1)
        self.grid_rowconfigure(4, weight=1)
        self.grid_rowconfigure(5, weight=1)
        self.grid_rowconfigure(6, weight=1)
        self.grid_rowconfigure(7, weight=1)
        self.grid_rowconfigure(8, weight=1)
        
        self.setup_form()
        
        self.in_roop()
        
    def in_roop(self):
        rclpy.spin_once(self.ros_gui, timeout_sec=0.1)
        
        self.conf7_label.configure(text="段差乗り越え %d/%d\nSpeed %d"%(self.now_preset, len(self.preset_config) - 1, self.ros_gui.msg[0]))
        self.updates()
        
        self.after(10, self.in_roop)

    def setup_form(self):
        ct.set_appearance_mode("dark")
        ct.set_default_color_theme("blue")
        self.radio_var = ct.IntVar(value=0)
        
        self.conf1_btn1 = ct.CTkButton(master=self, width=180, height=100, text="通過しない", command=lambda a = 1, b = 1, c = 1 :self.callback(a,b,c), font=self.fonts)
        self.conf1_btn1.grid(column=0, row=5, padx=5, pady=5)
        self.conf1_btn2 = ct.CTkButton(master=self, width=180, height=100, text="通過する", command=lambda a = 1, b = 1, c = 2 :self.callback(a,b,c), font=self.fonts)
        self.conf1_btn2.grid(column=1, row=5, padx=5, pady=5)
        self.conf1_btn3 = ct.CTkButton(master=self, width=180, height=100, text="はじめに戻す", command=lambda a = 1, b = 1, c = 3 :self.callback(a,b,c), font=self.fonts)
        self.conf1_btn3.grid(column=2, row=5, padx=5, pady=5)
        
        self.button_obj_keeper.append([self.conf1_btn1, self.conf1_btn2, self.conf1_btn3])
        
        self.conf2_btn1 = ct.CTkButton(master=self, width=180, height=80, text="うごかない", command=lambda a = 2, b = 1, c = 0 :self.callback(a,b,c), font=self.fonts)
        self.conf2_btn1.grid(column=0, row=0, padx=5, pady=5)
        self.conf2_label = ct.CTkLabel(master=self, width=180, height=80, text="ベルト", font=self.fonts)
        self.conf2_label.grid(column=1, row=0, padx=5, pady=5)
        self.conf2_btn2 = ct.CTkButton(master=self, width=180, height=80, text="うごく", command=lambda a = 2, b = 0, c = 0 :self.callback(a,b,c), font=self.fonts)
        self.conf2_btn2.grid(column=2, row=0, padx=5, pady=5)
        
        self.button_obj_keeper.append([self.conf2_btn1, self.conf2_btn2])
        
        self.conf3_btn1 = ct.CTkButton(master=self, width=180, height=80, text="さがる", command=lambda a = 3, b = 1, c = 0 :self.callback(a,b,c), font=self.fonts)
        self.conf3_btn1.grid(column=0, row=1, padx=5, pady=5)
        self.conf3_label = ct.CTkLabel(master=self, width=180, height=80, text="前タイヤ", font=self.fonts)
        self.conf3_label.grid(column=1, row=1, padx=5, pady=5)
        self.conf3_btn2 = ct.CTkButton(master=self, width=180, height=80, text="あがる", command=lambda a = 3, b = 0, c = 0 :self.callback(a,b,c), font=self.fonts)
        self.conf3_btn2.grid(column=2, row=1, padx=5, pady=5)
        
        self.button_obj_keeper.append([self.conf3_btn1, self.conf3_btn2])
        
        self.conf5_btn1 = ct.CTkButton(master=self, width=180, height=80, text="さがる", command=lambda a = 4, b = 1, c = 0 :self.callback(a,b,c), font=self.fonts)
        self.conf5_btn1.grid(column=0, row=3, padx=5, pady=5)
        self.conf5_label = ct.CTkLabel(master=self, width=180, height=80, text="後ろタイヤ", font=self.fonts)
        self.conf5_label.grid(column=1, row=3, padx=5, pady=5)
        self.conf5_btn2 = ct.CTkButton(master=self, width=180, height=80, text="あがる", command=lambda a = 5, b = 0, c = 0 :self.callback(a,b,c), font=self.fonts)
        self.conf5_btn2.grid(column=2, row=3, padx=5, pady=5)
        
        self.button_obj_keeper.append([self.conf5_btn1, self.conf5_btn2])
        
        self.conf6_btn1 = ct.CTkButton(master=self, width=180, height=80, text="オン", command=lambda a = 5, b = 1, c = 0 :self.callback(a,b,c), font=self.fonts)
        self.conf6_btn1.grid(column=0, row=4, padx=5, pady=5)
        self.conf6_label = ct.CTkLabel(master=self, width=180, height=80, text="自動操縦", font=self.fonts)
        self.conf6_label.grid(column=1, row=4, padx=5, pady=5)
        self.conf6_btn2 = ct.CTkButton(master=self, width=180, height=80, text="オフ", command=lambda a = 5, b = 0, c = 0 :self.callback(a,b,c), font=self.fonts)
        self.conf6_btn2.grid(column=2, row=4, padx=5, pady=5)
        
        self.button_obj_keeper.append([self.conf6_btn1, self.conf6_btn2])
        
        self.conf7_btn1 = ct.CTkButton(master=self, width=180, height=200, text="戻る", command=self.apply_preset_back, font=self.fonts)
        self.conf7_btn1.grid(column=0, row=6, padx=5, pady=5)
        self.conf7_label = ct.CTkLabel(master=self, width=180, height=200, text="段差乗り越え %d/%d\nSpeed %d"%(self.now_preset,len(self.preset_config) - 1,self.ros_gui.msg[0]), font=self.fonts)
        self.conf7_label.grid(column=1, row=6, padx=5, pady=5)
        self.conf7_btn2 = ct.CTkButton(master=self, width=180, height=200, text="進む", command=self.apply_preset_next, font=self.fonts)
        self.conf7_btn2.grid(column=2, row=6, padx=5, pady=5)
        
        self.retry_btn = ct.CTkButton(master=self, width=180, height=100, text="リトライ", command=self.retry, font=self.fonts)
        self.retry_btn.grid(column=0, row=7, padx=5, pady=5)
        self.radio_btn1 = ct.CTkRadioButton(master=self, text="DepthAI",command=self.change_camera, variable=self.radio_var, value=0)
        self.radio_btn1.grid(column=1, row=7, padx=5, pady=5)
        self.radio_btn2 = ct.CTkRadioButton(master=self, text="Realsense",command=self.change_camera, variable=self.radio_var, value=1)
        self.radio_btn2.grid(column=2, row=7, padx=5, pady=5)

        self.updates()
        
    def change_camera(self):
        self.config2[0] = self.radio_var.get()
        self.ros_gui.cvt_and_send2(self.config2)
            
    def retry(self):
        self.now_preset = 0
        self.now_timber_preset = 0
        
        for i in range(2):
            self.config_keeper[i + 2] = 1
        
        self.config_keeper[1] = Preset.BELT_OFF.value
            
        self.conf7_label.configure(text="段差乗り越え %d/%d\nSpeed %d"%(self.now_preset, len(self.preset_config) - 1, self.ros_gui.msg[0]))
        self.updates()
        
    
    def callback(self,config,mode,me):
        if config:
            self.config_keeper[config - 1] = me
        else:
            if mode:
                self.config_keeper[config - 1] += 1
                if self.config_keeper[config - 1] >= self.num_of_config[config - 1]:
                    self.config_keeper[config - 1] = 1
            else:
                self.config_keeper[config - 1] -= 1
                if self.config_keeper[config - 1] <= 1:
                    self.config_keeper[config - 1] = self.num_of_config[config - 1]
        self.updates()
            
    def updates(self):
        print(self.config_keeper)
        if self.ros_gui.limit[2] != self.last_limit[2] and self.last_limit[2] == 0:
            self.config_keeper[0] = 2
        for i in range(len(self.button_obj_keeper)):
            i_list = self.button_obj_keeper[i]
            for j in range(len(i_list)):
                target_obj = i_list[j] 
                target_obj.configure(fg_color=self.color_config[j + 1 != self.config_keeper[i]])
                target_obj.configure(hover_color=self.color_hover_config[j + 1 != self.config_keeper[i]])
        self.last_limit = self.ros_gui.limit
        self.ros_gui.cvt_and_send(self.config_keeper)
        self.ros_gui.cvt_and_send2(self.config2)
        
    def apply_preset_next(self):
        self.now_preset += 1
        if self.now_preset > len(self.preset_config) - 1:
            self.now_preset = 0
                
        for i in range(2):
            self.config_keeper[i + 2] = self.preset_config[self.now_preset][i]
        
        if self.now_preset == 0:
            self.config_keeper[1] = Preset.BELT_OFF.value
        else:
            self.config_keeper[1] = Preset.BELT_ON.value
            
        self.conf7_label.configure(text="段差乗り越え %d/%d\nSpeed %d"%(self.now_preset,len(self.preset_config) - 1,self.ros_gui.msg[0]))
        self.updates()
        
        # if self.now_preset == 1 or self.now_preset == 4 or self.now_preset == 7 or self.now_preset == 8:
        #     self.after(500,self.apply_preset_next)
            
    def apply_preset_back(self):
        self.now_preset -= 1
        if self.now_preset < 0:
            self.now_preset = len(self.preset_config) - 1
                
        for i in range(2):
            self.config_keeper[i + 2] = self.preset_config[self.now_preset][i]
        
        if self.now_preset == 0:
            self.config_keeper[1] = Preset.BELT_ON.value
        else:
            self.config_keeper[1] = Preset.BELT_OFF.value
            
        self.conf7_label.configure(text="段差乗り越え %d/%d\nSpeed %d"%(self.now_preset,len(self.preset_config) - 1,self.ros_gui.msg[0]))
        self.updates()
            

class RosGui(Node):
    node_name = "ros_gui"
    config_pub_topic_name = "config"
    config2_pub_topic_name = "config2"
    limit_pub_topic_name = "limit"
    config_sub_topic_name = "can_btn"
    msg = [0,0,0,0,0,0,0,0]
    msg2 = [0,0,0,0,0,0,0,0]
    limit = [0] * 8
    
    def __init__(self):
        super().__init__(self.node_name)
        self.pub_config = self.create_publisher(Int16MultiArray, self.config_pub_topic_name, 10)
        self.pub_config2 = self.create_publisher(Int16MultiArray, self.config2_pub_topic_name, 10)
        self.pub_limit = self.create_subscription(Int16MultiArray, self.limit_pub_topic_name, self.limit_callback, 10)
        self.sub_config = self.create_subscription(Int16MultiArray, self.config_sub_topic_name, self.callback, 10)

    def cvt_and_send(self,data):
        send_data = Int16MultiArray(data=data)
        self.pub_config.publish(send_data)
        
    def cvt_and_send2(self,data):
        send_data = Int16MultiArray(data=data)
        self.pub_config2.publish(send_data)
        
    def callback(self,data):
        self.msg = data.data
        self.get_logger().info(self.msg)
        
    def limit_callback(self, data):
        self.limit = data.data
        # self.get_logger().info(self.limit)
        
        
def main():
    try: 
        app = App()
        app.mainloop()
    except KeyboardInterrupt:
        app.destroy()
        
if __name__ == '__main__':
    main()