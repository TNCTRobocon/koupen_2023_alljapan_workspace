class JoyCalcTools():
    def __init__(self, PC):
        self.__PC = PC
        
    def recaluculating_joy(self, joy):
        recaluculated_joy = [0] * 8
        if self.__PC == 0:
            ###Potable-PC
            recaluculated_joy[0] = joy.axes[0] * 127 + 128 #left-horizontal
            recaluculated_joy[1] = joy.axes[1] * 127 + 128 #left-vertical
            recaluculated_joy[2] = joy.axes[4] * 127 + 128 #right-horizontal
            recaluculated_joy[3] = joy.axes[3] * -127 + 128 #right-vertical
            recaluculated_joy[4] = joy.axes[2] * 127 + 128 #left-trigger
            recaluculated_joy[5] = joy.axes[5] * 127 + 128 #right-trigger
            recaluculated_joy[6] = 0 #none
            recaluculated_joy[7] = 0 #none
        elif self.__PC == 1:
            ###F310
            recaluculated_joy[0] = joy.axes[0] * 127 + 128 #left-horizontal
            recaluculated_joy[1] = joy.axes[1] * 127 + 128 #left-vertical
            recaluculated_joy[2] = joy.axes[3] * 127 + 128 #right-horizontal
            recaluculated_joy[3] = joy.axes[2] * -127 + 128 #right-vertical
            recaluculated_joy[4] = 0 #none
            recaluculated_joy[5] = 0 #none
            recaluculated_joy[6] = 0 #none
            recaluculated_joy[7] = 0 #none
        else:
            pass
        
        ###common return
        recaluculated_joy = list(map(int, recaluculated_joy))
        # self.make_deadzone(recaluculated_joy)
        return recaluculated_joy
    
    def recaluculating_hat(self, joy):
        recaluculated_hat = [0] * 8
        if self.__PC == 0:
            ###Portable-PC
            recaluculated_hat[0] = joy.axes[7] + 1 #hat_vertical
            recaluculated_hat[1] = joy.axes[6] + 1 #hat_horizontal
            recaluculated_hat[2] = 0
            recaluculated_hat[3] = 0
            recaluculated_hat[4] = 0
            recaluculated_hat[5] = 0
            recaluculated_hat[6] = 0
            recaluculated_hat[7] = 0
        elif self.__PC == 1:
            ###F310
            recaluculated_hat[0] = joy.axes[5] + 1 #hat_vertical
            recaluculated_hat[1] = joy.axes[4] + 1 #hat_horizontal
            recaluculated_hat[2] = 0
            recaluculated_hat[3] = 0
            recaluculated_hat[4] = 0
            recaluculated_hat[5] = 0
            recaluculated_hat[6] = 0
            recaluculated_hat[7] = 0
        else :
            pass    
        
        ###common return
        return recaluculated_hat
    
    def deplicate_btn_data(self,joy):
        replased_button = [0] * 8
        for i in range(8):
            replased_button[i] = joy.buttons[i]
        return replased_button
    
    
    def override_joy(self,target_data,num,data):
        clamp = lambda x: min(255, max(x, 0))
        target_data[num] = clamp(target_data[num] + data)
        
        return target_data
    
    def copy_button(self,raw,pros):
        for i in range(len(pros)):
            raw[i] = pros[i]
        return raw
    
    def override_config(self,raw,config):
        for i in range(len(config)):
            raw[i + 2] = config[i]
        return raw
