class ToggleSwitch():
    
    def __init__(self) -> None:
        self.__lasttime_button_data = [0] * 8
    
    def judge_changed_button(self,data):
        for i in range(len(data)):
            if (data[i] != self.__lasttime_button_data[i]) and (data[i] == 1):
                return_switch = i
                return return_switch
            else:
                return_switch = -1
        self.__lasttime_button_data = data
        return return_switch

                

# class SwitchStatus():
#     __button_status = [1] * 8
#     def __init__(self,min_status,max_status,num) -> None:
#         self.__min_status = min_status
#         self.__max_status = max_status
#         self.__num = num
        
#     def toggle_status(self,switch_num):
#         if self.__button_status[switch_num] >= self.__max_status: 
#             self.__button_status[switch_num] = self.__min_status
#         else:
#             self.__button_status[switch_num] += 1
        
#     def get_status(self):
#         __return_button = [0] * self.__num
#         for i in range(self.__num):
#             __return_button[i] = self.__button_status[i]
#         return __return_button
        
        