import can

class UseMessage():
    def __init__(self,id):
        self.__id = id
        self.msg = can.Message(
            arbitration_id=self.__id,
            is_extended_id=False,
            data=None
        )
        
    def update_message(self,rcv_data):
        self.msg = can.Message(
            arbitration_id=self.__id,
            is_extended_id=False,
            data=[int(rcv_data[0]),int(rcv_data[1]),int(rcv_data[2]),int(rcv_data[3]),int(rcv_data[4]),int(rcv_data[5]),int(rcv_data[6]),int(rcv_data[7])]
        )
        return self.msg
    
    def getter(self):
        return self.msg