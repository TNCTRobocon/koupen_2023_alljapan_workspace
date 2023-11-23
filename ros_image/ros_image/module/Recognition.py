from ultralytics import YOLO
import cv2

from dataclasses import dataclass


class Recog():
    # None判定しろ
    def __init__(self):
        self.model = YOLO("honrobo_main/src/ros_main/weights/best.pt") # PATH
        self.detected_none_count = 0
    def detect_fruits(self, image):
        results = self.model.track(source=image, tracker="botsort.yaml", conf=0.5, iou=0.5, show=False)
        bbox = results[0].boxes.xyxy
        bbox_np = bbox.to('cpu').detach().numpy().copy()
        return bbox_np
    
    def calc_point(self, image, bbox_np):
        y, x, _ = image.shape
        f_c_x = int(x * 0.5)
        f_c_y = int(y * 0.5)

        origin_point = OriginPoint(x, y, f_c_x, f_c_y)
        
        if bbox_np.size == 0:
            return origin_point, None
        
        detected_list = []
        
        for i in range(len(bbox_np)):
            x1 = int(bbox_np[i][0])
            y1 = int(bbox_np[i][1])
            x2 = int(bbox_np[i][2])
            y2 = int(bbox_np[i][3])
            
            c_x = int((x1 + x2) * 0.5)
            c_y = int((y1 + y2) * 0.5)
            
            h_x1 = int((x1 + c_x)*0.5)
            h_x2 = int((x2 + c_x)*0.5)
            
            detected_rect_point = RectPoint(x1, y1, x2, y2,
                                            c_x, c_y,
                                            h_x1, h_x2)
            detected_list.append(detected_rect_point)

        return origin_point, detected_list

    def draw_frame_line(self, image, origin_point):
        cv2.line(image, (origin_point.origin_frame_centor_x, 0), (origin_point.origin_frame_centor_x, origin_point.origin_frame_y), (0, 255, 0), thickness=4)
        
        return image
    
    def draw_all_fruits_line(self, image, detected_list):
        for detected_rect_point in detected_list:
            cv2.rectangle(image, (detected_rect_point.detected_x1, detected_rect_point.detected_y1), 
                        (detected_rect_point.detected_x2, detected_rect_point.detected_y2), (0,255,0))
            cv2.rectangle(image, (detected_rect_point.helf_detected_x1, detected_rect_point.detected_y1), 
                        (detected_rect_point.helf_detected_x2, detected_rect_point.detected_y2), (0,255,0))    
        return image
    
    def mark_pointed_fruits(self, image, origin_point, detected_rect_point):
        diff = int((detected_rect_point.detected_x2 - detected_rect_point.detected_x1) / 4)
        cv2.line(image, (origin_point.origin_frame_centor_x, detected_rect_point.detected_centor_y), 
                (detected_rect_point.detected_centor_x, detected_rect_point.detected_centor_y), (0, 255, 0), thickness=4)
        cv2.rectangle(image, (detected_rect_point.detected_centor_x - diff, detected_rect_point.detected_centor_y - diff), (detected_rect_point.detected_centor_x + diff, detected_rect_point.detected_centor_y + diff), (0, 255, 0))
        return image
        
    def calc_side_movement(self, origin_point, detected_rect_point):
        distance_from_centor = abs(origin_point.origin_frame_centor_x - detected_rect_point.detected_centor_x)
        per_of_screen = ((detected_rect_point.detected_x2 - detected_rect_point.detected_x1) * (detected_rect_point.detected_y2 - detected_rect_point.detected_y1)  /  (origin_point.origin_frame_x * origin_point.origin_frame_y))
        if (detected_rect_point.detected_centor_x < origin_point.origin_frame_centor_x):
            direction = 1
        else:
            direction = -1
        if detected_rect_point.helf_detected_x1 < origin_point.origin_frame_centor_x < detected_rect_point.helf_detected_x2:
            return 0
        move_distance = (distance_from_centor / origin_point.origin_frame_x) * (1 - per_of_screen) * direction
        return move_distance
    
    def calc_front_distance(self, detected_rect_point, result):
        i = 1
        sum_meters = 0
        x = detected_rect_point.detected_centor_x
        y = detected_rect_point.detected_centor_y
        diff = int((detected_rect_point.detected_x2 - detected_rect_point.detected_x1) / 4)
        for new_x in range(x-diff, x+diff, 1):
            for new_y in range(y-diff, y+diff, 1):
                sum_meters += result.get_distance(new_x, new_y)
                i += 1
        return sum_meters / i
    
    # def calc_front_movement(self, detected_rect_point, result):
    #     x = detected_rect_point.detected_centor_x
    #     y = detected_rect_point.detected_centor_y
    #     depth_data_meters = result.get_distance(x,y)
        
    #     # if depth_data_meters < 0.1:
    #     #     return 0
        
    #     return depth_data_meters
    
    def detecting_check(self, bbox_np, mode):
        
        if bbox_np.size == 0 or mode != 1:
            self.detected_none_count += 1
        else :
            self.detected_none_count = 0
        print(self.detected_none_count)
        return self.detected_none_count
    
    def search_pointed_fruits_from_list(self, detect_list, point):
        for detect in detect_list:
            if (detect.detected_x1 < point.x < detect.detected_x2) and (detect.detected_y1 < point.y < detect.detected_y2):
                return detect
            
            
    

       

@dataclass
class OriginPoint:
    origin_frame_x : int
    origin_frame_y : int
    origin_frame_centor_x : int
    origin_frame_centor_y : int
    

@dataclass
class RectPoint:
    detected_x1 : int
    detected_y1 : int
    detected_x2 : int
    detected_y2 : int
    
    detected_centor_x : int
    detected_centor_y : int
    
    helf_detected_x1 : int
    helf_detected_x2 : int
    