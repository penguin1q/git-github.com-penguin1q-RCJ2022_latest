# -*- coding: utf-8 -*-
import cv2
import numpy as np

class Find_rect:
    def __init__(self, hsv_min, hsv_max):
        self.hsv_min = hsv_min
        self.hsv_max = hsv_max

    def find_rect_find_target_color(self, hsv):
        if self.hsv_max < self.hsv_min:
            mask_image1 = cv2.inRange(hsv, (0, self.hsv_min[1], self.hsv_min[2]), tuple(self.hsv_max))
            mask_image2 = cv2.inRange(hsv, tuple(self.hsv_min), (255, self.hsv_max[1], self.hsv_max[2]))
            mask_image = mask_image1 + mask_image2
        else :
            mask_image = cv2.inRange(hsv, tuple(self.hsv_min), tuple(self.hsv_max))
        
        contours,_ =cv2.findContours(mask_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        rects = []
        for contour in contours:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            rects.append(np.array(rect))
        return rects

