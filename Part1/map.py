
#!/usr/bin/python3
import cv2
import numpy as np


class Map:
    def __init__(self):
        self.height = 1020
        self.width = 1020

        self.scaling = 100
        self.black = (255, 0, 0)

        self.square_length = 150

        self.square_coords = np.array([(235, 135),
                                       (35, self.height - 510 - 75),
                                       (self.width - 110 - 75, self.height - 510 - 75)],
                                      dtype=np.int32)

        self.circle_radius = self.scaling * 1

        self.circle_centers = np.array([(self.width - 310, 210),
                                        (510, 510),
                                        (310, self.height - 210),
                                        (self.width - 310, self.height - 210)],
                                       dtype=np.int32)
        self.map_img = self.draw_obstacles()

    def draw_circle(self, img, thresh=0):

        for center in self.circle_centers:
            cv2.circle(img, (center[0], center[1]), self.circle_radius + thresh, self.black, -1)

    def draw_squares(self, img, thresh=0):
        for corner in self.square_coords:
            top_corner = (corner[0] - thresh), (corner[1] - thresh)
            bottom_corner = (corner[0] + self.square_length + thresh), (corner[1] + self.square_length + thresh)
            cv2.rectangle(img, top_corner, bottom_corner, self.black, -1)

    def draw_obstacles(self):
        self.map_img = cv2.imread('../images/map.png')
        if self.map_img is None:
            self.map_img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            self.map_img.fill(255)
            self.draw_circle(self.map_img)
            self.draw_squares(self.map_img)
            cv2.imwrite('../images/map.png', self.map_img)

        return self.map_img
