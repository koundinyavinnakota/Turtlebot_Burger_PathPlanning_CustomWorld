import cv2
import numpy as np
from matplotlib import pyplot as plt

scale = 10
gmap = np.zeros((10*scale,10*scale,3),dtype="uint8")

#clearance
c = 3



for x in range(10*scale):
    for y in range(10*scale):

        #border condition
        if 0 <= y < c or 0 <= x < c or 10*scale-c <= x < 100 or 10*scale-c <= y < 100 :
            gmap[(scale*10-1) - y, x,0] = 255

        
        #with clearance
        circle_1_c = (y-2*scale) ** 2 + (x-2*scale) **2 - (1*scale+c) ** 2
        circle_2_c = (y-8*scale) ** 2 + (x-2*scale) **2 - (1*scale+c) ** 2

        #Square 1
        sq_1_1_c = y - (5.75 * scale +c) 
        sq_1_2_c = x - (0.25 *scale - c)
        sq_1_3_c = y - (4.25 * scale -c)
        sq_1_4_c = x - (1.75 *scale +c)

        #Rect 1
        rec_1_1_c = y - (5.75 *scale +c)
        rec_1_2_c = x - (6.25 *scale +c )
        rec_1_3_c = y - (4.25*scale - c)
        rec_1_4_c = x - (3.75*scale -c)

        #Rect 2
        rec_2_1_c = y - (4 *scale + c)
        rec_2_2_c = x - (8.75*scale + c)
        rec_2_3_c = y - (2*scale - c)
        rec_2_4_c = x - (7.25*scale -c)


        if circle_1_c < 0 or circle_2_c < 0 or (sq_1_1_c < 0 and sq_1_2_c > 0 and sq_1_3_c > 0 and sq_1_4_c <0 ) or (rec_1_1_c < 0 and rec_1_2_c < 0 and rec_1_3_c >0 and rec_1_4_c > 0) or (rec_2_1_c < 0 and rec_2_2_c < 0 and rec_2_3_c > 0 and rec_2_4_c > 0):
            gmap[(scale*10-1) - y, x,0] = 255


        #without clearance
        circle_1 = (y-2*scale) ** 2 + (x-2*scale) **2 - (1*scale) ** 2
        circle_2 = (y-8*scale) ** 2 + (x-2*scale) **2 - (1*scale) ** 2

        #Square 1
        sq_1_1 = y - 5.75 * scale
        sq_1_2 = x - 0.25 *scale
        sq_1_3 = y - 4.25 * scale
        sq_1_4 = x - 1.75 *scale

        #Rect 1
        rec_1_1 = y - 5.75*scale
        rec_1_2 = x - 6.25*scale
        rec_1_3 = y - 4.25*scale
        rec_1_4 = x - 3.75*scale

        #Rect 2
        rec_2_1 = y - 4*scale
        rec_2_2 = x - 8.75*scale
        rec_2_3 = y - 2*scale
        rec_2_4 = x - 7.25*scale

        # Condition for circle, Polygon & Hexagon
        if circle_1 < 0 or circle_2 < 0 or (sq_1_1 < 0 and sq_1_2 > 0 and sq_1_3 > 0 and sq_1_4 <0 ) or (rec_1_1 < 0 and rec_1_2 < 0 and rec_1_3 >0 and rec_1_4 > 0) or (rec_2_1 < 0 and rec_2_2 < 0 and rec_2_3 > 0 and rec_2_4 > 0):
            gmap[(scale*10-1) - y, x,1] = 255


