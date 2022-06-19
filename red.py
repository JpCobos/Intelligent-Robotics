#!/usr/bin/env python3
import tensorflow as tf
from colorsys import hsv_to_rgb
import rospy  
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError 
import cv2
import numpy as np 
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32 
import time
from tensorflow.keras.preprocessing import image as image_utils

new_model = tf.keras.models.load_model('/home/angie/catkin_ws/src/beginner_tutorials/scripts/myModel.h5')

new_model.summary()

def greenOrRed(cv_image):
   color = 0
   hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) 
         
   min_green = np.array([45,120,120]) 
   max_green = np.array([75,255,255])  

   min_red = np.array([155,120,120]) 
   max_red = np.array([180,255,255]) 
   
   mask_g = cv2.inRange(hsv, min_green, max_green) 
   mask_r = cv2.inRange(hsv, min_red, max_red) 

   green_p = cv2.countNonZero(mask_g)
   red_p = cv2.countNonZero(mask_r)
   if (green_p > 5000):
      color = 0
      return color    
   elif (red_p > 10000 and green_p < 5000):
      color = 1
      return color

#print(array[np.argmax(prediction)])
#image = cv2.imread('home/angie/424.png')

class ColorFilter():  
   def __init__(self):  
      rospy.on_shutdown(self.cleanup) 
      ############ CONSTANTS ################  
      self.bridge = CvBridge() # create the cv_bridge object 
      #self.keypoints = self.detector.detect()
      self.image_received = 0 #Flag to indicate that we have already received an image 
      self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10) 
      self.rate = rospy.Rate(10) # 10hz delay

      ############################### SUBSCRIBERS #####################################  
      image_sub = rospy.Subscriber("/video_source/raw", Image, self.image_cb)
      self.dt =1/50.0

      while not rospy.is_shutdown(): #Mientras rospy no este apagadp
         # self.pub.publish(self.vel) #Publicando 
         self.rate.sleep()
         freq=20
         rate = rospy.Rate(freq) #50Hz  
         self.dt =1/20.0
            
   def image_cb(self, ros_image):  
      ## This function receives a ROS image and transforms it into opencv format   
      try:
         cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
         color = greenOrRed(cv_image)
         if color == 0:
            cv2.putText(cv_image, "Light: Green", (50,400), cv2.FONT_ITALIC, 2, (255, 255, 255), 5, cv2.LINE_AA)
            print("Color: Green")
         else:
            cv2.putText(cv_image, "Light: Red", (50,400), cv2.FONT_ITALIC, 2, (255, 255, 255), 5, cv2.LINE_AA)
            print("Color: Red")
         cv2.imwrite('/home/angie/image.png', cv_image)
         image = image_utils.load_img('/home/angie/image.png', target_size=(250,250))
         image = image_utils.img_to_array(image)
         image = image.reshape(1,250,250,3) 
         prediction = new_model.predict(image)
         prediction = np.argmax(prediction)
         if(prediction == 0):
            predictionText = "Forward"
         elif(prediction == 1):
            predictionText = "Right"
         elif(prediction == 2):
            predictionText = "No Speed Limit"
         elif(prediction == 3):
            predictionText = "Stop"
         print("Prediction: ", predictionText)
         cv2.putText(cv_image, predictionText, (50,200), cv2.FONT_ITALIC, 2, (255, 255, 255), 5, cv2.LINE_AA)
         self.image_received = 1 #Turn the flag on
         cv2.imshow("Image", cv_image)          
         cv2.waitKey(1)
      
      except CvBridgeError as e:
         print(e)    
     
            
   def cleanup(self):
      print("Done")
                 
############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("color_filter", anonymous=True)  
    ColorFilter()   
    rospy.spin()  

"""image = cv2.imread('home/angie/424.png')
image = image_utils.img_to_array(image)
image = image.reshape(1,250,250,3) 
prediction = new_model.predict(image)
print("Predict: ", np.argmax(prediction)) """
         