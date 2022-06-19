#!/usr/bin/env python  
from colorsys import hsv_to_rgb
import rospy  
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError 
import cv2
import numpy as np 
from matplotlib import pyplot as plt
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32 
import time



#This class will receive a ROS image and transform it to opencv format  

class ColorFilter():  

    def __init__(self):  

        rospy.on_shutdown(self.cleanup) 

        ############ CONSTANTS ################  

        self.bridge = CvBridge() # create the cv_bridge object 
        
     #   self.keypoints = self.detector.detect()

        self.image_received = 0 #Flag to indicate that we have already received an image 
        
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10) 
        
        self.rate = rospy.Rate(10) # 10hz delay
        
       #self.robot_vel = Twist()
        
        self.moving = 0
        self.vl=0
        self.vw=0
        self.kp=0.00005#0.0001
        self.ki=0.00005
        self.kd=0.00006 #Ganancia
        self.kt = 0.4
      #  self.xt = 0
        self.r=0.05 #wheel radius [m] 
        self.L=0.19 #wheel separation [m] 
        self.d=0 # trsaveled distance  
        self.theta=0 #angle 
        self.x = 0
        self.y = 0
        self.vel=Twist() #Robot's desired speed 
        self.wr=0 
        self.wl=0 
        self.band = 0
        self.arr = []
        self.img1 = cv2.imread("sftp://puzzlebot@10.42.0.1/home/puzzlebot/catkin_ws/src/beginner_tutorials/lena_opencv_red.jpg") 
        self.error_actual=0
        self.error_anterior=0
        self.error_d=0
        self.error_i=0
        self.error_p=0
        self.P=0
        self.I=0
        self.D=0

        
        


        ############################### SUBSCRIBERS #####################################  

        image_sub = rospy.Subscriber("/video_source/raw", Image, self.image_cb)
        
       
        

        #********** INIT NODE **********###  

        #self.r = rospy.Rate(10) #10Hz  
        
       # freq=50
        #rate = rospy.Rate(freq) #20Hz  
        self.dt =1/50.0
        

               
        while not rospy.is_shutdown(): #Mientras rospy no este apagadp
            

           # self.pub.publish(self.vel) #Publicando 

            self.rate.sleep()
            
            freq=20
            rate = rospy.Rate(freq) #50Hz  
            self.dt =1/20.0
            sumOfColumns = np.sum(self.img1, axis=0, dtype='float')
            picos=np.diff(sumOfColumns, n=1)
            picos2 = picos.copy()

            picos[picos<100]=0
            picos2[picos2>-100]=0
            picos1 = (picos2+picos)
            x=np.argmin(picos1)
            x1=np.argmax(picos1)
            follow = (x + x1)/2
            print(x)
            print(x1)
            print(follow)

            index = np.argmin(sumOfColumns)

           #plt.figure(figsize=(20,10))
          # plt.plot(picos2)
          # plt.show()
          # cv2.waitKey(1)
          # print(index)
            if index>0:
           #if index<=1200 and index>=380: #700

               self.error_actual=(275-follow)
 
               self.error_p = self.error_actual #450 250

               self.error_d= (self.error_actual-self.error_anterior)/self.dt

               self.error_i= self.error_actual*self.dt+self.error_anterior

               self.P=self. kp*self.error_actual

               self.I=self. ki*self.error_i

               self.D=self. kd*self.error_d
              
               self.vw = self.P+ self.I + self.D

               print(self.vw)
               
               self.vl = 0.03 #0.01
               
               self.vel.linear.x = self.vl
            
               self.vel.angular.z = self.vw
            
               self.pub.publish(self.vel)

               self.error_anterior=self.error_actual
               
               print(self.vel.angular.z)
               if index <= 325: #900
                print("Izquierda")
               elif index > 325: #900
                print("Derecha")

               
               
               #cv2.imshow("img", img)
               #cv2.waitKey(1) 
            else:
              self.vel.linear.x = 0
            
              self.vel.angular.z = 0



  
    def image_cb(self, ros_image):  
    
        ## This function receives a ROS image and transforms it into opencv format   

        try:
           cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
           print("received ROS image, I will convert it to opencv")
           self.image_received = 1 #Turn the flag on
           self.img=cv_image[600:720,365:915] #600:720,50:700       
           hsv = cv2.GaussianBlur(self.img,(5,5),0)
           kernel = np.ones((5, 5), 'uint8')
           dilate_img = cv2.dilate(hsv, kernel, iterations=1)
           kernel = np.ones((6, 6), np.uint8)
           #erode_image = cv2.erode(dilate_img, kernel, cv2.BORDER_REFLECT) 
           #self.img1 = cv2.cvtColor(erode_image, cv2.COLOR_BGR2GRAY)
           self.img1 = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
           
         
           #cv2.imshow("Image", self.img1)          
           #cv2.waitKey(1)


           #############################################################################################

           

           
         # indice m,enos mitad de la imagen 
         # centro menos linea
         
   

        except CvBridgeError as e:
           print(e)      
           
            
    def cleanup(self):
     
        print("Done")
        self.vel.linear.x = 0
        
        self.vel.angular.z = 0
        
        self.pub.publish(self.vel)          

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("color_filter", anonymous=True)  
    ColorFilter()   
    rospy.spin()
