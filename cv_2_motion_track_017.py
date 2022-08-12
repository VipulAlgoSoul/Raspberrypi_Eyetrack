from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import RPi.GPIO as gpio

#without motion reduction


gpio.setmode(gpio.BCM)
ver_mot=22
ang_mot=12
gpio.setup(ver_mot, gpio.OUT)
gpio.setup(ang_mot, gpio.OUT)

#x=input("C")
ser_ver = gpio.PWM(ver_mot,100)
ser_ang = gpio.PWM(ang_mot,100)
#x=input("C")

ser_ver.start(12)
ser_ang.start(13)

#row_size= 120#240 #480/2
#col_size=160 #320 #640/2
#row_size= 240 #480/2
#col_size=320 #640/2
#x=input("C")
row_size=480
col_size=640
camera = PiCamera()
camera.resolution =(col_size,row_size) #480 row, #640 col
camera.framerate= 30

rawCapture = PiRGBArray(camera, size=(col_size,row_size))

time.sleep(0.1)
prvs_frame=np.zeros((row_size,col_size), dtype=int)
new_mat=np.zeros((row_size,col_size),dtype=int)

total_no_of_segments=64
correction_threshold=20
correction_threshold_number=0

slice_x=int(row_size/np.sqrt(total_no_of_segments))
slice_y=int(col_size/np.sqrt(total_no_of_segments))
row_for = np.arange(0,row_size,slice_x)
col_for = np.arange(0,col_size,slice_y)

#print(row_for)
#print(col_for)
#print(row_for.dtype)
#print(col_for.dtype)
#x=input("C")
x_position=0
y_position=0

def angle_Segment():
    global slice_x
    global slice_y
    global row_for
    global col_for
    global row_size
    global col_size
    global new_mat1
    global x_position
    global y_position

    numx=0
    numy=0

    val=0
    prvs_val=0
    prvs_sector =0
    compute_full=0
    #print("angle segment")

    for x in row_for:
        numy=0
       # print("row_iteration",numx)
      #  print("previous val",prvs_val,"....",x_position,",",y_position,"###############")
        #input("enter")

        if(x+slice_x)>row_size:
            break

        for y in col_for:
            
            numy =numy+1
            #print("column iterartion",numy)
            if(y+slice_y)>col_size:
                break
            val=sum(sum(new_mat1[x:x+slice_x,y:y+slice_y]))/240
            compute_full=compute_full+val
            #print("val is ",val)
            #print(val.dtype)
            sector=(numx*(col_size/slice_y))+numy

            if val>prvs_val:
     #           print("prvs val_changed")
        
                x_position= int(x+slice_x/2)
                y_position=int(y+slice_y/2) 
                #print(numx)
                prvs_val=val
                prvs_sector=sector
    #        print("value is",val,"...position=>",x,",",y)

        numx=numx+1
    return x_position, y_position,prvs_val, prvs_sector,compute_full

################################  MAIN  ##################
def servo_error_motion(x_pos,y_pos):
    global row_size
    global col_size
    global correction_threshold
    global correction_threshold_number
    #x_error=0
    #y_error = 0
    #print("servo motion")

    x_error=(col_size/2)-y_pos
    y_error = (row_size/2)-x_pos

    if((x_error>0) & (y_error>0)):
        #print("OBJECT 3D RIGHT TOP")
        ser_ver.ChangeDutyCycle(12+(y_error/48))
        ser_ang.ChangeDutyCycle(13-(x_error/64))
        
    if((x_error<0) & (y_error>0)):
        #print("OBJECT 3D LEFT TOP")
        ser_ver.ChangeDutyCycle(12+(y_error/48))
        ser_ang.ChangeDutyCycle(13-(x_error/64))
        
    if((x_error>0) & (y_error<0)):

        #print("OBJECT 3D RIGHT BOTTOM")
        ser_ver.ChangeDutyCycle(12+(y_error/48))
        ser_ang.ChangeDutyCycle(13+(x_error/64))
        
    if((x_error<0) & (y_error<0)):

        #print("OBJECT 3D LEFT BOTTOM")
        ser_ver.ChangeDutyCycle(12+(y_error/48))
        ser_ang.ChangeDutyCycle(13+(x_error/64))
    
    
    correction_threshold=50
    correction_threshold_number=0
######################################


print("start")

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    

    image_c= frame.array
    image_gray= cv2.cvtColor(image_c,cv2.COLOR_BGR2GRAY)
    blur_gray= cv2.GaussianBlur(image_gray,(5,5),0)
    #time_diff= prvs_frame-blur_gray
    time_diff= blur_gray-prvs_frame
    #########
    #single derivative
    #new_mat=(time_diff<250)*240
    #new_mat = new_mat.astype(np.uint8)
    #new_mat1 =cv2.Canny(new_mat,100,200)
    #new_mat1 = new_mat1.astype(np.uint8)
    ##############
    #double derivative
    
    new_mat=((time_diff-new_mat)<250)*240
    print("0",new_mat.dtype)
    new_mat = new_mat.astype(np.uint8)
    new_mat1 =cv2.Canny(new_mat,100,200)
    print("1",new_mat1.dtype)
    new_mat1 = new_mat1.astype(np.uint8)
    print("2",new_mat1.dtype)
##########
    #without canny
    #new_mat=((time_diff-new_mat)<250)*240
    #new_mat = new_mat.astype(np.uint8)
    #new_mat1=new_mat


    #####################
    time_diff = time_diff.astype(np.uint8)
    blur= cv2.GaussianBlur(time_diff,(5,5),0)
    #print(new_mat1.dtype)

   # raw_slice=np.arange(0,120,15)
    #col_slice=np.arange(0,160,20)

    
    x_po,y_po,max_val,sector_num,full_pix=angle_Segment()
    #cv2.rectangle(new_mat1,((x_po-int(slice_x/2)),(y_po-int(slice_y/2))),((x_po+int(slice_x/2)),(y_po+int(slice_y/2))),(255,0,0),2)
    #cv2.rectangle(new_mat1,((x_po,y_po)),((x_po+int(slice_x/2)),(y_po+int(slice_y/2))),(255,0,0),2)
    cv2.rectangle(new_mat1,((y_po,x_po)),((y_po+int(slice_y)),(x_po+int(slice_x))),(255,0,0),2)
    cv2.imshow("frame", new_mat1)
    #print("max val",max_val)
    #print("total",full_pix)
    #print(full_pix,">>total sum,",max_val,">>maximum in sector,",full_pix/(max_val+0.01),">>ratio ")
    #print("sector --",sector_num,".....","position",x_po,",",y_po)
    ratio_variance= full_pix/(max_val+0.01)
    #print("ratio_variance",ratio_variance)
    if((ratio_variance)>6.5):
        #correction ratio =4
        #if correction threshold greater than 0 then the motion is 0
        servo_error_motion(x_po,y_po)
    else:
        #print("0,0")
        ser_ver.ChangeDutyCycle(12)
        ser_ang.ChangeDutyCycle(13)
        correction_threshold_number=correction_threshold_number+1
        if correction_threshold_number>15:
            correction_threshold=25
        else:
            correction_threshold=40

    #print("Correction threshold",correction_threshold)
    #print("corr_number",correction_threshold_number)
    #servo_error_motion(x_po,y_po)
    #print(x_po,",",y_po)
    new_mat=new_mat
    prvs_frame= blur_gray
    key = cv2.waitKey(1)
    rawCapture.truncate(0)
    #input("enter c")

    if key==ord("q"):
        break
