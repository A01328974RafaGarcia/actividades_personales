"""camera_pid controller."""

from controller import Display, Keyboard, Robot, Camera
from vehicle import Car, Driver
import numpy as np
import cv2
from datetime import datetime
import os

#Getting image from camera
def get_image(camera):
    raw_image = camera.getImage()
    image = np.frombuffer(raw_image, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )
    return image

#Image processing
def greyscale_cv2(image):
    gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 
    return gray_img

# Aplicación de la transformada de Hough
#Usamos blur para suavizar en la escala de pixeles de 3x3. 
#La parametrización de Canny en 40 y 120 fueron asumidos como los pixeles considerados como bordes.
def hough_transform(gray_img,image):
    img_blur = cv2.GaussianBlur(gray_img,(3,3),0,0) 
    img_canny = cv2.Canny(img_blur,40,100)

    #Definiendo la Región de Interés (ROI):
    vertices = np.array([[(0,200),(0,35),(120,35),(120,200)]],dtype=np.int32) #Area de interés
    img_roi = np.zeros_like(gray_img)
    cv2.fillPoly(img_roi,vertices,255)
    img_mask = cv2.bitwise_and(img_canny,img_roi)

    #Definición del número de líneas de la Transformada de Hough:
    #Importante tener en cuenta que las líneas detectadas se almacenarán en la variable lines.
    #Se crea una imagen en blanco (img_lines) del mismo tamaño que la imagen binaria original (img_mask). 
    #img_mask, se utilizará para dibujar las líneas detectadas
    rho = 2 #Acumulador de 2 pixeles
    theta = np.pi/180 # Aquí se consideran todas las posibles direcciones  
    threshold = 45 # Umbral aceptado 
    min_line_len = 25 # Mínimo valor aceptado de líneas
    max_line_gap = 20 # Máximo valor aceptado de líneas
    lines = cv2.HoughLinesP(img_mask,rho,theta,threshold,np.array([]),minLineLength=min_line_len,maxLineGap=max_line_gap)
    img_lines = np.zeros((img_mask.shape[0],img_mask.shape[1],1),dtype=np.uint8)
    rotation_angle = 0
    if lines is None:
        lines = []
    for line in lines:

        for x1,y1,x2,y2 in line:
            deltaY = y2 - y1
            deltaX = x2 - x1
            atan = np.arctan(deltaY/deltaX)
            rotation_angle = -atan

            cv2.line(img_lines,(x1,y1),(x2,y2),[255,255,0],1)
            break

    set_steering_angle(rotation_angle)
    return img_mask


#Display image
def display_image(display, image):
    # Image to display
    image_rgb = np.dstack((image, image,image,))
    # Display image
    image_ref = display.imageNew(
        image_rgb.tobytes(),
        Display.RGB,
        width=image_rgb.shape[1],
        height=image_rgb.shape[0],
    )
    display.imagePaste(image_ref, 0, 0, False)

#initial angle and speed
manual_steering = 0
steering_angle = 0
angle = 0.0
speed = 40

# set target speed
def set_speed(kmh):
    global speed            #robot.step(50)
#update steering angle
def set_steering_angle(wheel_angle):
    global angle, steering_angle
    # Check limits of steering
    if (wheel_angle - steering_angle) > 0.1:
        wheel_angle = steering_angle + 0.1
    if (wheel_angle - steering_angle) < -0.1:
        wheel_angle = steering_angle - 0.1
    steering_angle = wheel_angle

    # limit range of the steering angle
    if wheel_angle > 0.5:
        wheel_angle = 0.5
    elif wheel_angle < -0.5:
        wheel_angle = -0.5
    # update steering angle
    angle = wheel_angle

#validate increment of steering angle
def change_steer_angle(inc):
    global manual_steering
    # Apply increment
    new_manual_steering = manual_steering + inc
    # Validate interval
    if new_manual_steering <= 25.0 and new_manual_steering >= -25.0:
        manual_steering = new_manual_steering
        set_steering_angle(manual_steering * 0.02)
    # Debugging
    if manual_steering == 0:
        print("going straight")
    else:
        turn = "left" if steering_angle < 0 else "right"
        print("turning {} rad {}".format(str(steering_angle),turn))

# main
def main():
    # Create the Robot instance.
    robot = Car()
    driver = Driver()

    # Get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())

    # Create camera instance
    camera = robot.getDevice("camera")
    camera.enable(timestep)  # timestep

    # processing display
    display_img = Display("display_image")

    #create keyboard instance
    keyboard=Keyboard()
    keyboard.enable(timestep)

    while robot.step() != -1:
        # Get image from camera
        image = get_image(camera)

        # Process and display image
        gray_img = greyscale_cv2(image)

        # Transformada de Hough
        hough_image = hough_transform(gray_img,image)
        
        # Display images
        display_image(display_img, hough_image)

        #update angle and speed
        driver.setSteeringAngle(angle)
        driver.setCruisingSpeed(speed)


if __name__ == "__main__":
    main()