# Engineering materials

This repository contains engineering materials of "yatta-team (Leteam)" self-driven vehicle's model participating in the WRO Future Engineers competition in the season 2024.

## Content

-   `t-photos` contains photos of the team
-   `v-photos` contains photos of the vehicle
-   `video` contains the video.md file with the link to a video where driving demonstration exists
-   `schemes` contains diagrams about the design and cricut
-   `src` contains the source code that is used
-   `chasis` contains photos of the used chasis and its parts

**AS OF THIS COMMIT IT ONLY CONTAINS THE SOURCE**

## Hardware

| HARDWARE PARTS      | References                                                                                                           |
| ------------------- | -------------------------------------------------------------------------------------------------------------------- |
| Chasis              | [4WDRC Chassis S3003 Servo](https://www.elecrow.com/4wd-smart-car-robot-chassis-for-arduino-servo-steering.html)     |
| Servo               | [S3003 FUTABA SERVO](https://www.es.co.th/schemetic/pdf/et-servo-s3003.pdf)                                          |
| Main-Controller     | [RPI-5](https://www.raspberrypi.com/products/raspberry-pi-5/)                                                |
| Secondry-Controller | [Arduino ATmega328p (Arduino Nano 3.x)](https://store.arduino.cc/products/arduino-nano)                              |
| gyroscope           | [BNO085](https://www.ceva-ip.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf) |
| Camera              | [usb 170 degree fisheye cam](https://www.amazon.com/ALPCAM-Camera-Module-Webcam-Fisheye/dp/B07VVH39DC/)                                                 |
| Other-Parts         | Ultrasonics, full-bridge, Voltage regulator, green-light, button                                                     |

# Software

We are using the rasberry-pi-4 as a the main controller // computer.
We also use the arduino ATmega328p to - read the Ultrasonic sensors -> filter the readings -> send them using the serial connection to rpi

## Code basic idea

We took advantage of our rasberry pi ability to run multiple processes at the same time so we devided the main program into seperate threads

### MAIN-THREAD

In the main thread we run all the other threads while waiting for the mpu6050 to calibrate, then the main thread waits for a button click after the calibration is finished, our main thread continous to start the car movment by calling the moving function and turning function when needed

---

When the MPU-THREAD finishes calibration and the current angle is set to 0, the main thread waits for a button click.

if the button is clicked the main thread starts the car moving function.

### MPU-THREAD

In the MPU thread we run a function that calculates all the possible angles and calibrates the gyroscope to the current position and then it runs a while loop the gets the current angle by storing it in a class attribute.

-   This thread is a BLOCKING thread, because it blocks the execution of the main thread and that is needed to make the gyroscope calibrate correctly since the gyro we have can not save old calibration like other gyroscopes.

-   The main thread is blocked by a while loop that waits for the current angle to change from -1 to anything else (in this case it should be 0 because after the calibration of the mpu6050 the first angle is set to 0)

-   This thread then keeps running and reading the new angle and setting it to the current angle

-   Events -> None

### DISTANCE-THREAD

In the DISTANCE thread we run a loop that reads the serial continuously, when the arduino finishes reading the ultrasonics, it sends the data throught the serial and the rasberry pi throught this thread it reads and updates the distances.

-   Events -> stop_event which is called at the exit of the program to free and make sure the memory is clean to help pythons garbage collector and stop any overflows

### COLOR-THREAD - direction detection

In the COLOR thread we start reading frames from the camera and do some python and opencv processing on the frames and check for orange and blue colors ... if blue is found then it sets the turn direction to 1 else if orange is found it sets the turn direction to -1 else if orange nor blue were found then turn based on the left or right distance

-> direction-detection chart:
process camera frames -> blue is found <-> direction = 1 <-> else if orange is found <-> direction = -1 <-> else orange nor blue were found -> read left and right distance -> if left is bigger -> direction = 1 <-> else if right is bigger <-> direction = -1

-   Events -> stop_event which stops the thread in a certain case is met and when the program exits

## Turning:

Using ultrasonic sensors (sonar sensors) we check for the right, left, and front distances then we use certain conditions in the moving function that tells the robot to turn based on the direction that was defined by the [COLOR-THREAD](#color-thread---direction-detection).

The turning mechanism uses the mpu6050 to read the current angle and the desired angle to make the robot turn 90 degrees (somewhat exactly ...)

## Avoiding Walls:

Using the right, left distances from the ultrasonics, we calculate a distance error based on a threshold value, then we use a PID algorithm and get a negative or positive return value based on the error ...

After that we use the desired angle (the heading angle e.g 90, 180, 270, 360 or (0)) and the current angle that the gyroscope is reading to calculate an angle error and with some maths we make the robot get the required servo angle to make it go in a 90 degree straight path

we sum the values e.g if the distance error was negative then it helps go away from the right wall, otherwise if positive it helps go away from the left wall, while keeping the angle straight

In the last code, we replaced the readings from ultrasonic sensors with the x values of the black color contours on the right and left sides of the cropped image.

## 3 LAPS detection

At first the robot only runs a for loop for each lap and a for loop inside of it for each corner, and runs the moving and turning functions inside, which works correctly but, we are aware its error prone, In the latest update, we handled each edge of the track with a dedicated function that identifies the end of the edge in the first round when it detects the black color in front of it. In the second round, it identifies the end by the color of the line on the floor, considering the possibility of seeing the black color on the sides while trying to avoid obstacles as the end of the path.
