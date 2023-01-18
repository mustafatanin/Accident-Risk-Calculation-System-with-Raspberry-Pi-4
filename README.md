# Accident-Risk-Calculation-System

• SUMMARY
Firstly, we aimed to create a system that calculates the risk of accident by processing data obtained from various sensors. To do this, we used 6 sensors/modules and one camera (webcam). In addition to the sensors/modules we used to collect data, we used a programmable RGB LED to convert the risk data we obtained into a visual output, and we used a buzzer to receive audio alerts. We created the main schematic of the project using these components.

• INTRODUCTION: What I wanted to do
Here, what we primarily aimed to do was to design something that had not been done before. Instead of making a project that you could easily do by obtaining ready-made source codes on the internet, we aimed to produce an original product. For example, the GPS module we used is not even a module produced for Raspberry, but we were able to make it work and calculate speed despite the limited resources we found on the internet for this sensor produced for Arduino. In addition to this, to explain the main goal of the project, we aimed to calculate the risk and warn the driver using 6 sensors/modules and one camera (Webcam) in our project. While doing this, we also aimed to adapt the project completely to real life, which is why we applied the project on a real car instead of any model or mock-up. We didn't find it correct to implement the project on any mock-up or model. We tried to produce a product that could be used as much as possible in real life.

• APPLICATION: What did you do, with what, and how...
First of all, let's start by listing all the sensors and modules we used:

• Gy-NEO6MV2 GPS Module - Flight Control System
Using this module, we aimed to calculate the speed data by calculating the distance covered per unit time using satellite data. However, we were unable to make the module work for a long time due to the fact that the module we received from the place we bought it was defective. After further investigations, we decided that the module was indeed defective. Then, we completed the process by soldering the headers of the new GPS module we obtained. With this, we were able to measure speed, and we used this speed data in the algorithm we created for risk calculation.
• ADXL345 3-Axis Accelerometer
With this sensor, we calculated the instantaneous G-force. We calculated the G-force in the X and Y directions to calculate sudden braking and lateral tilting of the car in the vertical and horizontal axis. This way, when the car takes an action that would cause sudden risk, the risk output becomes affected.
• HC-SR04 Ultrasonic Distance Sensor
We used two of these sensors to measure the following distance of the car in front, and by increasing the risk output when the distance is too close. The reason we used two of these sensors is that because the front of the car is large, and because the sensor is designed for use in more minimalist environments, we preferred to use two sensors to get more accurate data.
• Sound Sensor Card
We used this sensor for a simple reason, we thought that a high-noise environment inside the car would distract the driver, so we used this sensor to measure the sound in the environment, especially music, in our risk calculation.
• Light Sensor Card
We used this sensor because we thought that if the opposing car uses high beams, the driver will have difficulty seeing the road, which will increase the risk. For this reason, we also used the light sensor in our risk output.
• Rain Sensor Card
We used this sensor because when it rains, the ground will become wet, and the car's grip on the road will decrease, which will increase the risk.
• Webcam
We used a webcam to take a picture of the driver's face and check if the driver is drowsy, which would also increase the risk.
We also used an RGB LED, a buzzer, and an breadboard to control the whole system.

• CONCLUSION: Were you successful?
Of course, conducting long scientific studies is necessary to calculate the correct risk data. And considering that there is an expert in each field, it is not very likely for us to do it ourselves, as we do not have expertise in this area. Our goal here was to develop a prototype and convey the general working logic of the system to the project owner. In this regard, we believe that we were extremely successful.

