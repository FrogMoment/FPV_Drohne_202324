
# FPV Drohne Diplomarbeit 2023/24

This Project aims to build a flight controller software for a custom drone for my diploma project.

The whole flight controller gets realized on the STM32H7A3RGT6 microcontroller.




## Features

- pitch, roll, yaw measurement with MPU9250 + AK8963
- battery voltage measurement with DS2438
- height approximation with BMP280
- I.Bus / S.Bus input from controller
- ESC / motor control via DShot protocol


## Datasheets

all datasheets can be found in the folder [Datenblätter](https://github.com/FrogMoment/FPV_Drohne_202324/tree/master/Datenbl%C3%A4tter)


## Used Software

- Keil µVision 5
- STM32CubeMX
- Visual Studio Code

## other parts of project

 - [App development](https://github.com/LyffLyff/FPV-Drohne)

