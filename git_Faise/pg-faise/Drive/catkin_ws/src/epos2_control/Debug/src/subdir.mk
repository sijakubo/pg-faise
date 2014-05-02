################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Epos2MotorController\ _OLD.cpp \
../src/Epos2MotorController.cpp \
../src/SampleMoving.cpp \
../src/TankSteering.cpp \
../src/epos2_driver.cpp \
../src/epos2_talker.cpp 

OBJS += \
./src/Epos2MotorController\ _OLD.o \
./src/Epos2MotorController.o \
./src/SampleMoving.o \
./src/TankSteering.o \
./src/epos2_driver.o \
./src/epos2_talker.o 

CPP_DEPS += \
./src/Epos2MotorController\ _OLD.d \
./src/Epos2MotorController.d \
./src/SampleMoving.d \
./src/TankSteering.d \
./src/epos2_driver.d \
./src/epos2_talker.d 


# Each subdirectory must supply rules for building sources it contributes
src/Epos2MotorController\ _OLD.o: ../src/Epos2MotorController\ _OLD.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I/opt/ros/groovy/include -I/home/eos/catkin_ws/devel/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"src/Epos2MotorController _OLD.d" -MT"src/Epos2MotorController\ _OLD.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I/opt/ros/groovy/include -I/home/eos/catkin_ws/devel/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


