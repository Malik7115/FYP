################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/DroneControl.cpp \
../src/DroneTracker.cpp \
../src/ImProcessing.cpp \
../src/RealSense.cpp \
../src/SerialComm.cpp \
../src/Thrust.cpp \
../src/Tracker.cpp \
../src/VidProc.cpp 

OBJS += \
./src/DroneControl.o \
./src/DroneTracker.o \
./src/ImProcessing.o \
./src/RealSense.o \
./src/SerialComm.o \
./src/Thrust.o \
./src/Tracker.o \
./src/VidProc.o 

CPP_DEPS += \
./src/DroneControl.d \
./src/DroneTracker.d \
./src/ImProcessing.d \
./src/RealSense.d \
./src/SerialComm.d \
./src/Thrust.d \
./src/Tracker.d \
./src/VidProc.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include/opencv4 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


