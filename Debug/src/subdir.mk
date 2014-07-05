################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/BlinkLed.cpp \
../src/CommonException_handlers.cpp \
../src/main.cpp 

C_SRCS += \
../src/_initialize_hardware.c 

OBJS += \
./src/BlinkLed.o \
./src/CommonException_handlers.o \
./src/_initialize_hardware.o \
./src/main.o 

C_DEPS += \
./src/_initialize_hardware.d 

CPP_DEPS += \
./src/BlinkLed.d \
./src/CommonException_handlers.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wpadded -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal  -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_ITM -DSTM32F407xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f4-hal" -std=gnu++11 -fabi-version=3 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -Wabi -Wctor-dtor-privacy -Wnoexcept -Wnon-virtual-dtor -Wstrict-null-sentinel -Wsign-promo -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wpadded -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal  -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_ITM -DSTM32F407xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f4-hal" -std=gnu11 -Wmissing-prototypes -Wstrict-prototypes -Wbad-function-cast -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


