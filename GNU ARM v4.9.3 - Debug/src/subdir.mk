################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/Con_UART.c \
../src/Ic2_Connection.c \
../src/PWM_LED.c \
../src/Proteus-II.c \
../src/dmactrl.c \
../src/dmadrv.c \
../src/i2cspm.c \
../src/main.c 

OBJS += \
./src/Con_UART.o \
./src/Ic2_Connection.o \
./src/PWM_LED.o \
./src/Proteus-II.o \
./src/dmactrl.o \
./src/dmadrv.o \
./src/i2cspm.o \
./src/main.o 

C_DEPS += \
./src/Con_UART.d \
./src/Ic2_Connection.d \
./src/PWM_LED.d \
./src/Proteus-II.d \
./src/dmactrl.d \
./src/dmadrv.d \
./src/i2cspm.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/Con_UART.o: ../src/Con_UART.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m0plus -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFM32HG322F64=1' -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/drivers" -I"D:\Users\Benjamin\SimplicityStudio\v4_workspace\SLSTK3400A_blink_3\dbprint" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/SLSTK3400A_EFM32HG/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/bsp" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/Device/SiliconLabs/EFM32HG/Include" -O0 -Wall -c -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -MMD -MP -MF"src/Con_UART.d" -MT"src/Con_UART.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/Ic2_Connection.o: ../src/Ic2_Connection.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m0plus -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFM32HG322F64=1' -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/drivers" -I"D:\Users\Benjamin\SimplicityStudio\v4_workspace\SLSTK3400A_blink_3\dbprint" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/SLSTK3400A_EFM32HG/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/bsp" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/Device/SiliconLabs/EFM32HG/Include" -O0 -Wall -c -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -MMD -MP -MF"src/Ic2_Connection.d" -MT"src/Ic2_Connection.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/PWM_LED.o: ../src/PWM_LED.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m0plus -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFM32HG322F64=1' -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/drivers" -I"D:\Users\Benjamin\SimplicityStudio\v4_workspace\SLSTK3400A_blink_3\dbprint" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/SLSTK3400A_EFM32HG/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/bsp" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/Device/SiliconLabs/EFM32HG/Include" -O0 -Wall -c -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -MMD -MP -MF"src/PWM_LED.d" -MT"src/PWM_LED.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/Proteus-II.o: ../src/Proteus-II.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m0plus -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFM32HG322F64=1' -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/drivers" -I"D:\Users\Benjamin\SimplicityStudio\v4_workspace\SLSTK3400A_blink_3\dbprint" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/SLSTK3400A_EFM32HG/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/bsp" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/Device/SiliconLabs/EFM32HG/Include" -O0 -Wall -c -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -MMD -MP -MF"src/Proteus-II.d" -MT"src/Proteus-II.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/dmactrl.o: ../src/dmactrl.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m0plus -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFM32HG322F64=1' -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/drivers" -I"D:\Users\Benjamin\SimplicityStudio\v4_workspace\SLSTK3400A_blink_3\dbprint" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/SLSTK3400A_EFM32HG/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/bsp" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/Device/SiliconLabs/EFM32HG/Include" -O0 -Wall -c -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -MMD -MP -MF"src/dmactrl.d" -MT"src/dmactrl.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/dmadrv.o: ../src/dmadrv.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m0plus -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFM32HG322F64=1' -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/drivers" -I"D:\Users\Benjamin\SimplicityStudio\v4_workspace\SLSTK3400A_blink_3\dbprint" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/SLSTK3400A_EFM32HG/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/bsp" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/Device/SiliconLabs/EFM32HG/Include" -O0 -Wall -c -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -MMD -MP -MF"src/dmadrv.d" -MT"src/dmadrv.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/i2cspm.o: ../src/i2cspm.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m0plus -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFM32HG322F64=1' -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/drivers" -I"D:\Users\Benjamin\SimplicityStudio\v4_workspace\SLSTK3400A_blink_3\dbprint" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/SLSTK3400A_EFM32HG/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/bsp" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/Device/SiliconLabs/EFM32HG/Include" -O0 -Wall -c -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -MMD -MP -MF"src/i2cspm.d" -MT"src/i2cspm.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/main.o: ../src/main.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m0plus -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFM32HG322F64=1' -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/drivers" -I"D:\Users\Benjamin\SimplicityStudio\v4_workspace\SLSTK3400A_blink_3\dbprint" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/SLSTK3400A_EFM32HG/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/bsp" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/Device/SiliconLabs/EFM32HG/Include" -O0 -Wall -c -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -MMD -MP -MF"src/main.d" -MT"src/main.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


