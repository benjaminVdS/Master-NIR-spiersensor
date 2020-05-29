################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/bsp_bcc.c \
../BSP/bsp_stk.c \
C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5/hardware/kit/common/bsp/bsp_stk_leds.c 

OBJS += \
./BSP/bsp_bcc.o \
./BSP/bsp_stk.o \
./BSP/bsp_stk_leds.o 

C_DEPS += \
./BSP/bsp_bcc.d \
./BSP/bsp_stk.d \
./BSP/bsp_stk_leds.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/bsp_bcc.o: ../BSP/bsp_bcc.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m0plus -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFM32HG322F64=1' -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/drivers" -I"D:\Users\Benjamin\SimplicityStudio\v4_workspace\SLSTK3400A_blink_3\dbprint" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/SLSTK3400A_EFM32HG/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/bsp" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/Device/SiliconLabs/EFM32HG/Include" -O0 -Wall -c -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -MMD -MP -MF"BSP/bsp_bcc.d" -MT"BSP/bsp_bcc.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

BSP/bsp_stk.o: ../BSP/bsp_stk.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m0plus -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFM32HG322F64=1' -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/drivers" -I"D:\Users\Benjamin\SimplicityStudio\v4_workspace\SLSTK3400A_blink_3\dbprint" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/SLSTK3400A_EFM32HG/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/bsp" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/Device/SiliconLabs/EFM32HG/Include" -O0 -Wall -c -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -MMD -MP -MF"BSP/bsp_stk.d" -MT"BSP/bsp_stk.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

BSP/bsp_stk_leds.o: C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5/hardware/kit/common/bsp/bsp_stk_leds.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m0plus -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFM32HG322F64=1' -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/drivers" -I"D:\Users\Benjamin\SimplicityStudio\v4_workspace\SLSTK3400A_blink_3\dbprint" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/SLSTK3400A_EFM32HG/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/bsp" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/Device/SiliconLabs/EFM32HG/Include" -O0 -Wall -c -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -MMD -MP -MF"BSP/bsp_stk_leds.d" -MT"BSP/bsp_stk_leds.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


