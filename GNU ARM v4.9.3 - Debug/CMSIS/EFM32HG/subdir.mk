################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5/platform/Device/SiliconLabs/EFM32HG/Source/system_efm32hg.c 

S_UPPER_SRCS += \
C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5/platform/Device/SiliconLabs/EFM32HG/Source/GCC/startup_efm32hg.S 

OBJS += \
./CMSIS/EFM32HG/startup_efm32hg.o \
./CMSIS/EFM32HG/system_efm32hg.o 

C_DEPS += \
./CMSIS/EFM32HG/system_efm32hg.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/EFM32HG/startup_efm32hg.o: C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5/platform/Device/SiliconLabs/EFM32HG/Source/GCC/startup_efm32hg.S
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Assembler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m0plus -mthumb -c -x assembler-with-cpp -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/drivers" -I"D:\Users\Benjamin\SimplicityStudio\v4_workspace\SLSTK3400A_blink_3\dbprint" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/SLSTK3400A_EFM32HG/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/bsp" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/Device/SiliconLabs/EFM32HG/Include" '-DEFM32HG322F64=1' -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

CMSIS/EFM32HG/system_efm32hg.o: C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5/platform/Device/SiliconLabs/EFM32HG/Source/system_efm32hg.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m0plus -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFM32HG322F64=1' -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/drivers" -I"D:\Users\Benjamin\SimplicityStudio\v4_workspace\SLSTK3400A_blink_3\dbprint" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/dmadrv/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/common/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emdrv/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/SLSTK3400A_EFM32HG/config" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//hardware/kit/common/bsp" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/emlib/inc" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/CMSIS/Include" -I"C:/SiliconLabs/SimplicityStudio/v4_2_3_4/developer/sdks/gecko_sdk_suite/v2.5//platform/Device/SiliconLabs/EFM32HG/Include" -O0 -Wall -c -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -MMD -MP -MF"CMSIS/EFM32HG/system_efm32hg.d" -MT"CMSIS/EFM32HG/system_efm32hg.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


