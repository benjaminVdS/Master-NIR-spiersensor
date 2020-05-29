################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include emlib/subdir.mk
-include dbprint/subdir.mk
-include CMSIS/EFM32HG/subdir.mk
-include BSP/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: SLSTK3400A_blink_3.axf

# Tool invocations
SLSTK3400A_blink_3.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m0plus -mthumb -T "SLSTK3400A_blink_3.ld" -Xlinker --gc-sections -Xlinker -Map="SLSTK3400A_blink_3.map" --specs=nano.specs -o SLSTK3400A_blink_3.axf "./BSP/bsp_bcc.o" "./BSP/bsp_stk.o" "./BSP/bsp_stk_leds.o" "./CMSIS/EFM32HG/startup_efm32hg.o" "./CMSIS/EFM32HG/system_efm32hg.o" "./dbprint/dbprint.o" "./emlib/em_acmp.o" "./emlib/em_adc.o" "./emlib/em_aes.o" "./emlib/em_assert.o" "./emlib/em_burtc.o" "./emlib/em_can.o" "./emlib/em_cmu.o" "./emlib/em_core.o" "./emlib/em_cryotimer.o" "./emlib/em_crypto.o" "./emlib/em_csen.o" "./emlib/em_dac.o" "./emlib/em_dbg.o" "./emlib/em_dma.o" "./emlib/em_ebi.o" "./emlib/em_emu.o" "./emlib/em_gpcrc.o" "./emlib/em_gpio.o" "./emlib/em_i2c.o" "./emlib/em_iadc.o" "./emlib/em_idac.o" "./emlib/em_int.o" "./emlib/em_lcd.o" "./emlib/em_ldma.o" "./emlib/em_lesense.o" "./emlib/em_letimer.o" "./emlib/em_leuart.o" "./emlib/em_mpu.o" "./emlib/em_msc.o" "./emlib/em_opamp.o" "./emlib/em_pcnt.o" "./emlib/em_pdm.o" "./emlib/em_prs.o" "./emlib/em_qspi.o" "./emlib/em_rmu.o" "./emlib/em_rtc.o" "./emlib/em_rtcc.o" "./emlib/em_se.o" "./emlib/em_system.o" "./emlib/em_timer.o" "./emlib/em_usart.o" "./emlib/em_vcmp.o" "./emlib/em_vdac.o" "./emlib/em_wdog.o" "./src/Con_UART.o" "./src/Ic2_Connection.o" "./src/PWM_LED.o" "./src/Proteus-II.o" "./src/dmactrl.o" "./src/dmadrv.o" "./src/i2cspm.o" "./src/main.o" -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: SLSTK3400A_blink_3.hex'
	arm-none-eabi-objcopy -O ihex "SLSTK3400A_blink_3.axf" "SLSTK3400A_blink_3.hex"
	@echo ' '

	@echo 'Building bin file: SLSTK3400A_blink_3.bin'
	arm-none-eabi-objcopy -O binary "SLSTK3400A_blink_3.axf" "SLSTK3400A_blink_3.bin"
	@echo ' '

	@echo 'Building s37 file: SLSTK3400A_blink_3.s37'
	arm-none-eabi-objcopy -O srec "SLSTK3400A_blink_3.axf" "SLSTK3400A_blink_3.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "SLSTK3400A_blink_3.axf" -A
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) SLSTK3400A_blink_3.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
