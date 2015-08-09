################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := E:/Code Composer Studio(CCS)/CCS/ccsv6/tools/compiler/ti-cgt-msp430_4.4.3

ORDERED_OBJS += \
"./CtlFourAxis.obj" \
"./sto_int/sto_int.obj" \
"./msp430_clock/msp430_clock.obj" \
"./inv_mpu/inv_mpu.obj" \
"./int_clk/int_clk.obj" \
"./i2c/i2c.obj" \
"./dmp_driver/inv_mpu_dmp_motion_driver.obj" \
"./bluth/bluth.obj" \
"./MSP430_IIC/MSP430_IIC.obj" \
"./HCM5886/HCM5886.obj" \
"./ESP6288/ESP6288.obj" \
"./Display1/Display1.obj" \
"../MSP430_IIC/lnk_msp430f5529.cmd" \
$(GEN_CMDS__FLAG) \
-l"libmath.a" \
-l"libc.a" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include sto_int/subdir_vars.mk
-include msp430_clock/subdir_vars.mk
-include inv_mpu/subdir_vars.mk
-include int_clk/subdir_vars.mk
-include i2c/subdir_vars.mk
-include dmp_driver/subdir_vars.mk
-include bluth/subdir_vars.mk
-include MSP430_IIC/subdir_vars.mk
-include HCM5886/subdir_vars.mk
-include ESP6288/subdir_vars.mk
-include Display1/subdir_vars.mk
-include subdir_rules.mk
-include sto_int/subdir_rules.mk
-include msp430_clock/subdir_rules.mk
-include inv_mpu/subdir_rules.mk
-include int_clk/subdir_rules.mk
-include i2c/subdir_rules.mk
-include dmp_driver/subdir_rules.mk
-include bluth/subdir_rules.mk
-include MSP430_IIC/subdir_rules.mk
-include HCM5886/subdir_rules.mk
-include ESP6288/subdir_rules.mk
-include Display1/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
CtlFourAxis.out \

EXE_OUTPUTS__QUOTED += \
"CtlFourAxis.out" \


# All Target
all: CtlFourAxis.out

# Tool invocations
CtlFourAxis.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: MSP430 Linker'
	"E:/Code Composer Studio(CCS)/CCS/ccsv6/tools/compiler/ti-cgt-msp430_4.4.3/bin/cl430" -vmspx --abi=coffabi --data_model=restricted --advice:power=all -g --gcc --define=__MSP430F5529__ --diag_warning=225 --display_error_number --diag_wrap=off --silicon_errata=CPU21 --silicon_errata=CPU22 --silicon_errata=CPU23 --silicon_errata=CPU40 --printf_support=minimal -z -m"CtlFourAxis.map" --heap_size=160 --stack_size=160 --use_hw_mpy=F5 --cinit_hold_wdt=on -i"E:/Code Composer Studio(CCS)/CCS/ccsv6/ccs_base/msp430/include" -i"E:/Code Composer Studio(CCS)/CCS/ccsv6/tools/compiler/ti-cgt-msp430_4.4.3/lib" -i"E:/Code Composer Studio(CCS)/CCS/ccsv6/tools/compiler/ti-cgt-msp430_4.4.3/include" -i"E:/Code Composer Studio(CCS)/CCS/ccsv6/ccs_base/msp430/lib" -i"E:/Code Composer Studio(CCS)/CCS/ccsv6/ccs_base/msp430/lib/5xx_6xx_FRxx" --reread_libs --warn_sections --display_error_number --diag_wrap=off --xml_link_info="CtlFourAxis_linkInfo.xml" --rom_model -o "CtlFourAxis.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)
	-$(RM) "CtlFourAxis.pp" "sto_int\sto_int.pp" "msp430_clock\msp430_clock.pp" "inv_mpu\inv_mpu.pp" "int_clk\int_clk.pp" "i2c\i2c.pp" "dmp_driver\inv_mpu_dmp_motion_driver.pp" "bluth\bluth.pp" "MSP430_IIC\MSP430_IIC.pp" "ESP6288\ESP6288.pp" "Display1\Display1.pp" 
	-$(RM) "HCM5886\HCM5886.pp" 
	-$(RM) "CtlFourAxis.obj" "sto_int\sto_int.obj" "msp430_clock\msp430_clock.obj" "inv_mpu\inv_mpu.obj" "int_clk\int_clk.obj" "i2c\i2c.obj" "dmp_driver\inv_mpu_dmp_motion_driver.obj" "bluth\bluth.obj" "MSP430_IIC\MSP430_IIC.obj" "HCM5886\HCM5886.obj" "ESP6288\ESP6288.obj" "Display1\Display1.obj" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

