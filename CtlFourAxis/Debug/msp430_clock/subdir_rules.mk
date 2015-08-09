################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
msp430_clock/msp430_clock.obj: ../msp430_clock/msp430_clock.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP430 Compiler'
	"E:/Code Composer Studio(CCS)/CCS/ccsv6/tools/compiler/ti-cgt-msp430_4.4.3/bin/cl430" -vmspx --abi=coffabi --data_model=restricted --include_path="E:/Code Composer Studio(CCS)/CCS/ccsv6/ccs_base/msp430/include" --include_path="D:/LargeFourAxis/LargeFourAxis/CtlFourAxis/bluth" --include_path="C:/Users/Administrator/Desktop/electronic competition/LargeFourAxis/HCM5883" --include_path="C:/Users/panfeng/Desktop/Fly_Me/LargeFourAxis/LargeFourAxis/CtlFourAxis" --include_path="C:/Users/panfeng/Desktop/Fly_Me/LargeFourAxis/LargeFourAxis/CtlFourAxis/Display1" --include_path="C:/Users/panfeng/Desktop/Fly_Me/LargeFourAxis/LargeFourAxis/CtlFourAxis/dmp_driver" --include_path="C:/Users/panfeng/Desktop/Fly_Me/LargeFourAxis/LargeFourAxis/CtlFourAxis/dmpKey" --include_path="C:/Users/panfeng/Desktop/Fly_Me/LargeFourAxis/LargeFourAxis/CtlFourAxis/dmpmap" --include_path="C:/Users/panfeng/Desktop/Fly_Me/LargeFourAxis/LargeFourAxis/CtlFourAxis/ESP6288" --include_path="C:/Users/panfeng/Desktop/Fly_Me/LargeFourAxis/LargeFourAxis/CtlFourAxis/i2c" --include_path="C:/Users/panfeng/Desktop/Fly_Me/LargeFourAxis/LargeFourAxis/CtlFourAxis/int_clk" --include_path="C:/Users/panfeng/Desktop/Fly_Me/LargeFourAxis/LargeFourAxis/CtlFourAxis/inv_mpu" --include_path="C:/Users/panfeng/Desktop/Fly_Me/LargeFourAxis/LargeFourAxis/CtlFourAxis/msp430_clock" --include_path="C:/Users/panfeng/Desktop/Fly_Me/LargeFourAxis/LargeFourAxis/CtlFourAxis/MSP430_IIC" --include_path="C:/Users/panfeng/Desktop/Fly_Me/LargeFourAxis/LargeFourAxis/CtlFourAxis/sto_int" --include_path="C:/Users/panfeng/Desktop/Fly_Me/LargeFourAxis/LargeFourAxis/CtlFourAxis/targetConfigs" --include_path="E:/Code Composer Studio(CCS)/CCS/ccsv6/tools/compiler/ti-cgt-msp430_4.4.3/include" --advice:power=all -g --gcc --define=__MSP430F5529__ --diag_warning=225 --display_error_number --diag_wrap=off --silicon_errata=CPU21 --silicon_errata=CPU22 --silicon_errata=CPU23 --silicon_errata=CPU40 --printf_support=minimal --preproc_with_compile --preproc_dependency="msp430_clock/msp430_clock.pp" --obj_directory="msp430_clock" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


