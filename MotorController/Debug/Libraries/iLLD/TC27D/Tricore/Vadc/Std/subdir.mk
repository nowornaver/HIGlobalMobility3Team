################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../Libraries/iLLD/TC27D/Tricore/Vadc/Std/IfxVadc.c" 

COMPILED_SRCS += \
"Libraries/iLLD/TC27D/Tricore/Vadc/Std/IfxVadc.src" 

C_DEPS += \
"./Libraries/iLLD/TC27D/Tricore/Vadc/Std/IfxVadc.d" 

OBJS += \
"Libraries/iLLD/TC27D/Tricore/Vadc/Std/IfxVadc.o" 


# Each subdirectory must supply rules for building sources it contributes
"Libraries/iLLD/TC27D/Tricore/Vadc/Std/IfxVadc.src":"../Libraries/iLLD/TC27D/Tricore/Vadc/Std/IfxVadc.c" "Libraries/iLLD/TC27D/Tricore/Vadc/Std/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/KK/hi/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/iLLD/TC27D/Tricore/Vadc/Std/IfxVadc.o":"Libraries/iLLD/TC27D/Tricore/Vadc/Std/IfxVadc.src" "Libraries/iLLD/TC27D/Tricore/Vadc/Std/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Vadc-2f-Std

clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Vadc-2f-Std:
	-$(RM) ./Libraries/iLLD/TC27D/Tricore/Vadc/Std/IfxVadc.d ./Libraries/iLLD/TC27D/Tricore/Vadc/Std/IfxVadc.o ./Libraries/iLLD/TC27D/Tricore/Vadc/Std/IfxVadc.src

.PHONY: clean-Libraries-2f-iLLD-2f-TC27D-2f-Tricore-2f-Vadc-2f-Std

