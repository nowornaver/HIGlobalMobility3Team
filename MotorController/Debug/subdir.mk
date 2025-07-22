################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../AngularControl.c" \
"../AppMode.c" \
"../AppMode_Error.c" \
"../AppMode_Idle.c" \
"../AppMode_Init.c" \
"../AppMode_Operation.c" \
"../AppScheduling.c" \
"../Cpu0_Main.c" \
"../Driver_Adc.c" \
"../Driver_Asc.c" \
"../Driver_Gtm.c" \
"../Driver_Port.c" \
"../Driver_Stm.c" \
"../Driver_Watchdog.c" \
"../DrvDio.c" \
"../MotorControl.c" 

COMPILED_SRCS += \
"AngularControl.src" \
"AppMode.src" \
"AppMode_Error.src" \
"AppMode_Idle.src" \
"AppMode_Init.src" \
"AppMode_Operation.src" \
"AppScheduling.src" \
"Cpu0_Main.src" \
"Driver_Adc.src" \
"Driver_Asc.src" \
"Driver_Gtm.src" \
"Driver_Port.src" \
"Driver_Stm.src" \
"Driver_Watchdog.src" \
"DrvDio.src" \
"MotorControl.src" 

C_DEPS += \
"./AngularControl.d" \
"./AppMode.d" \
"./AppMode_Error.d" \
"./AppMode_Idle.d" \
"./AppMode_Init.d" \
"./AppMode_Operation.d" \
"./AppScheduling.d" \
"./Cpu0_Main.d" \
"./Driver_Adc.d" \
"./Driver_Asc.d" \
"./Driver_Gtm.d" \
"./Driver_Port.d" \
"./Driver_Stm.d" \
"./Driver_Watchdog.d" \
"./DrvDio.d" \
"./MotorControl.d" 

OBJS += \
"AngularControl.o" \
"AppMode.o" \
"AppMode_Error.o" \
"AppMode_Idle.o" \
"AppMode_Init.o" \
"AppMode_Operation.o" \
"AppScheduling.o" \
"Cpu0_Main.o" \
"Driver_Adc.o" \
"Driver_Asc.o" \
"Driver_Gtm.o" \
"Driver_Port.o" \
"Driver_Stm.o" \
"Driver_Watchdog.o" \
"DrvDio.o" \
"MotorControl.o" 


# Each subdirectory must supply rules for building sources it contributes
"AngularControl.src":"../AngularControl.c" "subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/KK/hi/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"AngularControl.o":"AngularControl.src" "subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"AppMode.src":"../AppMode.c" "subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/KK/hi/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"AppMode.o":"AppMode.src" "subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"AppMode_Error.src":"../AppMode_Error.c" "subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/KK/hi/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"AppMode_Error.o":"AppMode_Error.src" "subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"AppMode_Idle.src":"../AppMode_Idle.c" "subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/KK/hi/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"AppMode_Idle.o":"AppMode_Idle.src" "subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"AppMode_Init.src":"../AppMode_Init.c" "subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/KK/hi/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"AppMode_Init.o":"AppMode_Init.src" "subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"AppMode_Operation.src":"../AppMode_Operation.c" "subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/KK/hi/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"AppMode_Operation.o":"AppMode_Operation.src" "subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"AppScheduling.src":"../AppScheduling.c" "subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/KK/hi/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"AppScheduling.o":"AppScheduling.src" "subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Cpu0_Main.src":"../Cpu0_Main.c" "subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/KK/hi/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"Cpu0_Main.o":"Cpu0_Main.src" "subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Driver_Adc.src":"../Driver_Adc.c" "subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/KK/hi/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"Driver_Adc.o":"Driver_Adc.src" "subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Driver_Asc.src":"../Driver_Asc.c" "subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/KK/hi/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"Driver_Asc.o":"Driver_Asc.src" "subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Driver_Gtm.src":"../Driver_Gtm.c" "subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/KK/hi/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"Driver_Gtm.o":"Driver_Gtm.src" "subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Driver_Port.src":"../Driver_Port.c" "subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/KK/hi/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"Driver_Port.o":"Driver_Port.src" "subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Driver_Stm.src":"../Driver_Stm.c" "subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/KK/hi/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"Driver_Stm.o":"Driver_Stm.src" "subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Driver_Watchdog.src":"../Driver_Watchdog.c" "subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/KK/hi/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"Driver_Watchdog.o":"Driver_Watchdog.src" "subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"DrvDio.src":"../DrvDio.c" "subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/KK/hi/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"DrvDio.o":"DrvDio.src" "subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"MotorControl.src":"../MotorControl.c" "subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc27xd "-fC:/KK/hi/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc27xd -Y0 -N0 -Z0 -o "$@" "$<"
"MotorControl.o":"MotorControl.src" "subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean--2e-

clean--2e-:
	-$(RM) ./AngularControl.d ./AngularControl.o ./AngularControl.src ./AppMode.d ./AppMode.o ./AppMode.src ./AppMode_Error.d ./AppMode_Error.o ./AppMode_Error.src ./AppMode_Idle.d ./AppMode_Idle.o ./AppMode_Idle.src ./AppMode_Init.d ./AppMode_Init.o ./AppMode_Init.src ./AppMode_Operation.d ./AppMode_Operation.o ./AppMode_Operation.src ./AppScheduling.d ./AppScheduling.o ./AppScheduling.src ./Cpu0_Main.d ./Cpu0_Main.o ./Cpu0_Main.src ./Driver_Adc.d ./Driver_Adc.o ./Driver_Adc.src ./Driver_Asc.d ./Driver_Asc.o ./Driver_Asc.src ./Driver_Gtm.d ./Driver_Gtm.o ./Driver_Gtm.src ./Driver_Port.d ./Driver_Port.o ./Driver_Port.src ./Driver_Stm.d ./Driver_Stm.o ./Driver_Stm.src ./Driver_Watchdog.d ./Driver_Watchdog.o ./Driver_Watchdog.src ./DrvDio.d ./DrvDio.o ./DrvDio.src ./MotorControl.d ./MotorControl.o ./MotorControl.src

.PHONY: clean--2e-

