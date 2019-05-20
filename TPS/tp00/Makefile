CC := arm-none-eabi-gcc

CFLAGS :=					\
	-Os					\
	-gdwarf-2	\
	-mthumb					\
	-fno-builtin	\
	-mcpu=cortex-m4			\
	-Wall 	\
	-std=c99	\
	-ffunction-sections	\
	-fdata-sections		\
	-mabi=aapcs				\
	-fno-unroll-loops	\
	-ffast-math	\
	-ftree-vectorize\
	-fverbose-asm \

LDFLAGS :=			\
	-nostartfiles		\
	--specs=nano.specs \
	-Wl,-gc-sections	\
	-L. -Tmyscript.ld

SRC=main.c
BIN=main.elf

all:
	${CC} ${CFLAGS} ${LDFLAGS} ${SRC} -o ${BIN}

%.bin: %.elf
	arm-none-eabi-objcopy -O binary -S $< $@
