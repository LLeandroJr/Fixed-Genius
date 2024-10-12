CHAIN=$(CHAINPATH)arm-none-eabi
CFLAGS=-mcpu=cortex-a8 -mfpu=vfp -mfloat-abi=softfp
AFLAGS=-mfpu=neon -mcpu=cortex-a8
IPATH=-Iinc/
SRC=src/
OBJ=obj/
BIN=bin/
TFTPBOOT=/tftpboot/

all: app

app: start.o main.o pad.o clock_module.o control_module.o gpio.o interruption.o timers.o uart.o watchdog.o
	$(CHAIN)-ld $(OBJ)start.o $(OBJ)main.o $(OBJ)pad.o $(OBJ)clock_module.o $(OBJ)control_module.o $(OBJ)gpio.o $(OBJ)interruption.o $(OBJ)timers.o $(OBJ)watchdog.o $(OBJ)uart.o -T $(SRC)memmap.ld -o $(OBJ)main.elf
	$(CHAIN)-objcopy $(OBJ)main.elf $(BIN)spl.boot -O binary
	cp $(BIN)spl.boot $(TFTPBOOT)appGpio.bin

start.o: $(SRC)start.s
	$(CHAIN)-as $(AFLAGS) $(SRC)start.s -o $(OBJ)start.o

main.o: $(SRC)main.c
	$(CHAIN)-gcc $(CFLAGS) $(IPATH) -c $(SRC)main.c -o $(OBJ)main.o

# adicionado
clock_module.o: $(SRC)CLOCK_MODULE_Registers.c
	$(CHAIN)-gcc $(CFLAGS) $(IPATH) -c $(SRC)CLOCK_MODULE_Registers.c -o $(OBJ)clock_module.o

control_module.o: $(SRC)CONTROL_MODULE_Registers.c
	$(CHAIN)-gcc $(CFLAGS) $(IPATH) -c $(SRC)CONTROL_MODULE_Registers.c -o $(OBJ)control_module.o

gpio.o: $(SRC)GPIO_Registers.c
	$(CHAIN)-gcc $(CFLAGS) $(IPATH) -c $(SRC)GPIO_Registers.c -o $(OBJ)gpio.o

interruption.o: $(SRC)INTERRUPT_SERVICE_ROUTINE.c
	$(CHAIN)-gcc $(CFLAGS) $(IPATH) -c $(SRC)INTERRUPT_SERVICE_ROUTINE.c -o $(OBJ)interruption.o

timers.o: $(SRC)TIMERS_Registers.c
	$(CHAIN)-gcc $(CFLAGS) $(IPATH) -c $(SRC)TIMERS_Registers.c -o $(OBJ)timers.o

uart.o: $(SRC)uart.c
	$(CHAIN)-gcc $(CFLAGS) $(IPATH) -c $(SRC)uart.c -o $(OBJ)uart.o
	
watchdog.o: $(SRC)WATCHDOG_TIMER_Registers.c
	$(CHAIN)-gcc $(CFLAGS) $(IPATH) -c $(SRC)WATCHDOG_TIMER_Registers.c -o $(OBJ)watchdog.o	

# fim

pad.o: $(SRC)PAD_CONTROL_Registers.c
	$(CHAIN)-gcc $(CFLAGS) $(IPATH) -c $(SRC)PAD_CONTROL_Registers.c -o $(OBJ)pad.o
                                         
copy:
	cp $(BIN)spl.boot $(TFTPBOOT)app.bin

link:
	ln -f $(TFTPBOOT)app.bin $(TFTPBOOT)download.bin

clean:
	rm -rf $(OBJ)*.o
	rm -rf $(OBJ)*.elf
	rm -rf $(BIN)*.boot
	rm $(TFTPBOOT)download.bin
	rm $(TFTPBOOT)app.bin

dump:
	$(CHAIN)-objdump -D $(OBJ)main.elf
