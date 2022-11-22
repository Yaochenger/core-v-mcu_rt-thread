# Using CLI Test on the CORE-V-MCU

## Interrupt Assignments

Interrupt 0 - 6   RESERVED for sw events
Interrupt 7       Timer low event
Interrupt 8 - 10  Unused
Interrupt 11      Event FIFO valid
Interrupt 12 - 15 Unused
Interrupt 16      Timer low event
Interrupt 17      Timer high event
Interrupt 18      Reference clock rise event
Interrupt 19      Reference clock fall event
Interrupt 20      I2C Slave event
Interrupt 21      Advance timer event 0
Interrupt 22      Advance timer event 1
Interrupt 23      Advance timer event 2
Interrupt 24      Advance timer event 3
Interrupt 25      eFPGA event 0
Interrupt 26      eFPGA event 1
Interrupt 27      eFPGA event 2
Interrupt 28      eFPGA event 3
Interrupt 29      eFPGA event 4
Interrupt 30      eFPGA event 5
Interrupt 31      Error event 

## cli_test.bin

cli_test.bin is the actual application binary file generated
Header.bin is added to cli_test.bin at the beginning for the bootloader to understand and load the application.
So, Header.bin + cli_test.bin = cli.bin which is programmed into QSPI flash or is used by any external host to load using the bootloader via I2C bus.
For simulating the hardware, header.bin is changed a bit to header_sim.bin which skips the flash loading into RAM as the RAM in simulation is already initialized with the app code. So Header_sim.bin + cli_test.bin = cli_sim.bin.

Structure of header.bin which is added to cli_test.bin so that the bootloader can load the application code and boot.

typedef struct {
  uint32_t nextDesc;
  uint32_t nbAreas;  Number of areas in the header.
  uint32_t entry;    Entry address of the application code.
  uint32_t bootaddr; Boot address of the application code.
} __packed flash_v2_header_t;

typedef struct {
  uint32_t start;   From where does the application code start in this bin file. i.e. Index of the application code.
  uint32_t ptr;     
  uint32_t size;    Size of the block
  uint32_t blocks;  Number of blocks to read
} __packed flash_v2_mem_area_t;


typedef struct {	//16+256 = 272 bytes of header.
  flash_v2_header_t header;
  flash_v2_mem_area_t memArea[MAX_NB_AREA]; MAX_NB_AREA = 16 flash areas can be described
} __packed boot_code_t;


Total header size is 272 bytes. 16 bytes of header and 256 bytes of memory area.

[0x10,0x00,0x00,0x00],[0x01,0x00,0x00,0x00],[0x80,0x08,0x00,0x1c],[0x80,0x08,0x00,0x1c],
nextDesc=0x00000010    nbAreas=0x00000001   entry=0x1c000880      bootaddr=0x1c000880

Memory area 1
[0x10,0x01,0x00,0x00],[0x00,0x08,0x00,0x1c],[0x00,0x00,0x03,0x00],[0x30,0x00,0x00,0x00],
start=0x00000110      ptr=0x1c000800        size=0x00030000       blocks=0x00000030

Memory area 2 (unused)
[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],
start=0xffffffff      ptr=0xffffffff        size=0xffffffff       blocks=0xffffffff

Memory area 3 (unused)
[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],
start=0xffffffff      ptr=0xffffffff        size=0xffffffff       blocks=0xffffffff

Memory area 4 (unused)
[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],
start=0xffffffff      ptr=0xffffffff        size=0xffffffff       blocks=0xffffffff

Memory area 5 (unused)
[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],
start=0xffffffff      ptr=0xffffffff        size=0xffffffff       blocks=0xffffffff

Memory area 6 (unused)
[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],
start=0xffffffff      ptr=0xffffffff        size=0xffffffff       blocks=0xffffffff

Memory area 7 (unused)
[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],
start=0xffffffff      ptr=0xffffffff        size=0xffffffff       blocks=0xffffffff

Memory area 8 (unused)
[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],
start=0xffffffff      ptr=0xffffffff        size=0xffffffff       blocks=0xffffffff

Memory area 9 (unused)
[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],
start=0xffffffff      ptr=0xffffffff        size=0xffffffff       blocks=0xffffffff

Memory area 10 (unused)
[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],

Memory area 11 (unused)
[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],
start=0xffffffff      ptr=0xffffffff        size=0xffffffff       blocks=0xffffffff

Memory area 12 (unused)
[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],
start=0xffffffff      ptr=0xffffffff        size=0xffffffff       blocks=0xffffffff

Memory area 13 (unused)
[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],
start=0xffffffff      ptr=0xffffffff        size=0xffffffff       blocks=0xffffffff

Memory area 14 (unused)
[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],
start=0xffffffff      ptr=0xffffffff        size=0xffffffff       blocks=0xffffffff

Memory area 15 (unused)
[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],
start=0xffffffff      ptr=0xffffffff        size=0xffffffff       blocks=0xffffffff

Memory area 16 (unused)
[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],[0xff,0xff,0xff,0xff],
start=0xffffffff      ptr=0xffffffff        size=0xffffffff       blocks=0xffffffff

## Connecting an external host to load an application into CORE-V-MCU

### Nexsys
JC10    Slave SCL
JC9     Slave SDA    

1. Load the bit file into the SD card. Insert the SD card and power on the Nexsys board. Press program button to program the bit file to FPGA.
2. Open a terminal window for UART1 of CORE-V-MCU. Once the Nexsys bit file is programmed and done bit is high, you can see bootloader message from CORE-V-MCU UART1 on the terminal. 
3. The value of the bootsel switch should be indicating 0 and dots will be printing indicating that CORE-V-MCU bootloader is waiting for an external host to connect to it over I2C.
4. Now, reset the STM32 board. The green LED will stay on and red LED will keep blinking indicating that the host is loading the application over I2C.
5. Once the application is loaded, the orange led will glow indicating the completion of loading the application code. The blue LED will keep blinking to indicate that STM32 is in a while (1) loop.
6. CORE-V-MCU should have loaded the application and this can be verified by connecting a termial to UART0 and checking if the command line is up. I2C BL JMP 1c000880 is printed on the terminal connected to UART1 of CORE-V-MCU.

### I2C protocol between an external host and CORE-V-MCU bootloader

CORE-V-MCU bootloader is running as an I2C slave with a 8-bit slave address 0x62

Protocol structure
2 Bytes   Start Of Frame (SOF)
4 Bytes   RAM address
1 Byte    Command type
1 Byte    Data length
240 Bytes Payload data
2 Bytes   CRC  (Optional)

Start Of Frame (SOF) indicates the message direction. A2 to Host is 0x5A70 and Host to A2 is 0xA507
RAM address indicates the A2 RAM address which needs to be written to read from.

All command and responses are implemented in the 1 byte I2C to APB / APB to I2C single-byte message register, except load memory command and jump to address command.
The command load memory uses the 256 byte deep FIFO to communicate with I2C slave. It uses the above mentioned protocol structure.

CORE-V-MCU slave on booting up writes the reason for power on and waits for the host to connect on I2C bus.

#### Initial connection process
The external host connects to the I2C slave and reads the reset reason.
Depending on the reset reason the host decides whether to program CORE-V-MCU or not.

#### Programming CORE-V-MCU via I2C
1. The I2C master (external host) after the initial connection process, checks if the I2C slave (CORE-V-MCU) is ready. There are 2 options with CRC and without CRC.
Both options are supported by the example implementation.
2. The host waits until there is a response from the slave to be ready. Once the slave replies with a ready status, the host proceeds next.
3. The host sends the firmware data using the load command to the slave. If the CRC is enabled the host calculates the CRC and sends it in the CRC field.
4. The slave upon receiving the firmware data, checks the CRC and copies the data into the RAM addresss specified - if CRC is enabled or if CRC is disabled, the slave directly copies the data into the RAM address specified.
5. If the CRC check has failed, the slave informs the host, so that the host can retry the frame.
6. Once all firmware data is pushed to the slave, the host issues a jump to address command with the RAM address to jump.
7. The slave (CORE-V-MCU) jumps to the RAM address mentioned by the host.

## Programming QSPI flash

launch spi_load.py in a terminal using 
python3 spi_load.py /dev/ttyUSB1

load the cli_test application on CORE-V-MCU.
Commands on CORE-V-MCU
qspi
program Default/cli.bin 0x0

Programming will start indicating the progress on CORE-V-MCU UART terminal

To change the bootsel pin status:
Open the terminal connected to UART 1 of CORE-V-MCU. 
Load the bit file. 
Once the bit file is programmed, and running, check the status of bootsel print.
If boot sel status is to be changed, first set SW0 to low (south) and the set SW1 to high or low.
Then press reset button. Now the SW1 (bootsel) will be taken correctly.

When connecting JTAG interface to debug make sure that the SW0 switch is in high position (north) for the debugger to work.

For any change in SW0 or SW1 settings to be reflected, press reset button.


