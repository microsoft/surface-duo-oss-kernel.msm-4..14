Install SynaToolbox
-------------------
- Connect host to target device via ADB
- Open DOS prompt on host and go to directory containing SynaToolbox binary executable
- Run 'adb devices' to ensure connection with target device
- Run 'adb root' to acquire root privileges
- Run 'adb wait-for-devices' to ensure re-establishment of ADB connection
- Run 'adb push SynaToolbox /data' to copy SynaToolbox binary executable to /data directory on target device
- Run 'adb shell chmod 777 /data/SynaToolbox' to change file permission settings of SynaToolbox on target device



Runing SynaToolbox
------------------
There are two ways to run SynaToolbox - through menu selection and through single-line commands.
- Manual selection
  - Go into shell prompt on target device
  - Go to /data directory
  - Run SynaToolbox to bring up menu selection
  - Enter tool selection
- Single-line commands
  - Go into shell prompt on target device
  - Go to /data directory
  - Run SynaToolbox and specify tool selection and parameters as single-line command
    SynaToolbox fw_update [parameters...]
    SynaToolbox read_report [parameters...]
    SynaToolbox reg_access [parameters...]
    SynaToolbox data_logger [parameters...]
    SynaToolbox image_logger [parameters...]



Firmware Update (fw_update)
---------------------------
Parameters
[-b {image file}] - Name of image file
[-h {ihex file}] - Name of iHex file
[-ld] - Do lockdown
[-gc] - Write guest code
[-r] - Read config area
[-ui] - UI config area
[-pm] - Permanent config area
[-bl] - Bootloader config area
[-dp] - Display config area
[-f] - Force reflash
[-v] - Verbose output
[-i {driver number}] - Target input driver number

Parameter usage examples
- Perform reflash using PR1234567.img
   -b PR1234567.img
- Perform reflash using PR1234567.img and do lockdown
   -b PR1234567.img -ld
- Perform reflash using PR1234567.img based on second input driver
   -b PR1234567.img -i 1
- Perform reflash using PR1234567.img regardless of PR number of firmware on target device
   -b PR1234567.img -f
- Write config data from PR1234567.img
   -b PR1234567.img -ui
- Write guest code from PR1234567.img
   -b PR1234567.img -gc
- Read UI config area
   -r -ui
- Read permanent config area
   -r -pm
- Perform microbootloader mode recovery using PR1234567.iHex.hex
   -h PR1234567.iHex.hex

Firmware update flow
* If [-f] parameter is set, proceed with reflash
* If target device is in flash programming mode, proceed with reflash
* If PR number of new firmware image is greater than PR number of firmware on target device, proceed with reflash
* If PR number of new firmware image is equal to PR number of firmware on target device, check config ID information. If config ID of new firmware image is greater than config ID of firmware on target device, proceed with updating UI config data only
* Otherwise, no reflash is performed



Read Report (read_report)
-------------------------
Parameters
[-n] - number of report readings to take
[-c] - display output in 2D format

Parameter usage examples
- Read report type 20 once
   20
- Read report type 3 once and display output in 2D format
   3 -c
- Read report type 2 15 times and display output in 2D format
   2 -n 15 -c



Register Access (reg_access)
----------------------------
Parameters
[-a {address in hex}] - Start address (16-bit) for doing reading/writing
[-o {register offset}] - Offset of register from start address to do read/write from
[-l {length to read}] - Length in bytes to read
[-d {data to write}] - Data to write (MSB = first byte to write)
[-r] - Read for number of bytes set with -l parameter
[-w] - Write data set with -d parameter
[-c] - Use /dev/rmi0 character device node
[-i {driver number}] - Target input driver number

Parameter usage examples
- Read five bytes of data starting from third register from address 0x048a
   -a 0x048a -o 3 -l 5 -r
- Use /dev/rmi0 to write 0x11 0x22 0x33 to address 0x048a starting with 0x11
   -a 0x048a -d 0x112233 -w -c
- Read one byte of data from address 0x048a based on second input driver
   -a 0x048a -l 1 -r -i 1



Data Logger (data_logger)
-------------------------
Parameters
[-a {address in hex}] - Start address (16-bit) for doing data reading
[-l {length to read}] - Length in bytes to read from start address
[-m {interrupt mask}] - Interrupt status bit(s) to trigger data reading
[-t {time duration}] - Amount of time in seconds to log data for

Parameter usage examples
- Read seven bytes of data starting from address 0x0006 for each 0x04 interrupt event for 10 seconds
   -a 0x0006 -l 7 -m 0x04 -t 10

Notes
* Logged data stored in /data/data_logger_output.txt on target device
* Multiple interrupt status bits may be OR'ed together to form interrupt mask



Image Logger (image_logger)
---------------------------
Parameters
[-n {number of readings}] - Number of image readings to keep in log
[-r {number of receivers}] - Number of receiver electrodes
[-t {number of transmitters}] - Number of transmitters electrodes

Parameter usage examples
- Log 15 delta cap (report type 2) image readings
   2 -n 15
- Log 200 raw cap (report type 3) image readings based on 26 receivers and 18 transmitters
   3 -n 200 -r 26 -t 18

Notes
* Only delta cap (report type 2) and raw cap (report type 3) image readings supported
* Image readings logged in circular buffer fashion until terminated with Ctrl + \
* Default numbers of receivers and transmitters overridden using -r and -t parameters
