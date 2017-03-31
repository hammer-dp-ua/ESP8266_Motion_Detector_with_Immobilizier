@echo off

Rem ******NOTICE******
Rem MUST set SDK_PATH & BIN_PATH firstly!!!
Rem example:
Rem set SDK_PATH=/c/esp_iot_sdk_freertos
Rem set BIN_PATH=/c/esp8266_bin

rem set SDK_PATH=C:\Users\USER\ESP8266\ESP8266_RTOS_SDK
set SDK_PATH=/c/Users/USER/ESP8266/ESP8266_RTOS_SDK
set BIN_PATH_WIN=C:\Users\USER\ESP8266\ESP8266_RTOS_SDK\bin
set BIN_PATH=/c/Users/USER/ESP8266/ESP8266_RTOS_SDK/bin
set FOTA_PATH=Z:\USER\workspace\ESP8266_FOTA

if not %SDK_PATH% == "" (
    echo SDK_PATH: %SDK_PATH%
) else (
    echo ERROR: Please set SDK_PATH in gen_misc.bat firstly, exit!!!
    goto end
)

if not %BIN_PATH% == "" (
    echo BIN_PATH: %BIN_PATH%
) else (
    echo ERROR: Please set BIN_PATH in gen_misc.bat firstly, exit!!!
    goto end
)

make clean
make COMPILE= BOOT=new APP=1 SPI_SPEED=40 SPI_MODE=QIO SPI_SIZE_MAP=2
cp %BIN_PATH_WIN%\upgrade\user1.1024.new.2.bin %FOTA_PATH%\user1.bin

make clean
make COMPILE= BOOT=new APP=2 SPI_SPEED=40 SPI_MODE=QIO SPI_SIZE_MAP=2
cp %BIN_PATH_WIN%\upgrade\user2.1024.new.2.bin %FOTA_PATH%\user2.bin

rem set out_file=%~dp0.output\eagle\debug\image\eagle.app.v6.out

rem if EXIST %out_file% (
rem    echo %out_file% was created
rem    xtensa-lx106-elf-objdump -dgl %out_file% > %~dp0\disassembled.txt
rem )

:end