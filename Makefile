all: clean build flash

build: clean
	/usr/bin/cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_BUILD_TYPE=Debug -DPRESET_NAME=debug -DCMAKE_INSTALL_PREFIX=/home/anders/git/dtu/embedded/02226_Firmware/build/debug/install -DCMAKE_TOOLCHAIN_FILE:FILEPATH=/home/anders/git/dtu/embedded/02226_Firmware/cmake/gcc-arm-none-eabi.cmake -S/home/anders/git/dtu/embedded/02226_Firmware -B/home/anders/git/dtu/embedded/02226_Firmware/build/debug/build -G Ninja
	cmake --build ./build/debug/build/

clean:
	rm -fr build

flash:
	STM32CubeProgrammer -c port=SWD freq=12000 -w ./build/debug/build/02226_Firmware.elf 0x08000000
