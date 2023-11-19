build-receiver: clean
	/usr/bin/cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=/home/anders/git/dtu/embedded/02226_Firmware/build/receiver/install -DCMAKE_TOOLCHAIN_FILE:FILEPATH=/home/anders/git/dtu/embedded/02226_Firmware/cmake/gcc-arm-none-eabi.cmake -S/home/anders/git/dtu/embedded/02226_Firmware -B/home/anders/git/dtu/embedded/02226_Firmware/build/receiver/build -G Ninja
	cmake --build ./build/receiver/build/

build-transmitter: clean
	/usr/bin/cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_BUILD_TYPE=Debug -DTRANSMITTER_ENABLE=ON -DCMAKE_INSTALL_PREFIX=/home/anders/git/dtu/embedded/02226_Firmware/build/transmitter/install -DCMAKE_TOOLCHAIN_FILE:FILEPATH=/home/anders/git/dtu/embedded/02226_Firmware/cmake/gcc-arm-none-eabi.cmake -S/home/anders/git/dtu/embedded/02226_Firmware -B/home/anders/git/dtu/embedded/02226_Firmware/build/transmitter/build -G Ninja
	cmake --build ./build/transmitter/build/

clean:
	rm -fr build

flash-receiver:
	STM32CubeProgrammer -c port=SWD freq=12000 sn=002A00193438510634313939 -w ./build/receiver/build/02226_Firmware.elf 0x08000000

flash-transmitter:
	STM32CubeProgrammer -c port=SWD freq=12000 sn=004500233438510A34313939 -w ./build/transmitter/build/02226_Firmware.elf 0x08000000
