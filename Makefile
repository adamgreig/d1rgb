.PHONY: build run

all: run

build:
	cargo objcopy --release -- -Obinary out.bin

run: build
	xfel ddr d1
	xfel write 0x40000000 out.bin
	xfel exec 0x40000000
