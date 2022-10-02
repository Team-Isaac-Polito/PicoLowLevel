COMPILER = arduino-cli
BOARD_NAME = rp2040:rp2040:rpipicow

BUILD_PATH = .//bin
BUILD_PATH1 = .//bin//mod1
BUILD_PATH2 = .//bin//mod2
BUILD_PATH3 = .//bin//mod3

MODULE1 = MOD_HEAD
MODULE2 = MOD_MIDDLE
MODULE3 = MOD_TAIL

VERSION := $(shell git rev-parse --short HEAD)

ifdef OS
	RM = del /s /q > nul
else
   ifeq ($(shell uname), Linux)
      RM = rm -rf
   endif
endif

target: clean mod1 mod2 mod3
	
mod1:
	@echo compiling module 1
	@$(COMPILER) compile --fqbn $(BOARD_NAME) --board-options flash=2097152_1048576 --build-path $(BUILD_PATH1) --build-property build.extra_flags="-DVERSION=\"$(VERSION)\" -D$(MODULE1)" PicoLowLevel.ino

mod2:
	@echo compiling module 2
	@$(COMPILER) compile --fqbn $(BOARD_NAME) --board-options flash=2097152_1048576 --build-path $(BUILD_PATH2) --build-property build.extra_flags="-DVERSION=\"$(VERSION)\" -D$(MODULE2)" PicoLowLevel.ino

mod3:
	@echo compiling module 3
	@$(COMPILER) compile --fqbn $(BOARD_NAME) --board-options flash=2097152_1048576 --build-path $(BUILD_PATH3) --build-property build.extra_flags="-DVERSION=\"$(VERSION)\" -D$(MODULE3)" PicoLowLevel.ino

clean: 
	@$(RM) "$(BUILD_PATH)" || true
