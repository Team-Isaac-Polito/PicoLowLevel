COMPILER = arduino-cli
BOARD_NAME = rp2040:rp2040:rpipico
BUILD_PATH = .//bin
BUILD_PATH1 = .//bin//mod1
BUILD_PATH2 = .//bin//mod2
BUILD_PATH3 = .//bin//mod3

MODULE1 = MOD_HEAD
MODULE2 = MOD_MIDDLE
MODULE3 = MOD_TAIL

ifdef OS
	RM = del /s /q
else
   ifeq ($(shell uname), Linux)
      RM = rm -rf
   endif
endif

target: distclean
	echo compiling module 1
	$(COMPILER) compile --fqbn $(BOARD_NAME) --build-path $(BUILD_PATH1) --build-property "build.extra_flags=\"-D$(MODULE1)\"" PicoLowLevel.ino
	echo compiling module 2
	$(COMPILER) compile --fqbn $(BOARD_NAME) --build-path $(BUILD_PATH2) --build-property "build.extra_flags=\"-D$(MODULE2)\"" PicoLowLevel.ino
	echo compiling module 3
	$(COMPILER) compile --fqbn $(BOARD_NAME) --build-path $(BUILD_PATH3) --build-property "build.extra_flags=\"-D$(MODULE3)\"" PicoLowLevel.ino
	

	

distclean: 
	$(RM) "$(BUILD_PATH)"
