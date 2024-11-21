#Modules
MODULE1 = mod1
MODULE2 = mod2
MODULE3 = mod3

target: clean mod1 mod2 mod3

#Build only
build_mod1: 
	pio run -e $(MODULE1) 
build_mod2:
	pio run -e $(MODULE2) 
build_mod3:
	pio run -e $(MODULE3)

#Build and upload
mod1:
	pio run -e $(MODULE1) -t upload
mod2:
	pio run -e $(MODULE2) -t upload
mod3:
	pio run -e $(MODULE3) -t upload

#Remove build files
clean:
	pio run -t clean