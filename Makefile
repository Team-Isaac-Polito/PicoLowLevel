#Modules
MODULE1 = mod1
MODULE2 = mod2
MODULE3 = mod3

target: mod1 mod2 mod3

#Build only
mod1: 
	pio run -e $(MODULE1) 
mod2:
	pio run -e $(MODULE2) 
mod3:
	pio run -e $(MODULE3)

#Build and upload
upload_mod1:
	pio run -e $(MODULE1) -t upload
upload_mod2:
	pio run -e $(MODULE2) -t upload
upload_mod3:
	pio run -e $(MODULE3) -t upload

#Remove build files
clean:
	pio run -t clean