#Modules
MODULE1 = mk1_mod1
MODULE2 = mk1_mod2
MODULE3 = mk2_mod1
MODULE4 = mk2_mod2

target: mk1_mod1 mk1_mod2 mk2_mod1 mk2_mod2

#Build only
mk1_mod1: 
	pio run -e $(MODULE1) 
mk1_mod2:
	pio run -e $(MODULE2) 
mk2_mod1:
	pio run -e $(MODULE3)
mk2_mod2:
	pio run -e $(MODULE4)

#Build and upload
upload_mk1_mod1:
	pio run -e $(MODULE1) -t upload
upload_mk1_mod2:
	pio run -e $(MODULE2) -t upload
upload_mk2_mod1:
	pio run -e $(MODULE3) -t upload
upload_mk2_mod2:
	pio run -e $(MODULE4) -t upload

#Remove build files
clean:
	pio run -t clean