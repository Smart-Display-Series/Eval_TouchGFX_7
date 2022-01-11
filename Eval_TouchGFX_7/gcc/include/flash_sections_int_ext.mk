_extflash_:
	@cd "$(st_stm32cube_programmer_path)" && ./$(stm32cube_programmer_filename) -c port=SWD -d $(application_path)/$(binary_output_path)/target.hex -el $(application_path)/Drivers/vendor/Board/WL0F0007000A8GAAASB00/Flashloader/$(stldr) -hardRst
_intflash_:
	@cd "$(st_stm32cube_programmer_path)" && ./$(stm32cube_programmer_filename) -c port=SWD -d $(application_path)/$(binary_output_path)/intflash.hex -hardRst
