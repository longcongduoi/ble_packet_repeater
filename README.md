Zephyr-OS example

Dependencies
	git clone https://github.com/longcongduoi/zephyr.git

Clone the code
	git clone https://github.com/longcongduoi/ble_packet_repeater.git
	
Compile
	cd zephyr
	source zephyr-env.sh
	cd ..
	cd ble_packet_repeater
	make BOARD=nr51_vbluno
	
Flash to board
	simple copy outdir/nr51_vbluno/zephyr.hex to /media/<username>/DAPLINK
	 
	

