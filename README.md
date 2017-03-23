Zephyr-OS example
=================

Dependencies:
-------------
	* git clone https://github.com/longcongduoi/zephyr.git

Clone the code:
---------------
	* git clone https://github.com/longcongduoi/ble_packet_repeater.git
	
Compile:
--------
(*~~linux~~*)  	
	1. cd zephyr
	2. source zephyr-env.sh
	3. cd ..
	4. cd ble_packet_repeater
	5. make BOARD=nr51_vbluno
	
Flash to board:
---------------
	* simple copy outdir/nr51_vbluno/zephyr.hex to /media/<user name>/DAPLINK
	 
	

