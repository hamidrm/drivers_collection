

# Drivers Collection
Currently, there are below drivers which written for using in RTOS/Bare-metal embedded systems.

 - ublox 8:
A simple driver for ublox 8 GPS. It's easy to use. Just you must fill following structure that defined inside its header.
`ublox_uart_send` used to transmit a `uint_8 *` pointer with a determined length as second argument.
Also, you must call `ublox_data_on_recv` function on each received raw data(s) from the serial.
	```C
	    typedef struct {
	        ublox_uart_send_cb_t    ublox_uart_send;
	        ublox_uart_update_params_cb_t   ublox_uart_update_params;
	        ublox_malloc_cb_t           ublox_malloc;
	        ublox_free_cb_t     ublox_free;
	        ublox_msg_received_cb  ublox_msg_received;
	    }ublox_platform_fns_t;
	```
	Next, by calling `ublox_create` with the above data structure as its argument, `ublox_msg_received` will be called on each received message from ublox. During creating ublox handler, all the NMEA messages will be disabled and data communication will be keeping on by binary data.
All the packets that defined in ublox datasheet have been implemented in this driver, and you can Send/Receive them easily.
There are some other functions to reset ublox, reconfig communication parameters, etc. that you can see in the ublox.c functions comments.

 -  nrf24l01:
All the nrf2401 functionality have been implemented. Although, I know, without document and comments inside its code, it's just a trash! I will try to add a few document/comment ASAP.

