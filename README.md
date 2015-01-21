# serial-avenger

## What?

This tests if the is correctly transferd and received by the underlying driver.

this tool sends data frames from 7 bytes to 245 bytes to an serial_port and checks if the received data is exactly the same.
This tool can also be startet in ` echo mode ` so it only send the received data back to the port.

## Usage

To use this software you need two peaces, the echo-server and the test-client

### echo mode
 first start the echo-server

 Start it like ` serial-tester echo /dev/ttyUSB0 `
 
### client

  start it like ` serial-tester /dev/ttyO3 `
  
  
### Details

 1. the `client` sends a byte frame with the content ` 0xFF 0xFE 0xFD 0xFC 0xFB ... 0x01`
the size of the frame starts with _ length of 7 bytes _ and is increased by 7 bytes each step. _ until 245 bytes _ the started by 7 bytes again.
  

 2. the `server` just receives the data and sends it back to the client, byte by byte.
 
 3. the client reveives the data and checks if it is exactly the data which is expected. If not enough data is send, the client waits until all data is received.
  if all Data is correct, after 25 milliseconds the next frame is send.





 
