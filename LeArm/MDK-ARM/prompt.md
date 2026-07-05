## 连接的是USB0

sunrise@ubuntu:~/arm_ws$ ls /dev/ttyUSB*
/dev/ttyUSB0

## lsof 没有任何输出
sunrise@ubuntu:~/arm_ws$   sudo lsof /dev/ttyUSB0
sunrise@ubuntu:~/arm_ws$

## 日志


[ 1393.879373] ch341 1-1.4:1.0: device disconnected
[ 2216.529635] usb 1-1.4: new full-speed USB device number 6 using xhci-hcd
[ 2216.680713] usb 1-1.4: New USB device found, idVendor=1a86, idProduct=7523, bcdDevice=c2.33
[ 2216.680777] usb 1-1.4: New USB device strings: Mfr=0, Product=2, SerialNumber=0
[ 2216.680813] usb 1-1.4: Product: USB Serial
[ 2216.737779] ch341 1-1.4:1.0: ch341-uart converter detected
[ 2216.738477] usb 1-1.4: ch341-uart converter now attached to ttyUSB0
