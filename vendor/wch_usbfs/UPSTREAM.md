# WCH USBFS CDC

Source: https://github.com/openwch/ch32v307/tree/080af2e52055334eaa6af0f7ad7df527b4ef5a66/EVT/EXAM/USB/USBFS/DEVICE/SimulateCDC/User/USB_Device

USB register header/declarations: EVT/EXAM/SRC/Peripheral/inc/ch32v30x_usb.h and ch32v30x.h, same revision.

Original WCH copyright and usage restriction (WCH microcontrollers) retained in vendor files. Local modifications: UART bridge replaced with receive/reset callbacks; old SDK IRQ names; independent CDC line coding. VID/PID are WCH example values for development, not an assigned product identity.
