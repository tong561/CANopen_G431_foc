#ifndef _BSPUSB_H_
#define	_BSPUSB_H_


#include <stdint.h>

int usb_print(const char *fmt, ...);
void USB_RxPush(const uint8_t *data, uint32_t length);
void USB_CommandProcess(void);
extern volatile int32_t USB_SpeedTargetRPM;
extern volatile int64_t USB_PositionTarget;
extern volatile uint8_t USB_ControlMode;



















#endif
