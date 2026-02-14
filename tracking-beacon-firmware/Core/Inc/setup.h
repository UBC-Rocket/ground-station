#ifndef SETUP_H
#define SETUP_H

/* Blocks until the user finishes manual alignment.
   Call after Stepper_Init, before Passthrough/RSSI/Tracker init. */
void Setup_ManualAlign(void);

#endif /* SETUP_H */
