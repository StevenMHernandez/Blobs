#ifndef BLOBS_CORE_H
#define BLOBS_CORE_H

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));

void initVariant() {}

// Initialize C library
extern "C" void __libc_init_array(void);

/*
 * \brief Main entry point of Arduino application
 */
int main(void) {
    init();

    __libc_init_array();

    initVariant();

    delay(1);

#if defined(USE_TINYUSB)
    Adafruit_TinyUSB_Core_init();
#elif defined(USBCON)
    USBDevice.init();
    USBDevice.attach();
#endif

    setup();

    for (;;) {
        loop();
        yield(); // yield run usb background task

        if (serialEventRun) serialEventRun();
    }

    return 0;
}

#endif //BLOBS_CORE_H
