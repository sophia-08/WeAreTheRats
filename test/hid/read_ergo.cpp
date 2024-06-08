#include <stdio.h>
#include <wchar.h>
#include <string.h>
#include <hidapi/hidapi.h>

int main(int argc, char* argv[]) {
    if (hid_init()) {
        printf("Failed to initialize HIDAPI.\n");
        return -1;
    }

    // Replace with your device's Vendor ID and Product ID
    hid_device *device = hid_open(13107, 30806,  NULL);
    if (!device) {
        printf("Unable to open device.\n");
        return -1;
    }

    unsigned char buffer[65];
    int res = hid_read(device, buffer, sizeof(buffer));
    if (res < 0) {
        printf("Error reading from device.\n");
    } else {
        printf("Read %d bytes:\n", res);
        for (int i = 0; i < res; i++) {
            printf("%02hhX ", buffer[i]);
        }
        printf("\n");
    }

    hid_close(device);
    hid_exit();

    return 0;
}
