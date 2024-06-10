#include <iostream>
#include <hidapi/hidapi.h>

int main() {
    // Initialize the hidapi library
    if (hid_init()) {
        std::cerr << "Failed to initialize hidapi library" << std::endl;
        return -1;
    }

    // Enumerate all HID devices
    struct hid_device_info *devs, *cur_dev;
    devs = hid_enumerate(0x0, 0x0);
    cur_dev = devs;

    // Iterate through the device list and print information
    while (cur_dev) {
        // std::wcout << L"Device Found:" << std::endl;
        std::wcout << L"  Type: " << cur_dev->vendor_id << L":" << cur_dev->product_id ;
        std::wcout << L"  Path: " << cur_dev->path ;
        std::wcout << L"  Manufacturer: " << cur_dev->manufacturer_string ;
        std::wcout << L"  Product: " << cur_dev->product_string ;
        std::wcout << L"  Serial Number: " << cur_dev->serial_number ;
        std::wcout << L"  Release: " << cur_dev->release_number ;
        std::wcout << L"  Interface: " << cur_dev->interface_number ;
                std::wcout << L"  usage_page: " << cur_dev->usage_page ;
        std::wcout << L"  usage: " << cur_dev->usage << std::endl;

        // std::wcout << std::endl;

        cur_dev = cur_dev->next;
    }

    // Free the enumeration linked list
}