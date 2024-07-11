#include <IOKit/hid/IOHIDManager.h>
#include <iostream>
#include <vector>
#include <memory>

class HIDDevice {
public:
    HIDDevice(IOHIDDeviceRef device) : m_device(device) {
        IOHIDDeviceOpen(m_device, kIOHIDOptionsTypeNone);
    }

    ~HIDDevice() {
        IOHIDDeviceClose(m_device, kIOHIDOptionsTypeNone);
    }

    void sendReport(const std::vector<uint8_t>& report) {
        IOHIDDeviceSetReport(m_device, kIOHIDReportTypeOutput, 0, report.data(), report.size());
    }

    static void inputReportCallback(void* context, IOReturn result, void* sender,
                                    IOHIDReportType type, uint32_t reportID,
                                    uint8_t* report, CFIndex reportLength) {
        std::cout << "Received report: ";
        for (CFIndex i = 0; i < reportLength; i++) {
            printf("%02X ", report[i]);
        }
        std::cout << std::endl;
    }

    void registerInputReportCallback() {
        static uint8_t report[64];  // Adjust size as needed
        IOHIDDeviceRegisterInputReportCallback(m_device, report, sizeof(report),
                                               inputReportCallback, nullptr);
    }

private:
    IOHIDDeviceRef m_device;
};

class HIDManager {
public:
    HIDManager() : m_manager(IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDOptionsTypeNone)) {
        IOHIDManagerSetDeviceMatching(m_manager, createMatchingDictionary());
        IOHIDManagerRegisterDeviceMatchingCallback(m_manager, deviceAdded, this);
        IOHIDManagerScheduleWithRunLoop(m_manager, CFRunLoopGetCurrent(), kCFRunLoopDefaultMode);
        IOHIDManagerOpen(m_manager, kIOHIDOptionsTypeNone);
    }

    ~HIDManager() {
        IOHIDManagerClose(m_manager, kIOHIDOptionsTypeNone);
        CFRelease(m_manager);
    }

    void run() {
        std::cout << "Waiting for device connection..." << std::endl;
        CFRunLoopRun();
    }

private:
    static void deviceAdded(void* context, IOReturn result, void* sender, IOHIDDeviceRef device) {
        auto* manager = static_cast<HIDManager*>(context);
        manager->handleDeviceAdded(device);
    }

    void handleDeviceAdded(IOHIDDeviceRef device) {
        std::cout << "Device added" << std::endl;

        auto hidDevice = std::make_unique<HIDDevice>(device);
        hidDevice->registerInputReportCallback();

        // Example: Send a custom report
        std::vector<uint8_t> report = {0x01, 0x02, 0x03};
        hidDevice->sendReport(report);

        m_devices.push_back(std::move(hidDevice));
    }

    static CFMutableDictionaryRef createMatchingDictionary() {
        CFMutableDictionaryRef dict = CFDictionaryCreateMutable(kCFAllocatorDefault, 0,
                                                                &kCFTypeDictionaryKeyCallBacks,
                                                                &kCFTypeDictionaryValueCallBacks);

        int vendorID = 0x3333;
        int productID = 0x7856;
        CFNumberRef vidNumber = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &vendorID);
        CFNumberRef pidNumber = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &productID);

        CFDictionarySetValue(dict, CFSTR(kIOHIDVendorIDKey), vidNumber);
        CFDictionarySetValue(dict, CFSTR(kIOHIDProductIDKey), pidNumber);

        CFRelease(vidNumber);
        CFRelease(pidNumber);

        return dict;
    }

    IOHIDManagerRef m_manager;
    std::vector<std::unique_ptr<HIDDevice>> m_devices;
};

int main() {
    HIDManager manager;
    manager.run();
    return 0;
}