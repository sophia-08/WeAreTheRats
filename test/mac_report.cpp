#include <IOKit/hid/IOHIDManager.h>
#include <iostream>
#include <memory>
#include <vector>

class HIDDevice {
public:
  HIDDevice(IOHIDDeviceRef device) : m_device(device) {
    IOHIDDeviceOpen(m_device, kIOHIDOptionsTypeNone);
  }

  ~HIDDevice() { IOHIDDeviceClose(m_device, kIOHIDOptionsTypeNone); }

  bool sendReport(int reportID, const std::vector<uint8_t> &report) {
    IOReturn result = IOHIDDeviceSetReport(m_device, kIOHIDReportTypeOutput, reportID,
                                           report.data(), report.size());

    if (result == kIOReturnSuccess) {
      std::cout << "Report sent successfully" << std::endl;
      return true;
    } else {
      std::cerr << "Failed to send report. Error: "
                << getIOReturnErrorString(result) << std::endl;
      return false;
    }
  }

  static void inputReportCallback(void *context, IOReturn result, void *sender,
                                  IOHIDReportType type, uint32_t reportID,
                                  uint8_t *report, CFIndex reportLength) {
    std::cout << "Received report: ";
    for (CFIndex i = 0; i < reportLength; i++) {
      printf("%02X ", report[i]);
    }
    std::cout << std::endl;

    // Cast the context back to HIDDevice*
    HIDDevice *device = static_cast<HIDDevice *>(context);

    /*
    Test 1: 

    code:
        int id=0;
    for (id=0; id<4; id++) {
    std::cerr << "send two bytes to " << id << std::endl;
    std::vector<uint8_t> out = {0x01,0x01};
    device->sendReport(id, out);      
    }

    host logs:
      send two bytes to 0
      Failed to send report. Error: Unknown error
      send two bytes to 1
      Report sent successfully
      send two bytes to 2
      Failed to send report. Error: Unknown error
      send two bytes to 3
      Failed to send report. Error: Unknown error

    Device log:
      Got 1 byte only, which match the second byte.  WHY?

    conclusion:
      only id 1, which is keyboard, has output report channel.
    */
        int id=2;
    // for (id=0; id<4; id++) 
    {
    std::cerr << "send two bytes to " << id << std::endl;
    std::vector<uint8_t> out = {0x01,0x02,0x03,0x04,0x05,6,7,8,9,};
    device->sendReport(id, out);      
    }

  }

  void registerInputReportCallback() {
    static uint8_t report[64]; // Adjust size as needed
    IOHIDDeviceRegisterInputReportCallback(m_device, report, sizeof(report),
                                           inputReportCallback,
                                           this); // Pass 'this' as context
  }

private:
  IOHIDDeviceRef m_device;
  const char *getIOReturnErrorString(IOReturn error) {
    switch (error) {
    case kIOReturnSuccess:
      return "kIOReturnSuccess";
    case kIOReturnError:
      return "kIOReturnError";
    case kIOReturnNoMemory:
      return "kIOReturnNoMemory";
    case kIOReturnNoResources:
      return "kIOReturnNoResources";
    case kIOReturnIPCError:
      return "kIOReturnIPCError";
    case kIOReturnNoDevice:
      return "kIOReturnNoDevice";
    case kIOReturnNotPrivileged:
      return "kIOReturnNotPrivileged";
    case kIOReturnBadArgument:
      return "kIOReturnBadArgument";
    case kIOReturnLockedRead:
      return "kIOReturnLockedRead";
    case kIOReturnLockedWrite:
      return "kIOReturnLockedWrite";
    case kIOReturnExclusiveAccess:
      return "kIOReturnExclusiveAccess";
    case kIOReturnBadMessageID:
      return "kIOReturnBadMessageID";
    case kIOReturnUnsupported:
      return "kIOReturnUnsupported";
    case kIOReturnVMError:
      return "kIOReturnVMError";
    case kIOReturnInternalError:
      return "kIOReturnInternalError";
    case kIOReturnIOError:
      return "kIOReturnIOError";
    default:
      return "Unknown error";
    }
  }
};

class HIDManager {
public:
  HIDManager()
      : m_manager(
            IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDOptionsTypeNone)) {
    IOHIDManagerSetDeviceMatching(m_manager, createMatchingDictionary());
    IOHIDManagerRegisterDeviceMatchingCallback(m_manager, deviceAdded, this);
    IOHIDManagerScheduleWithRunLoop(m_manager, CFRunLoopGetCurrent(),
                                    kCFRunLoopDefaultMode);
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
  static void deviceAdded(void *context, IOReturn result, void *sender,
                          IOHIDDeviceRef device) {
    auto *manager = static_cast<HIDManager *>(context);
    manager->handleDeviceAdded(device);
  }

  void handleDeviceAdded(IOHIDDeviceRef device) {
    std::cout << "Device added" << std::endl;

    auto hidDevice = std::make_unique<HIDDevice>(device);
    hidDevice->registerInputReportCallback();

    // Example: Send a custom report
    // std::vector<uint8_t> report = {0x01, 0x02, 0x03};
    // hidDevice->sendReport(report);

    m_devices.push_back(std::move(hidDevice));
  }

  static CFMutableDictionaryRef createMatchingDictionary() {
    CFMutableDictionaryRef dict = CFDictionaryCreateMutable(
        kCFAllocatorDefault, 0, &kCFTypeDictionaryKeyCallBacks,
        &kCFTypeDictionaryValueCallBacks);

    int vendorID = 0x3333;
    int productID = 0x7856;
    CFNumberRef vidNumber =
        CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &vendorID);
    CFNumberRef pidNumber =
        CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &productID);

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