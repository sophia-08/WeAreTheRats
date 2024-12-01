#include <iostream>
#include <chrono>
#include <thread>

// Workaround for GCC compatibility issues
// #define __AVAILABILITY_INTERNAL__MAC_10_2_DEP__MAC_10_15
// #define __AVAILABILITY_INTERNAL__MAC_11_3
// #define CF_ENUM_AVAILABLE(...)
// #define CF_ENUM_DEPRECATED(...)

#include <ApplicationServices/ApplicationServices.h>

void moveMouse(int dx, int dy) {
    CGEventRef event = CGEventCreate(nullptr);
    CGPoint currentPos = CGEventGetLocation(event);
    CFRelease(event);

    CGPoint newPos = CGPointMake(currentPos.x + dx, currentPos.y + dy);
    CGEventRef moveEvent = CGEventCreateMouseEvent(
        nullptr, kCGEventMouseMoved, newPos, kCGMouseButtonLeft
    );

    CGEventPost(kCGHIDEventTap, moveEvent);
    CFRelease(moveEvent);
}

void clickMouse() {
    CGPoint currentPos;
    CGEventRef event = CGEventCreate(nullptr);
    currentPos = CGEventGetLocation(event);
    CFRelease(event);

    CGEventRef clickDown = CGEventCreateMouseEvent(
        nullptr, kCGEventLeftMouseDown, currentPos, kCGMouseButtonLeft
    );
    CGEventRef clickUp = CGEventCreateMouseEvent(
        nullptr, kCGEventLeftMouseUp, currentPos, kCGMouseButtonLeft
    );

    CGEventPost(kCGHIDEventTap, clickDown);
    CGEventPost(kCGHIDEventTap, clickUp);

    CFRelease(clickDown);
    CFRelease(clickUp);
}

void pressKey(CGKeyCode keyCode) {
    CGEventRef keyDown = CGEventCreateKeyboardEvent(nullptr, keyCode, true);
    CGEventRef keyUp = CGEventCreateKeyboardEvent(nullptr, keyCode, false);

    CGEventPost(kCGHIDEventTap, keyDown);
    CGEventPost(kCGHIDEventTap, keyUp);

    CFRelease(keyDown);
    CFRelease(keyUp);
}

int main() {
    std::cout << "Starting input simulator. Press Ctrl+C to exit." << std::endl;

    while (true) {
        moveMouse(50, 50);
        std::this_thread::sleep_for(std::chrono::seconds(10));
        
        clickMouse();
        std::this_thread::sleep_for(std::chrono::seconds(10));
        
        pressKey(0); // 'A' key
        std::this_thread::sleep_for(std::chrono::seconds(10));
    }

    return 0;
}