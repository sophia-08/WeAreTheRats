#include <iostream>
#include <chrono>
#include <thread>
#include <random>
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
    std::cout << "Starting enhanced input simulator. Press Ctrl+C to exit." << std::endl;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(0, 2);

    while (true) {
        int action = distrib(gen);
        switch(action) {
            case 0:
                std::cout << "Moving mouse" << std::endl;
                moveMouse(50, 50);
                break;
            case 1:
                std::cout << "Clicking mouse" << std::endl;
                clickMouse();
                break;
            case 2:
                std::cout << "Pressing 'A' key" << std::endl;
                pressKey(0); // 'A' key
                break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(30));
    }

    return 0;
}