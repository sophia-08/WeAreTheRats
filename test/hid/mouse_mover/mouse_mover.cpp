#include <iostream>
#include <chrono>
#include <thread>
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

int main() {
    std::cout << "Starting mouse mover. Press Ctrl+C to exit." << std::endl;

    while (true) {
        moveMouse(50, 50);
        std::this_thread::sleep_for(std::chrono::seconds(30));
    }

    return 0;
}