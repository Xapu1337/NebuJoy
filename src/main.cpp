#include <Arduino.h>
#include <Adafruit_TinyUSB.h>

#define JOYSTICK_COUNT 4
#define DOF 6
#define FILTER_SIZE 10  // Size of the moving average filter
#define DEAD_THRES 1
#define SPEED_PARAM 600

// Debug levels
enum DebugLevel {
    DEBUG_OFF,
    DEBUG_RAW,
    DEBUG_CENTERED,
    DEBUG_DEADZONE,
    DEBUG_MOTION,
    DEBUG_COMBINED
};

int debug = DEBUG_COMBINED;

bool invX = false;
bool invY = false;
bool invZ = false;
bool invRX = false;
bool invRY = true;
bool invRZ = true;

int16_t speed = 40; // Reduced speed for less bouncy movement
int DEADZONE = 10;

class Joystick {
public:
    Joystick(int xPin, int yPin) : xPin(xPin), yPin(yPin), filterIndex(0) {
        pinMode(xPin, INPUT);
        pinMode(yPin, INPUT);
        for (int i = 0; i < FILTER_SIZE; i++) {
            xValues[i] = 0;
            yValues[i] = 0;
        }
    }

    void readData() {
        int x = analogRead(xPin);
        int y = analogRead(yPin);

        // Update the filter buffers
        xValues[filterIndex] = x;
        yValues[filterIndex] = y;

        // Move to the next position in the buffer
        filterIndex = (filterIndex + 1) % FILTER_SIZE;

        // Calculate the average
        xValue = average(xValues, FILTER_SIZE);
        yValue = average(yValues, FILTER_SIZE);
    }

    void setCenter() {
        centerPosX = xValue;
        centerPosY = yValue;
    }

    int getX() const {
        return xValue - centerPosX;
    }

    int getY() const {
        return yValue - centerPosY;
    }

private:
    int xPin;
    int yPin;
    int xValue;
    int yValue;
    int centerPosX;
    int centerPosY;
    int filterIndex;
    int xValues[FILTER_SIZE];
    int yValues[FILTER_SIZE];

    int average(int* values, int size) const {
        long sum = 0;
        for (int i = 0; i < size; i++) {
            sum += values[i];
        }
        return sum / size;
    }
};

class Motion {
public:
    Motion(int transX, int transY, int transZ, int rotX, int rotY, int rotZ) 
        : transX(transX), transY(transY), transZ(transZ), rotX(rotX), rotY(rotY), rotZ(rotZ) {}

    int transX;
    int transY;
    int transZ;
    int rotX;
    int rotY;
    int rotZ;
};

Joystick joysticks[JOYSTICK_COUNT] = {
    Joystick(16, 18),
    Joystick(3, 5),
    Joystick(11, 12),
    Joystick(7, 9)
};

Adafruit_USBD_HID usb_hid;

static const uint8_t _hidReportDescriptor[] PROGMEM = {
    0x05, 0x01,           //  Usage Page (Generic Desktop)
    0x09, 0x08,           //  0x08: Usage (Multi-Axis)
    0xa1, 0x01,           //  Collection (Application)
    0xa1, 0x00,           // Collection (Physical)
    0x85, 0x01,           //  Report ID
    0x16, 0x00, 0x80,     //logical minimum (-500)
    0x26, 0xff, 0x7f,     //logical maximum (500)
    0x36, 0x00, 0x80,     //Physical Minimum (-32768)
    0x46, 0xff, 0x7f,     //Physical Maximum (32767)
    0x09, 0x30,           //    Usage (X)
    0x09, 0x31,           //    Usage (Y)
    0x09, 0x32,           //    Usage (Z)
    0x75, 0x10,           //    Report Size (16)
    0x95, 0x03,           //    Report Count (3)
    0x81, 0x02,           //    Input (variable,absolute)
    0xC0,                 //  End Collection
    0xa1, 0x00,           // Collection (Physical)
    0x85, 0x02,           //  Report ID
    0x16, 0x00, 0x80,     //logical minimum (-500)
    0x26, 0xff, 0x7f,     //logical maximum (500)
    0x36, 0x00, 0x80,     //Physical Minimum (-32768)
    0x46, 0xff, 0x7f,     //Physical Maximum (32767)
    0x09, 0x33,           //    Usage (RX)
    0x09, 0x34,           //    Usage (RY)
    0x09, 0x35,           //    Usage (RZ)
    0x75, 0x10,           //    Report Size (16)
    0x95, 0x03,           //    Report Count (3)
    0x81, 0x02,           //    Input (variable,absolute)
    0xC0,                 //  End Collection
    0xC0
};

void setup() {
    Serial.begin(115200);

    USB.PID(0xc631);
    USB.VID(0x256f);
    USB.begin();
    TinyUSBDevice.setManufacturerDescriptor("3DCONNEXOR");
    TinyUSBDevice.setProductDescriptor("SpaceMouse Pro Wireless");
    TinyUSBDevice.setID(0x256f, 0xc631);
    usb_hid.setPollInterval(2);
    usb_hid.setReportDescriptor(_hidReportDescriptor, sizeof(_hidReportDescriptor));
    usb_hid.begin();

    while (!TinyUSBDevice.mounted()) delay(1);

    for (int i = 0; i < JOYSTICK_COUNT; i++) {
        joysticks[i].readData();
        joysticks[i].setCenter();
    }
}

void send_command(int16_t rx, int16_t ry, int16_t rz, int16_t x, int16_t y, int16_t z) {
    uint8_t trans[6] = {x & 0xFF, x >> 8, y & 0xFF, y >> 8, z & 0xFF, z >> 8};
    usb_hid.sendReport(1, trans, 6);
    uint8_t rot[6] = {rx & 0xFF, rx >> 8, ry & 0xFF, ry >> 8, rz & 0xFF, rz >> 8};
    usb_hid.sendReport(2, rot, 6);
}

Motion calculateMotion() {
    int16_t centered[JOYSTICK_COUNT * 2];
    int16_t currentReads[JOYSTICK_COUNT * 2];

    bool isCentered = true; // Flag to check if all joysticks are at the center

    for (int i = 0; i < JOYSTICK_COUNT; i++) {
        joysticks[i].readData();
        currentReads[2 * i] = joysticks[i].getX();
        currentReads[2 * i + 1] = joysticks[i].getY();

// Check if any joystick is not at the center
        if (currentReads[2 * i] != 0 || currentReads[2 * i + 1] != 0) {
            isCentered = false;
        }
    }

    if (isCentered) {
        // If all joysticks are at the center, return zero motion
        return Motion(0, 0, 0, 0, 0, 0);
    }

    for (int i = 0; i < JOYSTICK_COUNT * 2; i++) {
        centered[i] = currentReads[i];
        if (centered[i] < DEADZONE && centered[i] > -DEADZONE) centered[i] = 0;
    }

    Motion motion(
        (-centered[0] + centered[2]) / 2, // transX
        (centered[3] - centered[6]) / 2, // transY (adjusted)
        (-centered[0] - centered[2] - centered[4] - centered[6]) / 2, // transZ
        (-centered[1] + centered[5]) / 2, // rotX
        (-centered[0] + centered[4]) / 2, // rotY
        (centered[1] + centered[3] + centered[5] + centered[7]) / 4 // rotZ
    );

    motion.transX = motion.transX * speed / 100;
    motion.transY = motion.transY * speed / 100;
    motion.transZ = motion.transZ * speed / 100;
    motion.rotX = motion.rotX * speed / 100;
    motion.rotY = motion.rotY * speed / 100;
    motion.rotZ = motion.rotZ * speed / 100;

    if (invX) motion.transX *= -1;
    if (invY) motion.transY *= -1;
    if (invZ) motion.transZ *= -1;
    if (invRX) motion.rotX *= -1;
    if (invRY) motion.rotY *= -1;
    if (invRZ) motion.rotZ *= -1;

    if (debug >= DEBUG_MOTION) {
        Serial.print("Trans: X=");
        Serial.print(motion.transX);
        Serial.print(" Y=");
        Serial.print(motion.transY);
        Serial.print(" Z=");
        Serial.println(motion.transZ);
        Serial.print("Rot: X=");
        Serial.print(motion.rotX);
        Serial.print(" Y=");
        Serial.print(motion.rotY);
        Serial.print(" Z=");
        Serial.println(motion.rotZ);
    }

    return motion;
}

void loop() {
    if (!usb_hid.ready()) return;

    Motion motion = calculateMotion();
    send_command(motion.rotX, motion.rotY, motion.rotZ, motion.transX, motion.transY, motion.transZ);

    delay(20);
}