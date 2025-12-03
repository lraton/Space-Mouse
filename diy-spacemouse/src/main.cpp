#include <Arduino.h>
#include <OneButton.h>
#include <Tlv493d.h>
#include <SimpleKalmanFilter.h>
#include <Adafruit_TinyUSB.h>
#include <RP2040_PWM.h>
#include <Tween.h>
#include <pico/sleep.h>
#include <hardware/rosc.h>

static constexpr int SPACEMOUSE_PIN_BUTTON1 = 2;
static constexpr int SPACEMOUSE_PIN_BUTTON2 = 3;
[[maybe_unused]] static constexpr int SPACEMOUSE_PIN_SDA = 4;
[[maybe_unused]] static constexpr int SPACEMOUSE_PIN_SCL = 5;
static constexpr int SPACEMOUSE_PIN_LED1 = 28;
static constexpr int SPACEMOUSE_PIN_LED2 = 29;

static constexpr unsigned long LED_FADING_INACTIVITY_TIME = 1000 * 590;
static constexpr unsigned long MAX_INACTIVITY_TIME = LED_FADING_INACTIVITY_TIME + (1000 * 10);

static constexpr float PWM_FREQ = 1000;

static constexpr int MAGNETOMETER_SENSITIVITY = 10; // Increased from 10
static constexpr int MAGNETOMETER_INPUT_RANGE = 2.5 * MAGNETOMETER_SENSITIVITY;
static constexpr int MAGNETOMETER_INPUT_Z_RANGE = MAGNETOMETER_INPUT_RANGE * 8;
static constexpr float MAGNETOMETER_XY_THRESHOLD = 0.4;          // Reduced from 0.4
static constexpr float MAGNETOMETER_Z_THRESHOLD = 0.9;           // Reduced from 0.9
static constexpr float MAGNETOMETER_Z_ORBIT_MAX_THRESHOLD = 1.7; // Reduced from 1.7

static constexpr int HID_POLL_INTERVAL = 4;
static constexpr int HID_REPORT_LOGICAL_RANGE = 350;
static constexpr int HID_REPORT_PHYSICAL_RANGE = 1400;
static constexpr int HID_MAX_BUTTONS = 24;

static const uint8_t desc_hid_report[] = {
    HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),
    HID_USAGE(HID_USAGE_DESKTOP_MULTI_AXIS_CONTROLLER),
    HID_COLLECTION(HID_COLLECTION_APPLICATION),
    HID_COLLECTION(HID_COLLECTION_PHYSICAL),
    HID_REPORT_ID(1)
        HID_LOGICAL_MIN_N(-HID_REPORT_LOGICAL_RANGE, 2),
    HID_LOGICAL_MAX_N(HID_REPORT_LOGICAL_RANGE, 2),
    HID_PHYSICAL_MIN_N(-HID_REPORT_PHYSICAL_RANGE, 2),
    HID_PHYSICAL_MAX_N(HID_REPORT_PHYSICAL_RANGE, 2),
    HID_USAGE(HID_USAGE_DESKTOP_X),
    HID_USAGE(HID_USAGE_DESKTOP_Y),
    HID_USAGE(HID_USAGE_DESKTOP_Z),
    HID_REPORT_SIZE(16),
    HID_REPORT_COUNT(3),
    HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
    HID_COLLECTION_END,
    HID_COLLECTION(HID_COLLECTION_PHYSICAL),
    HID_REPORT_ID(2)
        HID_LOGICAL_MIN_N(-HID_REPORT_LOGICAL_RANGE, 2),
    HID_LOGICAL_MAX_N(HID_REPORT_LOGICAL_RANGE, 2),
    HID_PHYSICAL_MIN_N(-HID_REPORT_PHYSICAL_RANGE, 2),
    HID_PHYSICAL_MAX_N(HID_REPORT_PHYSICAL_RANGE, 2),
    HID_USAGE(HID_USAGE_DESKTOP_RX),
    HID_USAGE(HID_USAGE_DESKTOP_RY),
    HID_USAGE(HID_USAGE_DESKTOP_RZ),
    HID_REPORT_SIZE(16),
    HID_REPORT_COUNT(3),
    HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
    HID_COLLECTION_END,
    HID_COLLECTION(HID_COLLECTION_PHYSICAL),
    HID_REPORT_ID(3)
        HID_LOGICAL_MIN(0),
    HID_LOGICAL_MAX(1),
    HID_REPORT_SIZE(1),
    HID_REPORT_COUNT(HID_MAX_BUTTONS),
    HID_USAGE_PAGE(HID_USAGE_PAGE_BUTTON),
    HID_USAGE_MIN(1),
    HID_USAGE_MAX(HID_MAX_BUTTONS),
    HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
    HID_COLLECTION_END,
    HID_COLLECTION_END};

Tlv493d mag;
SimpleKalmanFilter xFilter(1, 1, 0.2);
SimpleKalmanFilter yFilter(1, 1, 0.2);
SimpleKalmanFilter zFilter(1, 1, 0.2);
OneButton button1(SPACEMOUSE_PIN_BUTTON1);
OneButton button2(SPACEMOUSE_PIN_BUTTON2);
RP2040_PWM led1Pwm(SPACEMOUSE_PIN_LED1, PWM_FREQ, 0, false);
RP2040_PWM led2Pwm(SPACEMOUSE_PIN_LED2, PWM_FREQ, 0, false);
Adafruit_USBD_HID usb_hid;
Tween::Timeline ledTimeline;
float nextLedLevel = 0;
uint8_t buttonsData[HID_MAX_BUTTONS / 8] = {};
bool freshButtonsData = false;
float xOffset = 0, yOffset = 0, zOffset = 0;
float xCurrent = 0, yCurrent = 0, zCurrent = 0;
float rxCurrent = 0, ryCurrent = 0, rzCurrent = 0;

void magnetometerSetup();
void setLeds(float dutyCycle);
void setButton(uint8_t bit, bool on);

void setup()
{
    // 1. Initialize USB first so PC sees it immediately
    TinyUSBDevice.setID(0x256f, 0xc631);
    TinyUSBDevice.setLanguageDescriptor(0x0409);
    TinyUSBDevice.setManufacturerDescriptor("3Dconnexion");
    TinyUSBDevice.setProductDescriptor("SpaceMouse Pro Wireless (cabled)");

    usb_hid.setPollInterval(HID_POLL_INTERVAL);
    usb_hid.setBootProtocol(HID_ITF_PROTOCOL_NONE);
    usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
    usb_hid.setStringDescriptor("SpaceMouse Pro Wireless (cabled)");
    usb_hid.begin();

    // 2. Wait briefly for mount, but timeout quickly so we don't hang
    long start = millis();
    while (!TinyUSBDevice.mounted())
    {
        delay(1);
        if (millis() - start > 1000)
            break; // Timeout after 1 second
    }

    // 3. Setup Hardware
    Serial.begin(115200); // Optional, non-blocking now
    magnetometerSetup();  // Configure I2C pins and sensor
    setLeds(100);

    // 4. Setup Buttons
    button1.attachClick([]
                        { setButton(0, true); });
    button2.attachClick([]
                        { setButton(1, true); });
    button1.attachLongPressStart([]
                                 { setButton(2, true); });
    button2.attachLongPressStart([]
                                 { setButton(3, true); });
    button1.attachIdle([]
                       { setButton( 0, false ); setButton( 2, false ); });
    button2.attachIdle([]
                       { setButton( 1, false ); setButton( 3, false ); });

    // 5. Setup LEDs
    ledTimeline.mode(Tween::Mode::REPEAT_TL);
    ledTimeline.add(nextLedLevel)
        .init(100)
        .then<Ease::SineInOut>(20, 1000)
        .hold(500)
        .then<Ease::SineInOut>(100, 1000)
        .hold(500);
}

void setLeds(float dutyCycle)
{
    led1Pwm.setPWM(led1Pwm.getPin(), PWM_FREQ, dutyCycle);
    led2Pwm.setPWM(led2Pwm.getPin(), PWM_FREQ, dutyCycle);
}

void setButton(uint8_t bit, bool on)
{
    if (on)
        buttonsData[0] |= 1 << bit;
    else
        buttonsData[0] &= ~(1 << bit);

    freshButtonsData = true;
}

void magnetometerSetup()
{
    static constexpr int CALIBRATION_SAMPLES = 300;

    Wire.setSDA(SPACEMOUSE_PIN_SDA);
    Wire.setSCL(SPACEMOUSE_PIN_SCL);
    Wire.setClock(100000);
    Wire.begin();

    Wire.beginTransmission(0x5E);
    if (Wire.endTransmission() == 0)
    {
        Serial.println("Sensor FOUND at 0x5E");
    }

    // Initialize sensor (from your working code)
    Wire.beginTransmission(0x5E);
    Wire.write(0x00); // Control register
    Wire.write(0x01); // Activate sensor
    Wire.endTransmission();
    delay(100);

    Serial.println("Testing readings:");
    for (int i = 0; i < 5; i++)
    {
        delay(100);

        // Point to register 0x00 before reading
        Wire.beginTransmission(0x5E);
        Wire.write(0x00);
        Wire.endTransmission();

        Wire.requestFrom(0x5E, 6);
        if (Wire.available() == 6)
        {
            uint8_t data[6];
            for (int j = 0; j < 6; j++)
            {
                data[j] = Wire.read();
            }

            int16_t x = ((data[0] << 4) | (data[4] >> 4));
            int16_t y = ((data[1] << 4) | (data[4] & 0x0F));
            int16_t z = ((data[2] << 4) | (data[5] & 0x0F));

            if (x > 2047)
                x -= 4096;
            if (y > 2047)
                y -= 4096;
            if (z > 2047)
                z -= 4096;

            // Convert to mT (these are raw values, scale factor ~0.098 mT per LSB)
            float xVal = x * 0.098;
            float yVal = y * 0.098;
            float zVal = z * 0.098;

            Serial.print("X: ");
            Serial.print(xVal);
            Serial.print(" Y: ");
            Serial.print(yVal);
            Serial.print(" Z: ");
            Serial.println(zVal);
        }
    }

    // Calibration
    Serial.println("Calibrating...");
    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        delay(10);

        Wire.beginTransmission(0x5E);
        Wire.write(0x00);
        Wire.endTransmission();

        Wire.requestFrom(0x5E, 6);
        if (Wire.available() == 6)
        {
            uint8_t data[6];
            for (int j = 0; j < 6; j++)
            {
                data[j] = Wire.read();
            }

            int16_t x = ((data[0] << 4) | (data[4] >> 4));
            int16_t y = ((data[1] << 4) | (data[4] & 0x0F));
            int16_t z = ((data[2] << 4) | (data[5] & 0x0F));

            if (x > 2047)
                x -= 4096;
            if (y > 2047)
                y -= 4096;
            if (z > 2047)
                z -= 4096;

            xOffset += x * 0.098;
            yOffset += y * 0.098;
            zOffset += z * 0.098;
        }

        if ((i % 20) == 0)
            setLeds(100);
        else if ((i % 10) == 0)
            setLeds(50);
    }

    xOffset /= CALIBRATION_SAMPLES;
    yOffset /= CALIBRATION_SAMPLES;
    zOffset /= CALIBRATION_SAMPLES;

    Serial.print("Offsets - X: ");
    Serial.print(xOffset);
    Serial.print(" Y: ");
    Serial.print(yOffset);
    Serial.print(" Z: ");
    Serial.println(zOffset);
}

void readMagnetometer()
{
    static bool isOrbit = false;
    static unsigned long lastDebug = 0;
    float absZ;

    // Point to register 0x00 before reading
    Wire.beginTransmission(0x5E);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.requestFrom(0x5E, 6);
    if (Wire.available() == 6)
    {
        uint8_t data[6];
        for (int j = 0; j < 6; j++)
        {
            data[j] = Wire.read();
        }

        int16_t x = ((data[0] << 4) | (data[4] >> 4));
        int16_t y = ((data[1] << 4) | (data[4] & 0x0F));
        int16_t z = ((data[2] << 4) | (data[5] & 0x0F));

        if (x > 2047)
            x -= 4096;
        if (y > 2047)
            y -= 4096;
        if (z > 2047)
            z -= 4096;

        float xRaw = x * 0.098;
        float yRaw = y * 0.098;
        float zRaw = z * 0.098;

        xCurrent = xFilter.updateEstimate(xRaw - xOffset);
        yCurrent = yFilter.updateEstimate(yRaw - yOffset);
        zCurrent = zFilter.updateEstimate(zRaw - zOffset);
    }

    absZ = abs(zCurrent);

    // Debug output every 500ms
    if (millis() - lastDebug > 500)
    {
        if (xCurrent != 0 || yCurrent != 0 || zCurrent != 0)
        {
            Serial.print("Raw: X=");
            Serial.print(xCurrent);
            Serial.print(" Y=");
            Serial.print(yCurrent);
            Serial.print(" Z=");
            Serial.print(zCurrent);
            Serial.print(" | orbit=");
            Serial.println(isOrbit);
        }
        lastDebug = millis();
    }

    if (abs(xCurrent) < MAGNETOMETER_XY_THRESHOLD)
        xCurrent = 0;

    if (abs(yCurrent) < MAGNETOMETER_XY_THRESHOLD)
        yCurrent = 0;

    if (absZ < MAGNETOMETER_Z_THRESHOLD)
    {
        zCurrent = 0;
        absZ = 0;
    }
    if (isOrbit)
    {

        if (!xCurrent && !yCurrent && (!absZ || (zCurrent > 0) || (absZ > MAGNETOMETER_Z_ORBIT_MAX_THRESHOLD)))
            isOrbit = false;
    }
    else if (!xCurrent && !yCurrent)
        isOrbit = absZ && (zCurrent < 0) && (absZ <= MAGNETOMETER_Z_ORBIT_MAX_THRESHOLD);

    if (isOrbit)
    {

        ryCurrent = xCurrent;
        rxCurrent = yCurrent;
        rzCurrent = zCurrent;
        xCurrent = 0;
        yCurrent = 0;
        zCurrent = 0;
    }
    else
    {

        rxCurrent = 0;
        ryCurrent = 0;
        rzCurrent = 0;

        if (absZ)
        {

            xCurrent = 0;
            yCurrent = 0;

            if (zCurrent < 0)
                zCurrent += MAGNETOMETER_Z_ORBIT_MAX_THRESHOLD;
        }
    }
}

void sendHidReports()
{
    static unsigned long lastActivityMillis = 0;
    bool activity = false;
    auto x = static_cast<int16_t>(map(static_cast<long>(xCurrent * MAGNETOMETER_SENSITIVITY), -MAGNETOMETER_INPUT_RANGE, MAGNETOMETER_INPUT_RANGE, -HID_REPORT_PHYSICAL_RANGE, HID_REPORT_PHYSICAL_RANGE));
    auto y = -static_cast<int16_t>(map(static_cast<long>(yCurrent * MAGNETOMETER_SENSITIVITY), -MAGNETOMETER_INPUT_RANGE, MAGNETOMETER_INPUT_RANGE, -HID_REPORT_PHYSICAL_RANGE, HID_REPORT_PHYSICAL_RANGE));
    auto z = -static_cast<int16_t>(map(static_cast<long>(zCurrent * MAGNETOMETER_SENSITIVITY), -MAGNETOMETER_INPUT_Z_RANGE, MAGNETOMETER_INPUT_Z_RANGE, -HID_REPORT_PHYSICAL_RANGE, HID_REPORT_PHYSICAL_RANGE));
    uint8_t trans[] = {
        static_cast<uint8_t>(x & 0xFF), static_cast<uint8_t>(x >> 8),
        static_cast<uint8_t>(y & 0xFF), static_cast<uint8_t>(y >> 8),
        static_cast<uint8_t>(z & 0xFF), static_cast<uint8_t>(z >> 8)};
    auto rx = static_cast<int16_t>(map(static_cast<long>(rxCurrent * MAGNETOMETER_SENSITIVITY), -MAGNETOMETER_INPUT_RANGE, MAGNETOMETER_INPUT_RANGE, -HID_REPORT_PHYSICAL_RANGE, HID_REPORT_PHYSICAL_RANGE));
    auto ry = -static_cast<int16_t>(map(static_cast<long>(ryCurrent * MAGNETOMETER_SENSITIVITY), -MAGNETOMETER_INPUT_RANGE, MAGNETOMETER_INPUT_RANGE, -HID_REPORT_PHYSICAL_RANGE, HID_REPORT_PHYSICAL_RANGE));
    auto rz = -static_cast<int16_t>(map(static_cast<long>(rzCurrent * MAGNETOMETER_SENSITIVITY), -MAGNETOMETER_INPUT_Z_RANGE, MAGNETOMETER_INPUT_Z_RANGE, -HID_REPORT_PHYSICAL_RANGE, HID_REPORT_PHYSICAL_RANGE));
    uint8_t rot[] = {
        static_cast<uint8_t>(rx & 0xFF), static_cast<uint8_t>(rx >> 8),
        static_cast<uint8_t>(ry & 0xFF), static_cast<uint8_t>(ry >> 8),
        static_cast<uint8_t>(rz & 0xFF), static_cast<uint8_t>(rz >> 8)};

    usb_hid.sendReport(1, trans, sizeof(trans));
    delay(HID_POLL_INTERVAL);
    usb_hid.sendReport(2, rot, sizeof(rot));

    if (x || y || z)
        activity = true;

    if (freshButtonsData)
    {
        delay(HID_POLL_INTERVAL);
        usb_hid.sendReport(3, buttonsData, sizeof(buttonsData));
        freshButtonsData = false;
        activity = true;
    }

    if (activity)
    {

        if (TinyUSBDevice.suspended())
            TinyUSBDevice.remoteWakeup();

        lastActivityMillis = millis();

        if (ledTimeline.isRunning())
        {
            ledTimeline.stop();
            setLeds(100);
        }
    }
    else
    {
        unsigned long dt = millis() - lastActivityMillis;

        if (dt > MAX_INACTIVITY_TIME)
        {
            Tween::Timeline fadeOut;

            ledTimeline.stop();

            fadeOut.add(nextLedLevel)
                .init(nextLedLevel)
                .then<Ease::SineOut>(0, 500);

            fadeOut.start();

            while (fadeOut.isRunning())
            {
                fadeOut.update();
                setLeds(nextLedLevel);
            }

            sleep_run_from_rosc();
            sleep_goto_dormant_until_pin(SPACEMOUSE_PIN_BUTTON1, true, false);
            rp2040.restart();
        }
        else if ((dt > LED_FADING_INACTIVITY_TIME) && !ledTimeline.isRunning())
            ledTimeline.start();
    }
}

void loop()
{
#ifdef TINYUSB_NEED_POLLING_TASK
    // Manual call tud_task since it isn't called by Core's background
    TinyUSBDevice.task();
#endif

    if (ledTimeline.isRunning())
    {
        ledTimeline.update();
        setLeds(nextLedLevel);
    }

    readMagnetometer();

    if (usb_hid.ready())
    {
        button1.tick();
        button2.tick();
        sendHidReports();
    }

    delay(10); // Start with 10ms, adjust if needed
}
