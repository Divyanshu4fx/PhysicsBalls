#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// Pin definitions
#define TFT_SCLK 14  // SCL
#define TFT_MOSI 13  // SDA
#define TFT_RST  12  // RES (RESET)
#define TFT_DC   2   // Data Command control pin
#define TFT_CS   15  // Chip select control pin

// Color definitions
const uint16_t Display_Color_Black = 0x0000;
const uint16_t Display_Color_White = 0xFFFF;

// Display initialization
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Screen dimensions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 160

// Physics constants
float gravityX = 0.0;
float gravityY = -9.8;
const float timeStep = 1.0 / 30.0;
const int numBalls = 500;
const float restitution = 0.20;
const float GRAVITY_SCALE = 1; 

// Double buffering setup
static uint16_t frameBuffer[2][SCREEN_WIDTH * SCREEN_HEIGHT];
bool currentBuffer = 0;

// Vertical sync control
volatile bool vSync = false;
const int vsyncPeriod = 33; // ~30fps in milliseconds

// Task handles and synchronization
SemaphoreHandle_t displayMutex;
TaskHandle_t physicsTask;
TaskHandle_t displayTask;
TaskHandle_t sensorTask;

// Vector structure
struct Vector {
    float x, y;
    
    Vector(float _x = 0.0, float _y = 0.0) : x(_x), y(_y) {}
    
    void add(const Vector& v, float scale = 1.0) {
        x += v.x * scale;
        y += v.y * scale;
    }
    
    float length() const {
        return sqrt(x * x + y * y);
    }
    
    void scale(float s) {
        x *= s;
        y *= s;
    }
    
    float dot(const Vector& v) const {
        return x * v.x + y * v.y;
    }
};

// Ball structure
struct Ball {
    float mass;
    Vector pos, vel;
};

// Global balls array
Ball balls[numBalls];

// Helper function to set pixel in buffer
inline void setPixel(int x, int y, uint16_t color, uint16_t* buffer) {
    if (x >= 0 && x < SCREEN_WIDTH && y >= 0 && y < SCREEN_HEIGHT) {
        buffer[y * SCREEN_WIDTH + x] = color;
    }
}

// Sensor task to update gravity values
void sensorLoop(void * parameter) {
    while(true) {
        sensors_event_t event;
        accel.getEvent(&event);
        
        // Update gravity values based on accelerometer readings
        // Note: We swap and invert axes to match screen orientation
        gravityX = event.acceleration.x * GRAVITY_SCALE;
        gravityY = -event.acceleration.y * GRAVITY_SCALE;
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Update every 10ms
    }
}

// Physics simulation functions
void handleCollision(Ball& b1, Ball& b2) {
    Vector dir = {b2.pos.x - b1.pos.x, b2.pos.y - b1.pos.y};
    float dist = dir.length();
    if (dist == 0.0 || dist > 2.0) return;

    dir.scale(1.0 / dist);
    float overlap = (2.0 - dist) / 2;
    b1.pos.add(dir, -overlap);
    b2.pos.add(dir, overlap);

    float v1 = b1.vel.dot(dir);
    float v2 = b2.vel.dot(dir);
    float m1 = b1.mass;
    float m2 = b2.mass;

    float newV1 = (m1 * v1 + m2 * v2 - m2 * restitution * (v1 - v2)) / (m1 + m2);
    float newV2 = (m1 * v1 + m2 * v2 - m1 * restitution * (v2 - v1)) / (m1 + m2);

    b1.vel.add(dir, newV1 - v1);
    b2.vel.add(dir, newV2 - v2);
}

void simulate(Ball& b) {
    // Use current gravity values from accelerometer
    b.vel.add({gravityX, gravityY}, timeStep);
    b.pos.add(b.vel, timeStep);

    // Boundary collision handling with velocity dampening
    if (b.pos.x < 1) {
        b.pos.x = 1;
        b.vel.x = -b.vel.x * restitution;
        b.vel.y *= 0.95;
    }
    if (b.pos.x > SCREEN_WIDTH - 2) {
        b.pos.x = SCREEN_WIDTH - 2;
        b.vel.x = -b.vel.x * restitution;
        b.vel.y *= 0.95;
    }
    if (b.pos.y < 1) {
        b.pos.y = 1;
        b.vel.y = -b.vel.y * restitution;
        b.vel.x *= 0.95;
    }
    if (b.pos.y > SCREEN_HEIGHT - 2) {
        b.pos.y = SCREEN_HEIGHT - 2;
        b.vel.y = -b.vel.y * restitution;
        b.vel.x *= 0.95;
    }
}

// Display update function
void updateDisplay(uint16_t* buffer) {
    tft.setAddrWindow(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1);
    
    SPI.beginTransaction(SPISettings(80000000, MSBFIRST, SPI_MODE0));
    digitalWrite(TFT_DC, HIGH);
    digitalWrite(TFT_CS, LOW);
    
    const int chunkSize = 2048;
    uint8_t* byteBuffer = (uint8_t*)buffer;
    int totalBytes = SCREEN_WIDTH * SCREEN_HEIGHT * 2;
    
    for(int i = 0; i < totalBytes; i += chunkSize) {
        int bytesToSend = min(chunkSize, totalBytes - i);
        SPI.writeBytes(&byteBuffer[i], bytesToSend);
    }
    
    digitalWrite(TFT_CS, HIGH);
    SPI.endTransaction();
}

// Physics task
void physicsLoop(void * parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(33);
    
    while(true) {
        for (int i = 0; i < numBalls; i++) {
            simulate(balls[i]);
            
            for (int j = i + 1; j < numBalls; j++) {
                float dx = balls[i].pos.x - balls[j].pos.x;
                float dy = balls[i].pos.y - balls[j].pos.y;
                if (dx * dx + dy * dy < 16) {
                    handleCollision(balls[i], balls[j]);
                }
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Display task
void displayLoop(void * parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(vsyncPeriod);
    
    while(true) {
        if (xSemaphoreTake(displayMutex, portMAX_DELAY)) {
            vSync = true;
            
            uint16_t* backBuffer = frameBuffer[!currentBuffer];
            memset(backBuffer, 0, SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(uint16_t));
            
            // Draw all balls
            for (int i = 0; i < numBalls; i++) {
                int x = (int)balls[i].pos.x;
                int y = (int)balls[i].pos.y;
                setPixel(x, y, Display_Color_White, backBuffer);
                setPixel(x+1, y, Display_Color_White, backBuffer);
                setPixel(x, y+1, Display_Color_White, backBuffer);
                setPixel(x+1, y+1, Display_Color_White, backBuffer);
            }
            
            updateDisplay(backBuffer);
            currentBuffer = !currentBuffer;
            vSync = false;
            
            xSemaphoreGive(displayMutex);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup() {
    Serial.begin(115200);
    
    // Initialize accelerometer
    if(!accel.begin()) {
        Serial.println("No ADXL345 detected!");
        while(1);
    }
    
    // Set accelerometer range
    accel.setRange(ADXL345_RANGE_16_G);
    
    // Initialize display
    tft.initR(INITR_GREENTAB);
    tft.setRotation(2);
    tft.fillScreen(Display_Color_Black);
    
    // Optimize SPI settings
    SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
    SPI.setFrequency(80000000);
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setHwCs(true);
    
    // Initialize frame buffers
    memset(frameBuffer[0], 0, SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(uint16_t));
    memset(frameBuffer[1], 0, SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(uint16_t));
    
    // Initialize balls
    for (int i = 0; i < numBalls; i++) {
        balls[i].mass = 1.0;
        balls[i].pos = {
            (float)random(2, SCREEN_WIDTH - 3),
            (float)random(2, SCREEN_HEIGHT - 3)
        };
        balls[i].vel = {
            random(-50, 50) / 10.0f,
            random(-50, 50) / 10.0f
        };
    }
    
    // Create mutex
    displayMutex = xSemaphoreCreateMutex();
    
    // Create tasks
    xTaskCreatePinnedToCore(
        physicsLoop,
        "PhysicsTask",
        8192,
        NULL,
        2,
        &physicsTask,
        0
    );
    
    xTaskCreatePinnedToCore(
        displayLoop,
        "DisplayTask",
        8192,
        NULL,
        1,
        &displayTask,
        1
    );
    
    xTaskCreatePinnedToCore(
        sensorLoop,
        "SensorTask",
        4096,
        NULL,
        2,
        &sensorTask,
        0
    );
}

void loop() {
    vTaskDelay(1000);
}