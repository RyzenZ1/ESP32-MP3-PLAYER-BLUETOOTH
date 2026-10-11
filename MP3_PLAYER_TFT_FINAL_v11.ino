/*
 * ESP32 Wireless Music Player
 * ST7735 TFT + Analog Joystick + MicroSD + Bluetooth A2DP
 *
 * v11 — screensaver + backlight dimming (on top of v9.1 / A2DPResampler)
 *
 * Idle 15 s -> backlight dims.  Idle 30 s -> bouncing-logo screensaver.
 * Idle 2 min -> backlight off.  Any joystick input wakes it (the first
 * input after the screensaver only wakes, it doesn't trigger an action).
 * Timings and brightness levels are #defines in the POWER SAVING block.
 * Audio path is unchanged.  Keep A2DPResampler.h in the sketch folder.
 *
 * All audio is resampled to 48000 Hz stereo before being sent to
 * the BT A2DP sink.  The JBL Tune 770NC negotiates 48 kHz, and the
 * ESP32's SBC encoder outputs whatever rate the sink requests.
 * Without resampling, 44.1 kHz source files play 8.8% fast because
 * they are decoded at 48 kHz.  The resampler fixes this.
 *
 * Files on the SD card should be 44.1 kHz stereo (verified with
 * ffprobe).  The resampler handles any source rate as a fallback.
 */

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SD.h>
#include <BluetoothA2DPSource.h>
#include "MP3DecoderHelix.h"
#include "A2DPResampler.h"
#include <vector>
#include <algorithm>
#include <strings.h>

using namespace libhelix;

static SPIClass sdSPI(HSPI);

// ============================================================
//  PINS
// ============================================================
#define TFT_CS   15
#define TFT_RST   4
#define TFT_DC    2
#define TFT_LED  22

#define SD_CS     5
#define SD_MOSI  13
#define SD_MISO  27
#define SD_SCK   14

#define JOY_X    34
#define JOY_Y    35
#define JOY_SW   25

// ============================================================
//  JOYSTICK CONFIG
// ============================================================
#define JOY_CENTER_X    2048
#define JOY_CENTER_Y    2048
#define JOY_DEADZONE     900
#define JOY_REPEAT_MS    300
#define JOY_STABLE_READS   3
#define JOY_LOCKOUT_MS   600
#define JOY_HOLD_MS      800

enum JoyDir { JOY_NONE, JOY_UP, JOY_DOWN, JOY_LEFT, JOY_RIGHT, JOY_PRESS, JOY_HOLD };
static JoyDir        lastJoyDir       = JOY_NONE;
static unsigned long lastJoyActionMs  = 0;
static unsigned long joyLockedUntil   = 0;
static bool          waitingForCenter = false;

// ============================================================
//  VOLUME SCALE
// ============================================================
#define VOL_MAX     127
#define VOL_STEP      8

// ============================================================
//  SINK RATE
//  The JBL Tune 770NC runs at 48000 Hz.  Set this to 44100 if
//  you swap to headphones that negotiate 44.1 kHz.
// ============================================================
#define SINK_RATE_HZ 44900.0f

// ============================================================
//  DISPLAY
// ============================================================
Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);

#define TFT_W 160
#define TFT_H 128

// ============================================================
//  CYBERPUNK PALETTE
// ============================================================
#define C_BLACK      0x0000
#define C_BG         0x0000
#define C_SURFACE    0x10C3
#define C_SURFACE2   0x18E4
#define C_EDGE       0x2945

#define C_CYAN       0x07FF
#define C_CYAN_DIM   0x03EF
#define C_MAGENTA    0xF81F
#define C_MAG_DIM    0xA80F
#define C_LIME       0x07E0
#define C_LIME_DIM   0x0400
#define C_ORANGE     0xFD20
#define C_YELLOW     0xFFE0

#define C_WHITE      0xFFFF
#define C_LTGREY     0xC618
#define C_GREY       0x8410
#define C_DKGREY     0x4208
#define C_RED        0xF800

// ============================================================
//  MUSIC ROOT & GENRES
// ============================================================
const String MUSIC_ROOT      = "/Music MP3";
const String GENRE_FOLDERS[] = { "Mentalist Edits Songs", "Phonk", "Normal" };
const int    NUM_GENRES      = 3;

// ============================================================
//  AUDIO GLOBALS
// ============================================================
BluetoothA2DPSource a2dp_source;
A2DPResampler       resampler;

volatile uint32_t      sampleRate      = 44100;
volatile uint16_t      channels        = 2;
volatile unsigned long bytesDecoded    = 0;
volatile bool          isPlaying       = false;
volatile bool          isPaused        = false;
volatile bool          feedTaskRunning = false;
volatile unsigned long mp3BytesRead    = 0;
volatile unsigned long fileSize        = 0;
volatile bool          songFinished    = false;

#define FADE_FULL  256
#define FADE_STEP    2
static int fadeLevel  = 0;
static int fadeTarget = 0;

static SemaphoreHandle_t decoderMutex   = NULL;
static SemaphoreHandle_t spiMutex       = NULL;
static SemaphoreHandle_t btMutex        = NULL;
static TaskHandle_t      feedTaskHandle = NULL;

// ============================================================
//  BT SCANNER STATE
// ============================================================
struct BtDevice { String name; int rssi; };
static std::vector<BtDevice> btDevices;
static volatile bool         btConnected    = false;
static int                   btPickerScroll = 0;
static int                   btPickerSel    = -1;

static char chosenDeviceName[40] = {0};
static int  lastBtCount          = -1;

// ============================================================
//  SCREEN / FILE / PLAYBACK STATE
// ============================================================
enum ScreenState { BT_PICKER, GENRE_SELECT, FILE_LIST, PLAYER };
static ScreenState currentScreen = BT_PICKER;

struct DirEntry { String name; bool isDir; };
static std::vector<DirEntry> currentEntries;
static String                currentDir      = MUSIC_ROOT + "/";
static int                   listScroll      = 0;
static int                   listCursor      = 0;
static int                   genreIndex      = 0;
static int                   currentTrackIdx = -1;

static File          musicFile;
static String        currentFilePath = "";
static unsigned long playStartMs     = 0;

static int           titleScrollPx = 0;
static unsigned long lastScrollMs  = 0;
#define TITLE_SCROLL_MS   35
#define TITLE_PAUSE_MS  2000

static bool playerNeedsFullRedraw = true;
static bool headBlinkOn = true;

// ============================================================
//  FORWARD DECLARATIONS
// ============================================================
void listCurrentDir();
void playFile(const String &fullPath, int trackIdx);
void nextSong();
void prevSong();
void stopPlayback();
void startFeedTask();
void stopFeedTask();
void drawBtPicker();
void drawGenreSelect();
void drawFileList();
void drawPlayerFull();
void drawPlayerData();
void drawPlayerScreen();
void handleJoystick();
JoyDir readJoystick();
JoyDir readJoystickStable();
void rewindTenSeconds();

#define SPI_TAKE() xSemaphoreTake(spiMutex, portMAX_DELAY)
#define SPI_GIVE() xSemaphoreGive(spiMutex)

// ============================================================
//  POWER SAVING: BACKLIGHT DIMMING + SCREENSAVER
//
//  Idle timeline (measured from the last joystick input):
//    DIM_AFTER_MS         -> backlight fades down to BL_DIM
//    SAVER_AFTER_MS       -> screen goes black, a small logo bounces
//                            around at BL_SAVER brightness
//    SCREEN_OFF_AFTER_MS  -> backlight fully off (set to 0 to disable)
//
//  Any joystick input brings the backlight back up.  If the
//  screensaver or screen-off stage was active, that first input
//  ONLY wakes the display (it is swallowed, so you can't skip a
//  song or change volume by accident).  Music is never touched.
//
//  Needs the TFT backlight pin to be switchable.  If your module's
//  LED pin is wired straight to 3.3V instead of GPIO 22, dimming
//  has no effect (the screensaver animation still works).
// ============================================================
#define BL_PWM_FREQ            5000
#define BL_PWM_BITS               8
#define BL_FULL                 255
#define BL_DIM                   45
#define BL_SAVER                 28

#define DIM_AFTER_MS         15000UL
#define SAVER_AFTER_MS       30000UL
#define SCREEN_OFF_AFTER_MS 120000UL   // 0 = never fully turn off

enum PowerStage { PWR_ACTIVE, PWR_DIM, PWR_SAVER, PWR_SCREEN_OFF };
static PowerStage    pwrStage       = PWR_ACTIVE;
static unsigned long lastActivityMs = 0;
static int           blLevel        = BL_FULL;

// True while the screensaver / blank screen owns the display.
// Every draw path that can fire without user input checks this.
static inline bool isSaverActive() { return pwrStage >= PWR_SAVER; }

static void setBacklight(int level) {
    if (level < 0)   level = 0;
    if (level > 255) level = 255;
    blLevel = level;
    ledcWrite(TFT_LED, level);
}

static void fadeBacklight(int target, int totalMs) {
    const int steps = 12;
    int start = blLevel;
    for (int i = 1; i <= steps; i++) {
        setBacklight(start + (target - start) * i / steps);
        delay(totalMs / steps);
    }
}

// ---- bouncing logo ----
#define LOGO_W          52
#define LOGO_H          26
#define SAVER_STEP_MS   40
#define SAVER_SPEED      2

static int  logoX = 20, logoY = 20;
static int  logoDX = SAVER_SPEED, logoDY = SAVER_SPEED;
static int  logoColorIdx = 0;
static unsigned long lastSaverFrameMs = 0;
static const uint16_t LOGO_COLORS[] = { C_CYAN, C_MAGENTA, C_LIME, C_YELLOW, C_ORANGE };
#define NUM_LOGO_COLORS 5

static void drawSaverLogo(int x, int y, uint16_t col) {
    tft.fillRoundRect(x, y, LOGO_W, LOGO_H, 4, C_BG);
    tft.drawRoundRect(x, y, LOGO_W, LOGO_H, 4, col);
    tft.setTextSize(1);
    tft.setTextColor(col);
    tft.setCursor(x + 11, y + 5);  tft.print("ESP32");
    tft.setCursor(x + 8,  y + 15); tft.print("PLAYER");
}

static void enterDim() {
    pwrStage = PWR_DIM;
    fadeBacklight(BL_DIM, 300);
}

static void enterScreensaver() {
    pwrStage = PWR_SAVER;               // set first so other draw paths stand down
    SPI_TAKE();
    tft.fillScreen(C_BG);
    logoX  = random(0, TFT_W - LOGO_W);
    logoY  = random(0, TFT_H - LOGO_H);
    logoDX = random(2) ? SAVER_SPEED : -SAVER_SPEED;
    logoDY = random(2) ? SAVER_SPEED : -SAVER_SPEED;
    drawSaverLogo(logoX, logoY, LOGO_COLORS[logoColorIdx]);
    SPI_GIVE();
    lastSaverFrameMs = millis();
    fadeBacklight(BL_SAVER, 200);
}

static void enterScreenOff() {
    pwrStage = PWR_SCREEN_OFF;
    SPI_TAKE();
    tft.fillScreen(C_BG);
    SPI_GIVE();
    fadeBacklight(0, 300);
}

static void animateScreensaver() {
    unsigned long now = millis();
    if (now - lastSaverFrameMs < SAVER_STEP_MS) return;
    lastSaverFrameMs = now;

    int ox = logoX, oy = logoY;
    logoX += logoDX;
    logoY += logoDY;

    bool bounced = false;
    if (logoX <= 0)                 { logoX = 0;                logoDX =  SAVER_SPEED; bounced = true; }
    else if (logoX >= TFT_W-LOGO_W) { logoX = TFT_W-LOGO_W;     logoDX = -SAVER_SPEED; bounced = true; }
    if (logoY <= 0)                 { logoY = 0;                logoDY =  SAVER_SPEED; bounced = true; }
    else if (logoY >= TFT_H-LOGO_H) { logoY = TFT_H-LOGO_H;     logoDY = -SAVER_SPEED; bounced = true; }
    if (bounced) logoColorIdx = (logoColorIdx + 1) % NUM_LOGO_COLORS;

    int dx = logoX - ox, dy = logoY - oy;

    SPI_TAKE();
    // Erase only the strips the logo just uncovered (no full-screen flicker)
    if (dx > 0)      tft.fillRect(ox,               oy, dx,  LOGO_H, C_BG);
    else if (dx < 0) tft.fillRect(ox + LOGO_W + dx, oy, -dx, LOGO_H, C_BG);
    if (dy > 0)      tft.fillRect(ox, oy,               LOGO_W, dy,  C_BG);
    else if (dy < 0) tft.fillRect(ox, oy + LOGO_H + dy, LOGO_W, -dy, C_BG);
    drawSaverLogo(logoX, logoY, LOGO_COLORS[logoColorIdx]);
    SPI_GIVE();
}

// Bring the display back to full brightness.
// redraw=true repaints the current screen first (while still dark),
// so you never see a flash of the screensaver before the UI returns.
static void wakeScreen(bool redraw) {
    bool wasSaver = isSaverActive();
    pwrStage       = PWR_ACTIVE;
    lastActivityMs = millis();

    if (wasSaver && redraw) {
        switch (currentScreen) {
            case BT_PICKER:    drawBtPicker();    break;
            case GENRE_SELECT: drawGenreSelect(); break;
            case FILE_LIST:    drawFileList();    break;
            case PLAYER:
                playerNeedsFullRedraw = true;
                drawPlayerScreen();
                break;
        }
    }
    if (blLevel != BL_FULL) fadeBacklight(BL_FULL, 120);
}

// Call once per loop()
static void updatePowerSaving() {
    unsigned long idle = millis() - lastActivityMs;
    switch (pwrStage) {
        case PWR_ACTIVE:
            if (idle >= DIM_AFTER_MS) enterDim();
            break;
        case PWR_DIM:
            if (idle >= SAVER_AFTER_MS) enterScreensaver();
            break;
        case PWR_SAVER:
            if (SCREEN_OFF_AFTER_MS > 0 && idle >= SCREEN_OFF_AFTER_MS) enterScreenOff();
            else animateScreensaver();
            break;
        case PWR_SCREEN_OFF:
            break;
    }
}

// ============================================================
//  NEON UI HELPERS
// ============================================================
void drawNeonPanel(int x, int y, int w, int h, int r,
                   uint16_t fill, uint16_t glow, uint16_t border)
{
    tft.fillRoundRect(x, y, w, h, r, fill);
    tft.drawRoundRect(x, y, w, h, r, glow);
    tft.drawRoundRect(x+1, y+1, w-2, h-2, r-1, border);
}

void drawPlayIcon(int cx, int cy, uint16_t color) {
    tft.fillTriangle(cx-5, cy-7, cx-5, cy+7, cx+8, cy, color);
}

void drawPauseIcon(int cx, int cy, uint16_t color) {
    tft.fillRect(cx-6, cy-7, 4, 14, color);
    tft.fillRect(cx+2, cy-7, 4, 14, color);
}

void drawLedDot(int x, int y, uint16_t color) {
    tft.fillCircle(x, y, 3, color);
    tft.drawCircle(x, y, 4, C_EDGE);
}

void transitionWipe() {
    for (int x = 0; x < TFT_W; x += 12) {
        tft.drawFastVLine(x, 0, TFT_H, C_CYAN);
        delay(2);
    }
    for (int x = 0; x < TFT_W; x += 12) {
        tft.drawFastVLine(x, 0, TFT_H, C_BG);
        delay(2);
    }
}

void drawProgressBar(int x, int y, int w, int h,
                     float filled, uint16_t fg, uint16_t bg)
{
    if (filled < 0.0f) filled = 0.0f;
    if (filled > 1.0f) filled = 1.0f;

    int innerW = w - 2;
    int innerH = h - 2;
    int fillPx = (int)(filled * innerW);

    tft.fillRect(x, y, w, h, bg);
    if (fillPx > 0) {
        tft.fillRect(x+1, y+1, fillPx, innerH, fg);
        int headX = (fillPx < innerW) ? (x + fillPx) : (x + w - 2);
        tft.drawFastVLine(headX, y+1, innerH, headBlinkOn ? C_WHITE : fg);
    }
    tft.drawRect(x, y, w, h, C_EDGE);
}

// ============================================================
//  MP3 DECODER CALLBACK  — feeds the resampler
// ============================================================
void dataCallback(MP3FrameInfo &info, int16_t *pcm_buffer, size_t len, void *) {
    sampleRate = info.samprate;
    channels   = info.nChans;
    resampler.feed(info, pcm_buffer, len);
    bytesDecoded += len * sizeof(int16_t);
}

MP3DecoderHelix mp3Decoder(dataCallback);

// ============================================================
//  BT SCAN CALLBACK
// ============================================================
bool btScanCallback(const char *ssid, esp_bd_addr_t, int rssi) {
    if (!ssid || strlen(ssid) == 0) return false;

    if (xSemaphoreTake(btMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        bool found = false;
        for (auto &d : btDevices) {
            if (strcasecmp(d.name.c_str(), ssid) == 0) {
                d.rssi = rssi; found = true; break;
            }
        }
        if (!found) {
            BtDevice dev; dev.name = String(ssid); dev.rssi = rssi;
            btDevices.push_back(dev);
            std::sort(btDevices.begin(), btDevices.end(),
                [](const BtDevice &a, const BtDevice &b){ return a.rssi > b.rssi; });
        }
        xSemaphoreGive(btMutex);
    }
    Serial.printf("[BT] Found: '%s' RSSI=%d\n", ssid, rssi);

    if (chosenDeviceName[0] != '\0' &&
        strcasecmp(ssid, chosenDeviceName) == 0) {
        Serial.println("[BT] Chosen device found - connecting");
        return true;
    }
    return false;
}

// ============================================================
//  BT AUDIO CALLBACK  — pulls from resampler, applies fade
// ============================================================
int32_t get_audio_data(uint8_t *data, int32_t len) {
    int16_t *out    = (int16_t *)data;
    int      frames = len / 4;

    resampler.pull(out, frames);

    // Fade ramp
    fadeTarget = (isPlaying && !isPaused) ? FADE_FULL : 0;
    for (int i = 0; i < frames * 2; i++) {
        if      (fadeLevel < fadeTarget) fadeLevel += FADE_STEP;
        else if (fadeLevel > fadeTarget) fadeLevel -= FADE_STEP;
        out[i] = (int16_t)(((int32_t)out[i] * fadeLevel) >> 8);
    }
    return len;
}

// ============================================================
//  FEED DECODER TASK
// ============================================================
void feedDecoderTask(void *) {
    uint8_t       mp3Buf[512];
    unsigned long loopCount = 0;
    int           zeroReads = 0;

    while (feedTaskRunning) {
        if (!musicFile || !isPlaying) {
            vTaskDelay(pdMS_TO_TICKS(10)); continue;
        }
        // Gate on resampler buffer occupancy
        int avail = resampler.available();
        if (avail > 3500) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }

        int n = musicFile.read(mp3Buf, sizeof(mp3Buf));
        if (n > 0) {
            zeroReads     = 0;
            mp3BytesRead += n;
            if (xSemaphoreTake(decoderMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                mp3Decoder.write(mp3Buf, n);
                xSemaphoreGive(decoderMutex);
            }
            if (++loopCount % 200 == 0)
                Serial.printf("[feed] %lu KB  avail=%d\n", mp3BytesRead/1024, avail);
        } else if (n == 0) {
            size_t pos = musicFile.position();
            size_t sz  = musicFile.size();
            zeroReads++;
            if (sz > 0 && pos >= sz && zeroReads >= 2) {
                Serial.println("[feed] TRUE EOF");
                musicFile.close();
                isPlaying    = false;
                songFinished = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        } else {
            Serial.println("[feed] SD error");
            isPlaying    = false;
            songFinished = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    feedTaskRunning = false;
    vTaskDelete(NULL);
}

void startFeedTask() {
    if (feedTaskRunning) return;
    feedTaskRunning = true;
    songFinished    = false;

    BaseType_t ok = xTaskCreatePinnedToCore(
        feedDecoderTask, "FeedDec", 3072, NULL, 1, &feedTaskHandle, 0);
    if (ok != pdPASS)
        ok = xTaskCreatePinnedToCore(
            feedDecoderTask, "FeedDec", 2048, NULL, 1, &feedTaskHandle, 0);

    if (ok != pdPASS) {
        feedTaskRunning = false; feedTaskHandle = NULL;
        Serial.printf("[ERROR] Task failed! heap=%u\n", ESP.getFreeHeap());
    }
}

void stopFeedTask() {
    if (!feedTaskRunning && feedTaskHandle == NULL) return;
    feedTaskRunning = false;
    unsigned long deadline = millis() + 600;
    while (feedTaskHandle != NULL && millis() < deadline) {
        if (eTaskGetState(feedTaskHandle) == eDeleted) { feedTaskHandle = NULL; break; }
        delay(10);
    }
    feedTaskHandle = NULL;
    delay(20);
}

void stopPlayback() {
    isPlaying  = false;
    isPaused   = false;
    fadeTarget = 0;
    delay(15);
    stopFeedTask();
    if (musicFile) musicFile.close();
    resampler.reset();
    fadeLevel       = 0;
    songFinished    = false;
    currentFilePath = "";
    mp3BytesRead    = 0;
    fileSize        = 0;
    currentTrackIdx = -1;
}

// ============================================================
//  REWIND ~10 SECONDS
// ============================================================
void rewindTenSeconds() {
    if (!musicFile) return;

    unsigned long curPos = musicFile.position();
    unsigned long target = (curPos > 160000UL) ? (curPos - 160000UL) : 0;

    Serial.printf("[seek] %lu -> %lu\n", curPos, target);

    resampler.reset();

    if (xSemaphoreTake(decoderMutex, pdMS_TO_TICKS(300)) == pdTRUE) {
        mp3Decoder.end();
        mp3Decoder.begin();
        xSemaphoreGive(decoderMutex);
    }

    musicFile.seek(target);
    mp3BytesRead = target;

    SPI_TAKE();
    tft.fillRect(56, 50, 48, 28, C_CYAN);
    tft.setTextColor(C_BLACK); tft.setTextSize(1);
    tft.setCursor(64, 60); tft.print(" REW ");
    SPI_GIVE();
    delay(150);
    drawPlayerData();
}

// ============================================================
//  SD / FILE OPERATIONS
// ============================================================
void listCurrentDir() {
    currentEntries.clear();
    File dir = SD.open(currentDir);
    if (!dir) { Serial.println("Cannot open: " + currentDir); return; }
    while (true) {
        File e = dir.openNextFile();
        if (!e) break;
        String nm = e.name();
        if (nm.startsWith(".")) { e.close(); continue; }
        DirEntry de; de.name = nm; de.isDir = e.isDirectory();
        if (!de.isDir) {
            String lc = nm; lc.toLowerCase();
            if (!lc.endsWith(".mp3")) { e.close(); continue; }
        }
        currentEntries.push_back(de);
        e.close();
    }
    dir.close();
    std::sort(currentEntries.begin(), currentEntries.end(),
        [](const DirEntry &a, const DirEntry &b){
            if (a.isDir != b.isDir) return (int)a.isDir > (int)b.isDir;
            return strcasecmp(a.name.c_str(), b.name.c_str()) < 0;
        });
    listScroll = 0;
    listCursor = 0;
}

void playFile(const String &fullPath, int trackIdx) {
    stopFeedTask();
    if (musicFile) musicFile.close();
    musicFile = SD.open(fullPath);
    if (!musicFile) { Serial.println("Cannot open: " + fullPath); return; }

    currentFilePath  = fullPath;
    currentTrackIdx  = trackIdx;
    fileSize         = musicFile.size();
    bytesDecoded     = 0;
    mp3BytesRead     = 0;
    isPaused         = false;
    songFinished     = false;
    titleScrollPx    = 0;
    lastScrollMs     = millis() + TITLE_PAUSE_MS;

    resampler.reset();
    fadeLevel  = 0;
    fadeTarget = 0;

    playStartMs = millis();
    isPlaying   = true;

    joyLockedUntil   = millis() + JOY_LOCKOUT_MS;
    waitingForCenter = true;

    if (xSemaphoreTake(decoderMutex, pdMS_TO_TICKS(300)) == pdTRUE) {
        mp3Decoder.end();
        if (!mp3Decoder.begin()) Serial.println("[WARN] mp3Decoder.begin() false");
        xSemaphoreGive(decoderMutex);
    }
    Serial.printf("[play] %s  %lu bytes\n", fullPath.c_str(), (unsigned long)fileSize);
    startFeedTask();
}

int findNextTrack(int from) {
    int n = (int)currentEntries.size();
    for (int i = from+1; i < n;    i++) if (!currentEntries[i].isDir) return i;
    for (int i = 0;      i <= from; i++) if (!currentEntries[i].isDir) return i;
    return -1;
}
int findPrevTrack(int from) {
    int n = (int)currentEntries.size();
    for (int i = from-1; i >= 0; i--) if (!currentEntries[i].isDir) return i;
    for (int i = n-1; i > from;  i--) if (!currentEntries[i].isDir) return i;
    return -1;
}

void nextSong() {
    int idx = findNextTrack(currentTrackIdx);
    if (idx < 0) return;
    playFile(currentDir + currentEntries[idx].name, idx);
    playerNeedsFullRedraw = true;
    drawPlayerScreen();
}
void prevSong() {
    int idx = findPrevTrack(currentTrackIdx);
    if (idx < 0) return;
    playFile(currentDir + currentEntries[idx].name, idx);
    playerNeedsFullRedraw = true;
    drawPlayerScreen();
}

// ============================================================
//  DRAW: BT PICKER
// ============================================================
#define BT_ROW_H    22
#define BT_ROWS_VIS  4
#define BT_LIST_Y   18

void drawBtPicker() {
    SPI_TAKE();
    tft.fillScreen(C_BG);
    tft.fillRect(0, 0, TFT_W, 16, C_SURFACE);
    tft.drawFastHLine(0, 15, TFT_W, C_CYAN);
    tft.setTextColor(C_CYAN); tft.setTextSize(1);
    tft.setCursor(4, 4); tft.print(":: BLUETOOTH");

    std::vector<BtDevice> snap;
    if (xSemaphoreTake(btMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
        snap = btDevices; xSemaphoreGive(btMutex);
    }
    lastBtCount = (int)snap.size();

    if (snap.empty()) {
        tft.setTextColor(C_MAGENTA); tft.setTextSize(1);
        tft.setCursor(4, 40); tft.print("SCANNING...");
        tft.setTextColor(C_LTGREY);
        tft.setCursor(4, 58); tft.print("Wake your speaker");
        tft.setCursor(4, 72); tft.print("to make it visible.");
        tft.setTextColor(C_YELLOW);
        tft.setCursor(4, 100); tft.print("HOLD LEFT 1.5s to skip");
    } else {
        btPickerScroll = constrain(btPickerScroll, 0,
                                   max(0, (int)snap.size() - BT_ROWS_VIS));
        int endRow = min((int)snap.size(), btPickerScroll + BT_ROWS_VIS);

        for (int i = btPickerScroll; i < endRow; i++) {
            int      ry  = BT_LIST_Y + (i - btPickerScroll) * BT_ROW_H;
            bool     sel = (i == btPickerSel);
            uint16_t rowBg = sel ? C_SURFACE2 : C_SURFACE;
            tft.fillRect(0, ry, TFT_W, BT_ROW_H-2, rowBg);
            tft.fillRect(0, ry, 3, BT_ROW_H-2, sel ? C_MAGENTA : C_CYAN_DIM);
            if (sel) tft.drawFastHLine(0, ry+BT_ROW_H-2, TFT_W, C_MAGENTA);

            tft.setTextColor(sel ? C_WHITE : C_LTGREY); tft.setTextSize(1);
            String nm = snap[i].name;
            if ((int)nm.length() > 18) nm = nm.substring(0,15) + "...";
            tft.setCursor(8, ry+7); tft.print(nm);

            int rssi = snap[i].rssi;
            int bars = rssi > -60 ? 3 : rssi > -75 ? 2 : 1;
            for (int b = 0; b < 3; b++) {
                int bh = 4+b*3, bx = TFT_W-22+b*5, by = ry+6+(9-bh);
                if (b < bars) {
                    uint16_t bc = (bars==3)?C_LIME:(bars==2)?C_YELLOW:C_ORANGE;
                    tft.fillRect(bx, by, 3, bh, bc);
                } else {
                    tft.drawRect(bx, by, 3, bh, C_DKGREY);
                }
            }
        }
        tft.setTextColor(C_GREY); tft.setTextSize(1);
        tft.setCursor(4, BT_LIST_Y + BT_ROWS_VIS*BT_ROW_H + 4);
        tft.print("R/BTN=connect  holdL=skip");

        if (btPickerScroll > 0) {
            tft.setTextColor(C_CYAN);
            tft.setCursor(TFT_W-10, BT_LIST_Y); tft.print("^");
        }
        if (endRow < (int)snap.size()) {
            tft.setTextColor(C_CYAN);
            tft.setCursor(TFT_W-10, BT_LIST_Y+BT_ROWS_VIS*BT_ROW_H-10);
            tft.print("v");
        }
    }

    if (btConnected) {
        drawNeonPanel(4, 108, 152, 18, 4, C_SURFACE, C_LIME, C_LIME_DIM);
        tft.setTextColor(C_LIME); tft.setTextSize(1);
        tft.setCursor(12, 114); tft.print("CONNECTED  R/BTN continue");
    }
    SPI_GIVE();
}

// ============================================================
//  DRAW: GENRE SELECT
// ============================================================
void drawGenreSelect() {
    SPI_TAKE();
    tft.fillScreen(C_BG);
    tft.fillRect(0, 0, TFT_W, 16, C_SURFACE);
    tft.drawFastHLine(0, 15, TFT_W, C_CYAN);
    tft.setTextColor(C_CYAN); tft.setTextSize(1);
    tft.setCursor(4, 4); tft.print(":: SELECT GENRE");

    const int CARD_H   = 28;
    const int CARD_GAP = 4;
    const int START_Y  = 20;

    struct Theme { uint16_t bright; uint16_t dim; };
    static const Theme THEMES[3] = {
        { C_CYAN,    C_CYAN_DIM },
        { C_MAGENTA, C_MAG_DIM  },
        { C_LIME,    C_LIME_DIM }
    };

    for (int i = 0; i < NUM_GENRES; i++) {
        int      gy  = START_Y + i*(CARD_H+CARD_GAP);
        uint16_t bri = THEMES[i].bright;
        uint16_t dim = THEMES[i].dim;
        bool     sel = (i == genreIndex);

        tft.fillRoundRect(8, gy, TFT_W-16, CARD_H, 5, C_SURFACE);
        tft.fillRect(8, gy, 5, CARD_H, bri);
        tft.drawRoundRect(8, gy, TFT_W-16, CARD_H, 5, sel ? bri : dim);
        if (sel) tft.drawRoundRect(7, gy-1, TFT_W-14, CARD_H+2, 6, dim);

        tft.setTextColor(sel ? C_WHITE : C_LTGREY); tft.setTextSize(1);
        String lbl = GENRE_FOLDERS[i];
        if ((int)lbl.length() > 18) lbl = lbl.substring(0,15) + "...";
        tft.setCursor(22, gy+10); tft.print(lbl);

        if (sel) {
            tft.setTextColor(bri);
            tft.setCursor(TFT_W-18, gy+10); tft.print(">");
        }
    }

    tft.setTextColor(C_GREY); tft.setTextSize(1);
    tft.setCursor(4, 116); tft.print("UD=sel  R/BTN=open  L=BT");
    SPI_GIVE();
}

// ============================================================
//  DRAW: FILE LIST
// ============================================================
#define LIST_ROW_H    20
#define LIST_ROWS_VIS  5
#define LIST_Y        18

void drawFileList() {
    SPI_TAKE();
    tft.fillScreen(C_BG);

    String hdr = currentDir;
    if (hdr.startsWith(MUSIC_ROOT)) hdr = hdr.substring(MUSIC_ROOT.length());
    if (hdr.startsWith("/"))        hdr = hdr.substring(1);
    if (hdr.endsWith("/"))          hdr = hdr.substring(0, hdr.length()-1);
    if (hdr.isEmpty())              hdr = "Tracks";
    if ((int)hdr.length() > 12)    hdr = hdr.substring(0,9) + "...";

    tft.fillRect(0, 0, TFT_W, 16, C_SURFACE);
    tft.drawFastHLine(0, 15, TFT_W, C_MAGENTA);
    tft.setTextColor(C_MAGENTA); tft.setTextSize(1);
    tft.setCursor(4, 4); tft.print(":: " + hdr);

    if (!currentEntries.empty()) {
        char counter[12];
        sprintf(counter, "%d/%d", listCursor + 1, (int)currentEntries.size());
        tft.setTextColor(C_CYAN);
        tft.setCursor(TFT_W - 40, 4);
        tft.print(counter);
    }

    if (currentEntries.empty()) {
        tft.setTextColor(C_ORANGE); tft.setCursor(8, 50);
        tft.print("NO MP3 FILES");
        SPI_GIVE(); return;
    }

    int maxScroll = max(0, (int)currentEntries.size()-LIST_ROWS_VIS);
    listScroll    = constrain(listScroll, 0, maxScroll);
    int endRow    = min((int)currentEntries.size(), listScroll+LIST_ROWS_VIS);

    for (int i = listScroll; i < endRow; i++) {
        int      ry     = LIST_Y + (i-listScroll)*LIST_ROW_H;
        bool     isCur  = (i == listCursor);
        bool     isPlPg = (i == currentTrackIdx);

        uint16_t rowBg = isCur ? C_SURFACE2 : (i%2 ? C_BG : C_SURFACE);
        tft.fillRect(0, ry, TFT_W, LIST_ROW_H-1, rowBg);

        uint16_t strip = C_BLACK;
        if      (isCur && isPlPg) strip = C_CYAN;
        else if (isCur)           strip = C_WHITE;
        else if (isPlPg)          strip = C_CYAN;
        tft.fillRect(0, ry, 3, LIST_ROW_H-1, strip);

        if (isPlPg)
            tft.fillTriangle(6, ry+6, 6, ry+14, 12, ry+10, C_CYAN);

        uint16_t textCol = C_LTGREY;
        if (isCur)  textCol = C_WHITE;
        if (isPlPg) textCol = C_CYAN;
        tft.setTextColor(textCol); tft.setTextSize(1);

        String nm = currentEntries[i].name;
        if (nm.endsWith(".mp3") || nm.endsWith(".MP3"))
            nm = nm.substring(0, nm.length()-4);
        if (currentEntries[i].isDir) nm = "[D] " + nm;
        if ((int)nm.length() > 22)   nm = nm.substring(0,19) + "...";

        tft.setCursor(isPlPg ? 16 : 8, ry+6); tft.print(nm);
    }

    if (listScroll > 0) {
        tft.setTextColor(C_CYAN); tft.setCursor(TFT_W-10, LIST_Y); tft.print("^");
    }
    if (endRow < (int)currentEntries.size()) {
        tft.setTextColor(C_CYAN);
        tft.setCursor(TFT_W-10, LIST_Y+LIST_ROWS_VIS*LIST_ROW_H-8); tft.print("v");
    }

    if (isPlaying) {
        tft.fillRect(0, TFT_H - 12, TFT_W, 12, C_SURFACE);
        tft.drawFastHLine(0, TFT_H - 12, TFT_W, C_CYAN);
        tft.setTextColor(C_CYAN); tft.setTextSize(1);
        tft.setCursor(4, TFT_H - 9);

        String now = currentFilePath.substring(currentFilePath.lastIndexOf('/') + 1);
        if (now.endsWith(".mp3")) now = now.substring(0, now.length() - 4);
        if ((int)now.length() > 24) now = now.substring(0, 21) + "...";

        tft.print("> " + now);
    }

    SPI_GIVE();
}

// ============================================================
//  DRAW: PLAYER FULL
// ============================================================
void drawPlayerFull() {
    SPI_TAKE();
    tft.fillScreen(C_BG);

    tft.fillRect(0, 0, TFT_W, 15, C_SURFACE);
    tft.drawFastHLine(0, 15, TFT_W, C_CYAN);
    drawLedDot(6, 7, C_CYAN);

    drawNeonPanel(4, 50, 46, 28, 5, C_SURFACE, C_CYAN_DIM, C_CYAN);
    tft.setTextColor(C_CYAN); tft.setTextSize(2);
    tft.setCursor(16, 56); tft.print("<<");

    drawNeonPanel(110, 50, 46, 28, 5, C_SURFACE, C_CYAN_DIM, C_CYAN);
    tft.setTextColor(C_CYAN); tft.setCursor(122, 56); tft.print(">>");

    drawNeonPanel(4, 86, 24, 20, 4, C_SURFACE, C_MAG_DIM, C_MAGENTA);
    tft.setTextColor(C_MAGENTA); tft.setTextSize(2); tft.setCursor(12, 90); tft.print("-");

    drawNeonPanel(132, 86, 24, 20, 4, C_SURFACE, C_MAG_DIM, C_MAGENTA);
    tft.setTextColor(C_MAGENTA); tft.setCursor(140, 90); tft.print("+");

    SPI_GIVE();
    playerNeedsFullRedraw = false;
}

// ============================================================
//  DRAW: PLAYER DATA
// ============================================================
void drawPlayerData() {
    String title = currentFilePath.substring(currentFilePath.lastIndexOf('/')+1);
    if (title.endsWith(".mp3") || title.endsWith(".MP3"))
        title = title.substring(0, title.length()-4);

    SPI_TAKE();

    tft.fillRect(14, 0, TFT_W-14, 15, C_SURFACE);
    tft.setTextColor(C_CYAN); tft.setTextSize(1);
    int titlePx   = (int)title.length()*6;
    int maxScroll = max(0, titlePx-(TFT_W-20));
    if (maxScroll == 0) {
        tft.setCursor((TFT_W-titlePx)/2+6, 4);
        tft.print(title);
        titleScrollPx = 0;
    } else {
        tft.setCursor(14-titleScrollPx, 4);
        tft.print(title);
        unsigned long now = millis();
        if (now >= lastScrollMs && now-lastScrollMs >= (unsigned long)TITLE_SCROLL_MS) {
            lastScrollMs = now;
            titleScrollPx++;
            if (titleScrollPx > maxScroll+20) {
                titleScrollPx = 0;
                lastScrollMs  = now+TITLE_PAUSE_MS;
            }
        }
    }
    tft.drawFastHLine(0, 15, TFT_W, C_CYAN);

    {
        float pct = 0.0f;
        unsigned long fsz = fileSize, rd = mp3BytesRead;
        if (fsz > 0 && rd <= fsz)
            pct = (float)((uint64_t)rd*10000ULL/fsz)/10000.0f;
        else if (fsz > 0) pct = 1.0f;
        drawProgressBar(4, 20, 152, 9, pct, C_CYAN, C_SURFACE);
    }

    {
        tft.fillRect(0, 32, TFT_W, 12, C_BG);
        unsigned long el = 0;
        if (sampleRate > 0 && channels > 0)
            el = (bytesDecoded/sizeof(int16_t))/((unsigned long)sampleRate*channels);
        unsigned long tot = (fileSize > 0) ? (fileSize*8UL/128000UL) : 0;
        char buf[10];
        tft.setTextSize(1);
        tft.setTextColor(C_CYAN);
        sprintf(buf, "%02lu:%02lu", el/60, el%60);
        tft.setCursor(4, 34); tft.print(buf);
        tft.setTextColor(C_GREY); tft.setCursor(72, 34); tft.print(":");
        tft.setTextColor(C_MAGENTA);
        sprintf(buf, "%02lu:%02lu", tot/60, tot%60);
        tft.setCursor(TFT_W-34, 34); tft.print(buf);
    }

    {
        uint16_t btnBg = isPaused ? C_LIME : C_CYAN;
        drawNeonPanel(56, 50, 48, 28, 5, btnBg, btnBg, C_WHITE);
        int cx = 80, cy = 64;
        if (isPaused) drawPlayIcon(cx, cy, C_BLACK);
        else          drawPauseIcon(cx, cy, C_BLACK);
    }

    {
        int vol = a2dp_source.get_volume();
        if (vol < 0)       vol = 0;
        if (vol > VOL_MAX) vol = VOL_MAX;
        float volFrac = (float)vol/(float)VOL_MAX;
        int   volPct  = (int)(volFrac*100.0f);
        tft.fillRect(32, 86, 96, 20, C_BG);
        uint16_t volCol = (volPct>66)?C_LIME:(volPct>33)?C_CYAN:C_MAGENTA;
        drawProgressBar(32, 89, 96, 12, volFrac, volCol, C_SURFACE);
        char vb[8]; sprintf(vb, "%d%%", volPct);
        tft.setTextColor(C_WHITE); tft.setTextSize(1);
        tft.setCursor(70, 92); tft.print(vb);
    }

    {
        tft.fillRect(0, 108, TFT_W, 20, C_BG);
        drawLedDot(8, 118, btConnected ? C_LIME : C_MAGENTA);
        tft.setTextColor(btConnected ? C_LIME : C_MAGENTA);
        tft.setTextSize(1);
        tft.setCursor(16, 114);
        tft.print(btConnected ? "LINK OK" : "NO LINK");
    }

    SPI_GIVE();
}

void drawPlayerScreen() {
    if (isSaverActive()) return;   // screensaver owns the display; wakeScreen() repaints
    if (playerNeedsFullRedraw) drawPlayerFull();
    drawPlayerData();
}

// ============================================================
//  JOYSTICK READING
// ============================================================
JoyDir readJoystick() {
    if (digitalRead(JOY_SW) == LOW) {
        unsigned long t = millis();
        while (digitalRead(JOY_SW) == LOW && millis()-t < 2000);
        unsigned long held = millis() - t;
        if (held >= JOY_HOLD_MS) return JOY_HOLD;
        return JOY_PRESS;
    }
    int x  = analogRead(JOY_X);
    int y  = analogRead(JOY_Y);
    int dx = x - JOY_CENTER_X;
    int dy = y - JOY_CENTER_Y;
    if (abs(dy) > abs(dx)) {
        if (dy < -JOY_DEADZONE) return JOY_UP;
        if (dy >  JOY_DEADZONE) return JOY_DOWN;
    } else {
        if (dx < -JOY_DEADZONE) return JOY_LEFT;
        if (dx >  JOY_DEADZONE) return JOY_RIGHT;
    }
    return JOY_NONE;
}

JoyDir readJoystickStable() {
    static JoyDir pending      = JOY_NONE;
    static int    pendingCount = 0;
    JoyDir raw = readJoystick();
    if (raw == JOY_PRESS || raw == JOY_HOLD) {
        pending = JOY_NONE; pendingCount = 0; return raw;
    }
    if (raw == pending) { if (pendingCount < JOY_STABLE_READS) pendingCount++; }
    else                { pending = raw; pendingCount = 1; }
    if (pendingCount >= JOY_STABLE_READS) return pending;
    return JOY_NONE;
}

// ============================================================
//  JOYSTICK HANDLER
// ============================================================
static unsigned long leftHeldSince = 0;
#define SKIP_BT_HOLD_MS 1500

static void openCursorEntry() {
    if (currentEntries.empty()) return;
    int row = listCursor;
    if (row < 0 || row >= (int)currentEntries.size()) return;
    if (currentEntries[row].isDir) {
        currentDir += currentEntries[row].name + "/";
        listCurrentDir();
        transitionWipe();
        drawFileList();
    } else {
        playFile(currentDir + currentEntries[row].name, row);
        playerNeedsFullRedraw = true;
        currentScreen = PLAYER;
        drawPlayerScreen();
    }
}

void handleJoystick() {
    unsigned long now = millis();
    if (now < joyLockedUntil) return;

    JoyDir dir = readJoystickStable();

    // ---- activity tracking for dimming / screensaver ----
    // Uses the filtered direction, so analog noise can't keep the screen awake.
    if (dir != JOY_NONE) {
        lastActivityMs = now;
        if (pwrStage != PWR_ACTIVE) {
            bool swallow = isSaverActive();
            wakeScreen(true);
            if (swallow) {
                // This input only wakes the display; don't also trigger an action.
                lastJoyDir       = JOY_NONE;
                waitingForCenter = (dir != JOY_PRESS && dir != JOY_HOLD);
                joyLockedUntil   = millis() + 250;
                return;
            }
        }
    }

    if (waitingForCenter) {
        if (dir == JOY_NONE) { waitingForCenter = false; lastJoyDir = JOY_NONE; }
        return;
    }

    if (currentScreen == BT_PICKER) {
        if (dir == JOY_LEFT) {
            if (leftHeldSince == 0) leftHeldSince = now;
            if (now-leftHeldSince >= SKIP_BT_HOLD_MS) {
                leftHeldSince = 0;
                transitionWipe();
                currentScreen = GENRE_SELECT; drawGenreSelect();
                lastJoyDir = JOY_NONE; return;
            }
        } else { leftHeldSince = 0; }
    }

    if (dir == JOY_NONE) { lastJoyDir = JOY_NONE; return; }
    if (dir == lastJoyDir && now-lastJoyActionMs < JOY_REPEAT_MS) return;
    lastJoyDir      = dir;
    lastJoyActionMs = now;
    waitingForCenter = (dir != JOY_PRESS && dir != JOY_HOLD);

    if (currentScreen == BT_PICKER) {
        std::vector<BtDevice> snap;
        if (xSemaphoreTake(btMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            snap = btDevices; xSemaphoreGive(btMutex);
        }
        if (dir == JOY_UP && btPickerSel > 0) {
            btPickerSel--;
            if (btPickerSel < btPickerScroll) btPickerScroll = btPickerSel;
            drawBtPicker();
        } else if (dir == JOY_DOWN && btPickerSel < (int)snap.size()-1) {
            btPickerSel++;
            if (btPickerSel >= btPickerScroll+BT_ROWS_VIS)
                btPickerScroll = btPickerSel-BT_ROWS_VIS+1;
            drawBtPicker();
        } else if (dir == JOY_RIGHT || dir == JOY_PRESS) {
            if (btConnected) {
                transitionWipe();
                currentScreen = GENRE_SELECT; drawGenreSelect();
            } else if (btPickerSel >= 0 && btPickerSel < (int)snap.size()) {
                strncpy(chosenDeviceName, snap[btPickerSel].name.c_str(),
                        sizeof(chosenDeviceName)-1);
                chosenDeviceName[sizeof(chosenDeviceName)-1] = '\0';
                SPI_TAKE();
                tft.fillScreen(C_BG);
                tft.setTextColor(C_CYAN); tft.setTextSize(1);
                tft.setCursor(4, 36); tft.print(":: CONNECTING");
                tft.drawFastHLine(4, 48, TFT_W-8, C_CYAN);
                tft.setTextColor(C_MAGENTA); tft.setCursor(4, 56);
                String s = String(chosenDeviceName);
                if ((int)s.length()>22) s = s.substring(0,19)+"...";
                tft.print(s);
                tft.setTextColor(C_GREY);
                tft.setCursor(4, 80); tft.print("Move device closer");
                tft.setCursor(4, 94); tft.print("if it takes a while.");
                SPI_GIVE();
            } else if (!snap.empty() && btPickerSel < 0) {
                btPickerSel = 0; drawBtPicker();
            }
        }
        return;
    }

    if (currentScreen == GENRE_SELECT) {
        if (dir == JOY_UP)
            { genreIndex = (genreIndex-1+NUM_GENRES)%NUM_GENRES; drawGenreSelect(); }
        else if (dir == JOY_DOWN)
            { genreIndex = (genreIndex+1)%NUM_GENRES; drawGenreSelect(); }
        else if (dir == JOY_RIGHT || dir == JOY_PRESS) {
            currentDir    = MUSIC_ROOT+"/"+GENRE_FOLDERS[genreIndex]+"/";
            listCurrentDir();
            transitionWipe();
            currentScreen = FILE_LIST; drawFileList();
        } else if (dir == JOY_LEFT) {
            transitionWipe();
            currentScreen = BT_PICKER; drawBtPicker();
        }
        return;
    }

    if (currentScreen == FILE_LIST) {
        int entries = (int)currentEntries.size();
        if (dir == JOY_UP && listCursor > 0) {
            listCursor--;
            if (listCursor < listScroll) listScroll = listCursor;
            drawFileList();
        } else if (dir == JOY_DOWN && listCursor < entries-1) {
            listCursor++;
            if (listCursor >= listScroll+LIST_ROWS_VIS)
                listScroll = listCursor-LIST_ROWS_VIS+1;
            drawFileList();
        } else if (dir == JOY_RIGHT || dir == JOY_PRESS) {
            openCursorEntry();
        } else if (dir == JOY_LEFT) {
            stopPlayback();
            transitionWipe();
            currentScreen = GENRE_SELECT; drawGenreSelect();
        }
        return;
    }

    if (currentScreen == PLAYER) {
        if (dir == JOY_UP) {
            int v = min(a2dp_source.get_volume()+VOL_STEP, VOL_MAX);
            a2dp_source.set_volume(v);
            drawPlayerData();
        } else if (dir == JOY_DOWN) {
            int v = max(a2dp_source.get_volume()-VOL_STEP, 0);
            a2dp_source.set_volume(v);
            drawPlayerData();
        } else if (dir == JOY_RIGHT) {
            nextSong();
        } else if (dir == JOY_LEFT) {
            transitionWipe();
            currentScreen = FILE_LIST; drawFileList();
        } else if (dir == JOY_PRESS) {
            isPaused = !isPaused;
            drawPlayerData();
        } else if (dir == JOY_HOLD) {
            rewindTenSeconds();
        }
        return;
    }
}

// ============================================================
//  BOOT SPLASH + CHECKLIST
// ============================================================
void drawBootSplash() {
    SPI_TAKE();
    tft.fillScreen(C_BG);

    const char *line1 = "ESP32";
    const char *line2 = "PLAYER";
    int t1x = (TFT_W - strlen(line1)*12) / 2;
    int t2x = (TFT_W - strlen(line2)*12) / 2;

    for (int y = 0; y < TFT_H; y += 3) {
        tft.drawFastHLine(0, y, TFT_W, C_CYAN);
        if (y >= 12) tft.drawFastHLine(0, y - 12, TFT_W, C_BG);

        if (y == 42) {
            tft.setTextColor(C_CYAN); tft.setTextSize(2);
            tft.setCursor(t1x, 40); tft.print(line1);
        }
        if (y == 62) {
            tft.setTextColor(C_MAGENTA);
            tft.setCursor(t2x, 60); tft.print(line2);
        }
        if (y == 84) {
            tft.drawFastHLine(t1x, 82, 132, C_LIME);
        }
        delay(3);
    }
    tft.fillRect(0, 0, TFT_W, 20, C_BG);
    tft.setTextColor(C_GREY); tft.setTextSize(1);
    tft.setCursor(50, 92); tft.print("booting...");
    SPI_GIVE();
    delay(300);
}

void drawBootChecklist(bool sdOk, bool decOk, bool btOk) {
    SPI_TAKE();
    tft.fillScreen(C_BG);
    tft.setTextColor(C_CYAN); tft.setTextSize(1);
    tft.setCursor(4, 4);  tft.print(":: BOOT SEQUENCE");
    tft.drawFastHLine(0, 15, TFT_W, C_CYAN);

    auto row = [](int i, const char *label, bool ok) {
        int y = 24 + i * 14;
        tft.setTextColor(C_LTGREY); tft.setCursor(4, y); tft.print(label);
        tft.setTextColor(ok ? C_LIME : C_RED);
        tft.setCursor(TFT_W - 30, y); tft.print(ok ? "OK" : "FAIL");
    };

    row(0, "SD card",   sdOk);  delay(120);
    row(1, "Decoder",   decOk); delay(120);
    row(2, "Bluetooth", btOk);  delay(120);

    tft.setTextColor(C_CYAN);
    tft.setCursor(4, 24 + 3*14);
    tft.print("> Ready.");
    SPI_GIVE();
    delay(350);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== ESP32 TFT Music Player v11 ===");
    Serial.printf("Sink rate: %.0f Hz\n", SINK_RATE_HZ);

    // Backlight on a PWM channel so it can be dimmed (Arduino-ESP32 3.x API)
    if (!ledcAttach(TFT_LED, BL_PWM_FREQ, BL_PWM_BITS))
        Serial.println("[WARN] backlight PWM attach failed");
    setBacklight(BL_FULL);

    decoderMutex = xSemaphoreCreateMutex();
    spiMutex     = xSemaphoreCreateMutex();
    btMutex      = xSemaphoreCreateMutex();
    if (!decoderMutex || !spiMutex || !btMutex) {
        Serial.println("[FATAL] Mutex creation failed");
        while (1) delay(1000);
    }

    pinMode(TFT_CS, OUTPUT); digitalWrite(TFT_CS, HIGH);
    pinMode(SD_CS,  OUTPUT); digitalWrite(SD_CS,  HIGH);
    pinMode(JOY_SW, INPUT_PULLUP);
    analogReadResolution(12);

    tft.initR(INITR_BLACKTAB);
    tft.setRotation(1);

    drawBootSplash();

    bool sdOk = false, decOk = false, btOk = false;

    sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (SD.begin(SD_CS, sdSPI, 4000000)) sdOk = true;
    Serial.println(sdOk ? "SD OK" : "[FATAL] SD failed");

    if (!SD.exists(MUSIC_ROOT.c_str())) {
        Serial.println("[FATAL] Missing: " + MUSIC_ROOT);
        while (1) delay(1000);
    }

    if (mp3Decoder.begin()) decOk = true;
    Serial.println(decOk ? "Decoder OK" : "[WARN] decoder init false");

    // Configure the resampler for the JBL's 48 kHz sink rate.
    resampler.begin(SINK_RATE_HZ);

    a2dp_source.set_ssid_callback(btScanCallback);
    a2dp_source.set_data_callback(get_audio_data);
    a2dp_source.set_auto_reconnect(false);
    a2dp_source.start();
    a2dp_source.set_volume(64);
    btOk = true;

    drawBootChecklist(sdOk, decOk, btOk);

    Serial.println("BT scanning...");
    Serial.printf("[mem] boot done  free=%u  max=%u\n",
                  ESP.getFreeHeap(), ESP.getMaxAllocHeap());

    currentScreen = BT_PICKER;
    drawBtPicker();
    lastActivityMs = millis();   // idle timer starts now, not at power-on
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
    handleJoystick();

    {
        static unsigned long lastBtPoll = 0;
        if (millis()-lastBtPoll >= 500) {
            lastBtPoll = millis();
            bool nowConn = a2dp_source.is_connected();
            if (nowConn && !btConnected) {
                btConnected = true;
                Serial.println("[BT] Connected");
                wakeScreen(false);   // screen comes up; the menu is drawn right below
                transitionWipe();
                currentScreen = GENRE_SELECT; drawGenreSelect();
            } else if (!nowConn && btConnected) {
                btConnected = false;
                Serial.println("[BT] Disconnected");
                wakeScreen(true);    // let the user see the link dropped
            }
            if (currentScreen == BT_PICKER) {
                int cnt = 0;
                if (xSemaphoreTake(btMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    cnt = (int)btDevices.size(); xSemaphoreGive(btMutex);
                }
                if (cnt != lastBtCount && !isSaverActive()) drawBtPicker();
            }
        }
    }

    if (currentScreen == PLAYER
        && songFinished && !feedTaskRunning
        && !currentEntries.empty() && currentTrackIdx >= 0
        && millis()-playStartMs > 10000UL)
    {
        Serial.println("[loop] auto-advance");
        songFinished = false;
        nextSong();
    }

    if (currentScreen == PLAYER && isPlaying && !isPaused && !isSaverActive()) {
        static unsigned long lastUpdate = 0;
        if (millis()-lastUpdate >= 250) {
            lastUpdate = millis();
            headBlinkOn = !headBlinkOn;
            drawPlayerData();
        }
    }

    updatePowerSaving();

    delay(10);
}