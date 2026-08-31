// ---- Includes & MAVLink Buffer Setup ----
#define SERIAL_TX_BUFFER_SIZE 128
#define SERIAL_RX_BUFFER_SIZE 128
#include <AltSoftSerial.h>
#define MAVLINK_NO_SIGN_PACKET
#define MAVLINK_NO_SIGNATURE_CHECK
#define MAVLINK_COMM_NUM_BUFFERS 1
#define MAVLINK_MAX_PAYLOAD_LEN 54
#define MAVLINK_GET_CHANNEL_BUFFER 1
#define MAVLINK_GET_CHANNEL_STATUS 1
#define MAVLINK_MESSAGE_CRCS {\
  {0, 50, 9, 9, 0, 0, 0}, \
  {11, 89, 6, 6, 1, 4, 0}, \
  {33, 104, 28, 28, 0, 0, 0}, \
  {35, 244, 22, 22, 0, 0, 0}, \
  {76, 152, 33, 33, 3, 30, 31}, \
  {86, 5, 53, 53, 3, 50, 51}, \
  {251, 170, 18, 18, 0, 0, 0}, \
  {253, 83, 51, 54, 0, 0, 0}}
#include <mavlink_types.h>

static mavlink_message_t mavMsgTx;
static mavlink_message_t mavChanBufRx;
static mavlink_status_t mavRxStatus;

mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan) {
    (void)chan;
    return &mavChanBufRx;
}

mavlink_status_t* mavlink_get_channel_status(uint8_t chan) {
    (void)chan;
    return &mavRxStatus;
}

#include <mavlink.h>
#include <avr/wdt.h>
#include <EEPROM.h>
AltSoftSerial nmeaSerial;
static mavlink_message_t mavMsg;

// ---- Config Constants ----
#define MAV_BAUD 57600UL
#define NMEA_BAUD 4800UL // 598ci HD
#define TARGET_SYS 1
#define TARGET_COMP 1
#define SYS_ID 1
#define COMP_ID 191
#define ROVER_MODE_HOLD 4
#define ROVER_MODE_LOITER 5
#define ROVER_MODE_GUIDED 15
#define MAX_SPEED_MS 3.5f
#define MIN_SPEED_MS 0.1f
#define SPEED_STEP_MS 0.06f
#define SPEED_DEADBAND_US 200
#define SPEED_TRIM_CHANNEL 3

#define TYPE_MASK ((uint16_t)((1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7)|(1<<8)|(1<<10)|(1<<11)))
#define COORD_FRAME MAV_FRAME_GLOBAL_INT
#define HB_INTERVAL_MS 1000UL
#define SEND_INTERVAL_MS 1000UL
#define RMB_TIMEOUT_MS 3000UL // can be lowered for helix dont change for 598ci HD
#define VTG_TIMEOUT_MS 3000UL
#define DPT_TIMEOUT_MS 3000UL

#define ENABLE_DPT_GLITCH_FILTER 1
#if ENABLE_DPT_GLITCH_FILTER
#define DPT_GLITCH_M 0.6f
#define DPT_GLITCH_NUM_SAMPLES 3
#endif

#define CONTOUR_CIRCLE_RADIUS_M 3.0f
#define CONTOUR_MIN_TURN_RADIUS_M 1.0f
#define CONTOUR_CORRECT_DEPTH_M 0.15f
#define CONTOUR_SAFETY_DEPTH_M 0.8f
#define CONTOUR_GAIN_DEG_PER_M 80.0f
#define CONTOUR_MAX_OFFSET_DEG 90.0f
#define CONTOUR_TREND_DIST_M 4.0f
#define CONTOUR_FLIP_ABORT_M 0.5f
#define CONTOUR_LOST_TIMEOUT_MS 50000UL
#define CONTOUR_FLIP_GATE_MS 3000UL
#define CONTOUR_LOST_DIST_M (CONTOUR_CIRCLE_RADIUS_M * 4)
#define LOITER_RESET_MS 1500UL
#define DEFAULT_SPEED_MS 3.4f
#define METERS_PER_DEG_LAT 111111.0f
#define METERS_PER_INT7_LAT (METERS_PER_DEG_LAT * 1e-7f)
#define INT7_PER_METER_LAT (1.0f / METERS_PER_INT7_LAT)
#define CIRCLE_RADIUS_GAIN 5.0f
#define CIRCLE_RADIUS_MAX_CORR_DEG 45.0f
#define TREND_NUDGE_GAIN 5.0f
#define TREND_NUDGE_MAX_DEG 25.0f
#define TREND_DEPTH_MIN_M 0.05f
#define SPEED_RESEND_THRESHOLD_MS 0.005f
#define SPEED_KEEPALIVE_MS 5000UL
#define MODE_SYNC_RESEND_MS 2000UL
#define NMEA_LINE_TIMEOUT_MS 200UL
#define DEPTH_SLOPE_SAMPLES 4
#define DEPTH_SLOPE_MIN_DIST_M 1.5f
#define DEPTH_SLOPE_CONF_THR 0.01f

// ---- EEPROM Address Map & Breadcrumb Codes ----
#define EEPROM_MAGIC 0xAB13
#define EEPROM_ADDR_MAGIC 0
#define EEPROM_ADDR_TRAVEL_SPEED 2
#define EEPROM_ADDR_FISHING_SPEED 6
#define EEPROM_ADDR_BREADCRUMB 10
#define EEPROM_ADDR_MIN_RAM 12
#define EEPROM_ADDR_RESET_COUNT 14
#define EEPROM_ADDR_SESSION_OK 16
#define SESSION_OK_MAGIC 0xA5
#define EEPROM_WRITE_DELAY_MS 3000UL

#define BC_BOOT 0
#define BC_ENTER_CONTOUR_FOLLOW 1
#define BC_REENTER_LAST_GOOD 2
#define BC_SET_CIRCLE_ANCHOR 3
#define BC_LOITER_WP_RESET_RMB 4
#define BC_LOITER_WP_RESET_APPROACH 5
#define BC_LOITER_WP_RESET_SURVEY 6
#define BC_LOITER_WP_RESET_FOLLOW 7
#define BC_SURVEY_ENTER 8
#define BC_FOLLOW_ENTER 9
#define BC_EEPROM_SPEED_WRITE 10
#define BC_MODE_CONFIRM_TIMEOUT 11
#define BC_VESC_FAILSAFE 12
#define BC_CONTOUR_DEPTH_LOST 13
#define BC_CONTOUR_DEPTH_RECOVERED 14
#define BC_FOLLOW_HEADING_DONE 15
#define BC_FOLLOW_SEND_DONE 16
#define BC_FOLLOW_REENTER 17
#define BC_FOLLOW_LOOP_TOP 18

// ---- Debug Flags ----
#define DEBUG_RMB_PARSE 0
#define DEBUG_GUIDED 1
#define DEBUG_APPROACH 1
#define DEBUG_SURVEY 1
#define DEBUG_FOLLOW 1
#define DEBUG_CONTOUR_TUNING 1
#define DEBUG_SPEED_TRIM 0
#define DEBUG_FREE_RAM 0
#define DEBUG_LINK 0
#define DEBUG_NO_VESC 1
#include <avr/pgmspace.h>

#if DEBUG_FREE_RAM
extern int __heap_start, *__brkval;
static void sendStatusText(uint8_t severity, const char* text);
static void sendStatusText_P(uint8_t severity, const char* text);
static int freeRam() {
    int v;
    return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}
static int16_t minFreeRamSeen = 32767;
static void checkFreeRam(char tag) {
    int r = freeRam();
    if ((int16_t)r < minFreeRamSeen) {
        minFreeRamSeen = (int16_t)r;
        EEPROM.put(EEPROM_ADDR_MIN_RAM, minFreeRamSeen);
        char dbg[24];
        snprintf_P(dbg, sizeof(dbg), PSTR("RAM min=%d %c"), r, tag);
        sendStatusText(MAV_SEVERITY_DEBUG, dbg);
    }
}
#endif

// ---- Global State Variables ----
enum class NavState : uint8_t { GUIDED, LOITER_WP_RESET, CONTOUR_FOLLOW };
enum class ContourPhase : uint8_t { CIRCLE_APPROACH, CIRCLE_SURVEY, FOLLOW };
enum class SpeedSlot : int8_t { NONE = -1, TRAVEL = 0, FISHING = 1 };

static uint16_t resetCount = 0;              // Total processor restarts
static uint8_t lastBreadcrumb = 0;           // Crash origin tracker
static int16_t lastMinRam = 0;               // Historical minimum RAM
static uint32_t loopCount = 0;               // Main iteration counter
static NavState navState = NavState::GUIDED; // Active operational logic state

// rmb* Subsystem (NMEA Route Status)
static bool rmbActive = false;               // Current RMB message valid flag
static bool rmbEverActive = false;           // Indicates if RMB was received since boot
static int32_t rmb_lat = 0;                  // Target longitude from RMB (1e7 deg)
static int32_t rmb_lon = 0;                  // Target latitude from RMB (1e7 deg)
static uint32_t lastRMBms = 0;               // Timestamp of last parsed RMB sentence
static char prevDestWpId[13] = "";           // ID of previous destination WP
static char lastLoiterResetDestId[13] = "";  // ID of WP mapped prior to loiter
static bool routeIsMultiWP = false;          // True if executing multi-waypoint sequence
static bool rmbArrived = false;              // Active when reaching WP destination ring
static bool contourTrigger = false;          // Target transition edge latch
static bool rmbSeenNotArrivedForDest = false;// Wait for stable distance before target latch
static bool posTargetSentForDest = false;    // Has WP target been transmitted
static float lastLegHeadingDeg = 0.0f;       // Desired course towards WP
static bool haveLastLegHeading = false;      // RMB valid course parsed
static bool followTriggeredForDest = false;  // Contour activation lock

// vtg* Subsystem (NMEA Course/Speed)
static bool haveVTG = false;                 // Current VTG message valid flag
static bool haveVTGcog = false;              // Current VTG holds course data
static float vtg_cog_deg = 0.0f;             // Course over ground (deg)
static uint32_t lastVTGms = 0;               // Timestamp of last valid VTG

// depth*/dpt* Subsystem (Sonar Depth)
static bool haveDPT = false;                 // Current DPT message valid flag
static float depth_m = 0.0f;                 // Instantaneous depth (meters)
static float depthEma_m = 0.0f;              // Low-pass filtered depth (meters)
static bool haveDptEma = false;              // Moving average valid flag
#define DPT_EMA_ALPHA 0.1f
static uint32_t lastDPTms = 0;               // Timestamp of last valid DPT
#if ENABLE_DPT_GLITCH_FILTER
static int8_t dptGlitchCount = 0;            // Accumulator to reject anomalous readings
#endif

// contour* Subsystem (Contour Control Logic)
static ContourPhase contourPhase = ContourPhase::CIRCLE_APPROACH;
static float activationCosLat = 1.0f;
static float contourHeadingDeg = 0.0f;
static float contourBaseHeadingDeg = 0.0f;
static float contourTargetDepth_m = 0.0f;
static float contourKpSigned = 1.0f;
static float lastContourOffsetDeg = 0.0f;
static float activationHeadingDeg = 0.0f;
static int32_t activationLat_int = 0;
static int32_t activationLon_int = 0;
static bool activationPosValid = false;
static bool circleCrossFound = false;
static float circleBestHeading = 0.0f;
static bool circleBestDeepRight = false;
static float circleBestDev = 999.0f;
static int32_t circleBestLat_int = 0;
static int32_t circleBestLon_int = 0;
static float circleAccumAngle = 0.0f;
static float circlePrevTheta = 0.0f;
static int32_t circlePrevLat_int = 0;
static int32_t circlePrevLon_int = 0;
static float circleLastDepth = 0.0f;
static float circleApproachAccumAngle = 0.0f;
static float circleApproachPrevTheta = 0.0f;
static bool circleApproachSampled = false;
static int32_t approachCenterLat_int = 0;
static int32_t approachCenterLon_int = 0;
static bool surveyDptWasValid = false;
static int16_t depthSlopeBufDepth[DEPTH_SLOPE_SAMPLES];
static int16_t depthSlopeBufDist[DEPTH_SLOPE_SAMPLES];
static uint8_t depthSlopeHead = 0;
static uint8_t depthSlopeCount = 0;
static float depthSlopeCumDist = 0.0f;
static float followPrevError_m = 0.0f;
static uint32_t lastSideFlipMs = 0;
static bool contourTrendRefSet = false;
static bool contourHoldTargetSent = false;
static int32_t contourTrendRefLat_int = 0;
static int32_t contourTrendRefLon_int = 0;
static float contourTrendRefDepth = 0.0f;
static float contourTrendRefCosLat = 1.0f;
static bool depthLost = false;
#define CONTOUR_DPT_LOSS_MS 4000UL
static int32_t contourHoldLat_int = 0;
static int32_t contourHoldLon_int = 0;
static bool contourLastGoodValid = false;
static int32_t contourLastGoodLat_int = 0;
static int32_t contourLastGoodLon_int = 0;
static float contourLastGoodHeadingDeg = 0.0f;
static uint32_t contourLastGoodMs = 0;

// vesc* & controller Subsystem
static uint8_t pixhawkMode = 255;            // Cached MAVLink auto-pilot mode
static bool pixhawkModeKnown = false;        // Indicates if mode has been initially read
static uint8_t lastHbSysid = 0;              // System ID of transmitting host
static uint16_t hbRxCount = 0;               // Total heartbeat packets received
static int32_t vescState = -1;               // Arm/Disarm VESC node status
static uint32_t lastVescStMs = 0;            // Timestamp of latest VESC health pack
static bool vescStaleOrBad = true;           // Flag for active node dropout
static bool vescDisarmSent = false;          // True if automatic disarm was fired
static const uint32_t VESC_ST_STALE_MS = 1000;
static bool vescNotActive = true;            // Hardware halt override
static bool crashLoopMode = false;           // Reboot storm protector
static bool sessionOkWritten = false;        // Lock indicating stable run
static uint32_t loiterStartMs = 0;           // Timer to clear waypoint path
static bool loiterConfirmed = false;         // Pixhawk asserted loiter mode
static uint32_t lastSend = 0;                // MAVLink periodic clock
static bool waypointReset = false;           // Needs path regeneration
static uint32_t lastHeartbeat = 0;           // Next packet timestamp
static int16_t pendingMode = -1;             // Desired operating mode
static uint32_t pendingStartMs = 0;          // Timeout tracker for switch validation
static uint32_t pendingLastSendMs = 0;       // Resend delay for commands
#define MODE_CONFIRM_TIMEOUT_MS 5000UL
static bool modeConfirmFailsafe = false;     // Triggered on MAVLink failure
static bool holdLatched = false;             // Mandatory manual override mode lock
static float pixhawkHeadingDeg = 0.0f;       // Active IMU absolute orientation
static bool havePixhawkHeading = false;      // Valid heading parsed flag
static float currentLat_deg = 0.0f;          // Absolute global current position
static int32_t currentLat_int = 0;           // Global position (1e7 deg)
static int32_t currentLon_int = 0;           // Global position (1e7 deg)
static bool haveCurrentPos = false;          // Position structure valid flag

// speed & RC-trim Subsystem
static uint8_t prevTrimState = 0;            // RC channel speed adjustment position
static float travelSpeed = 0.0f;             // Active linear movement velocity (m/s)
static float fishingSpeed = 0.0f;            // Active surveying movement velocity (m/s)
static SpeedSlot activeSpeedSlot = SpeedSlot::NONE;
static float lastSentSpeed = 0.0f;           // Cached command comparison variable
static bool speedDirty = false;              // Active trim awaiting EEPROM commit
static uint32_t speedChangedMs = 0;          // EEPROM commit delay timer
static uint32_t lastSpeedSendMs = 0;         // Keepalive timestamp
static bool speedNeedsResend = false;        // Resend request flag after mode switches

// ---- NMEA Parsing Helpers ----
static char nmeaBuf[84];
static uint8_t nmeaIdx = 0;
static uint8_t mavBuf[70];

static float parseFloat(const char* s) {
    if (!s || !*s) return 0.0f;
    float res = 0.0f, fact = 1.0f;
    bool neg = (*s == '-');
    if (neg) s++;
    while (*s >= '0' && *s <= '9') {
        res = res * 10.0f + (*s - '0');
        s++;
    }
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') {
            fact *= 0.1f;
            res += (*s - '0') * fact;
            s++;
        }
    }
    return neg ? -res : res;
}

static bool nmeaChecksumValid(const char* s) {
    if (*s != '$') return false;
    uint8_t cs = 0;
    const char* p = s + 1;
    while (*p && *p != '*') {
        cs ^= (uint8_t)(*p++);
    }
    if (*p != '*') return false;
    p++;
    char hi = p[0], lo = p[1];
    if (!isxdigit((uint8_t)hi) || !isxdigit((uint8_t)lo)) return false;
    uint8_t h = (hi >= 'a') ? (uint8_t)(hi - 'a' + 10) : (hi >= 'A') ? (uint8_t)(hi - 'A' + 10) : (uint8_t)(hi - '0');
    uint8_t l = (lo >= 'a') ? (uint8_t)(lo - 'a' + 10) : (lo >= 'A') ? (uint8_t)(lo - 'A' + 10) : (uint8_t)(lo - '0');
    return cs == ((h << 4) | l);
}

static uint8_t splitFields(char* buf, char* fields[], uint8_t maxFields) {
    uint8_t n = 0;
    char* p = buf + 1;
    if (n < maxFields) fields[n++] = p;
    while (*p && *p != '*' && n < maxFields) {
        if (*p == ',') {
            *p = '\0';
            fields[n++] = p + 1;
        }
        p++;
    }
    if (*p != '*' && *p != '\0') {
        while (*p && *p != '*' && *p != ',') p++;
    }
    if (*p == '*' || *p == ',') *p = '\0';
    return n;
}

static int32_t parseDDMM(const char* s, char hemi) {
    if (!s || !*s) return 0;
    const char* dot = strchr(s, '.');
    if (!dot) return 0;
    int32_t lenBeforeDot = (int32_t)(dot - s);
    if (lenBeforeDot != 4 && lenBeforeDot != 5) return 0;
    int32_t val = 0;
    const char* p = s;
    while (*p >= '0' && *p <= '9') {
        val = val * 10 + (*p - '0');
        p++;
    }
    int32_t degrees = val / 100;
    int32_t minutes_us = (val % 100) * 1000000L;
    if (*p == '.') {
        p++;
        int32_t mult = 100000L;
        uint8_t digs = 0;
        while (*p >= '0' && *p <= '9' && digs < 6) {
            minutes_us += (*p - '0') * mult;
            mult /= 10;
            p++; digs++;
        }
    }
    int32_t scaled = (degrees * 10000000L) + ((minutes_us + 3) / 6);
    if (hemi == 'S' || hemi == 'W') scaled = -scaled;
    return scaled;
}

// ---- NMEA Sentence Handlers ----
static void handleRMB(char* fields[], uint8_t n) {
    if (n < 14) return;
    if (fields[1][0] != 'A') { return; }

    int32_t lat = parseDDMM(fields[6], fields[7][0]);
    int32_t lon = parseDDMM(fields[8], fields[9][0]);

    if (lat == 0 && lon == 0) return;
    if (lat < -900000000L || lat > 900000000L) return;
    if (lon < -1800000000L || lon > 1800000000L) return;

    rmb_lat = lat;
    rmb_lon = lon;

#if DEBUG_FREE_RAM
    checkFreeRam('R');
#endif

    bool destChanged = (strncmp(fields[5], prevDestWpId, sizeof(prevDestWpId) - 1) != 0);
    bool isOriginStart = (strncasecmp(fields[4], "START", 5) == 0);

    if (fields[11][0]) {
        lastLegHeadingDeg = parseFloat(fields[11]);
        haveLastLegHeading = true;
    }

    if (destChanged) {
        haveLastLegHeading = false;
        bool isResume = isOriginStart
            && lastLoiterResetDestId[0] != '\0'
            && (strncmp(fields[5], lastLoiterResetDestId, sizeof(lastLoiterResetDestId) - 1) == 0);

        strncpy(prevDestWpId, fields[5], sizeof(prevDestWpId) - 1);
        prevDestWpId[sizeof(prevDestWpId) - 1] = '\0';
        posTargetSentForDest = false;
        followTriggeredForDest = false;
        rmbSeenNotArrivedForDest = false;
        routeIsMultiWP = (!isOriginStart) || isResume;

        activeSpeedSlot = routeIsMultiWP ? SpeedSlot::FISHING : SpeedSlot::TRAVEL;
        rmbArrived = false;
        contourTrigger = false;
    }

    if (fields[13][0] == 'A') {
        rmbArrived = true;
        if (routeIsMultiWP && !followTriggeredForDest && rmbSeenNotArrivedForDest) {
            contourTrigger = true;
            followTriggeredForDest = true;
        }
    } else {
        rmbArrived = false;
        rmbSeenNotArrivedForDest = true;
    }

    rmbActive = true;
    rmbEverActive = true;
    lastRMBms = millis();

#if DEBUG_RMB_PARSE
    {
        char dbg[64];
        snprintf_P(dbg, sizeof(dbg),
            PSTR("RMB dC=%d arr=%d mWP=%d trg=%d slot=%d id=%s"),
            (int)destChanged, (int)rmbArrived, (int)routeIsMultiWP,
            (int)contourTrigger,
            (int)(int8_t)SpeedSlot, prevDestWpId);
        sendStatusText(MAV_SEVERITY_DEBUG, dbg);
    }
#endif
}

static void handleVTG(char* fields[], uint8_t n) {
    if (n < 6 || !fields[5][0]) return;
    if (n >= 10 && fields[9][0] == 'N') return;
    if (!isdigit((uint8_t)fields[5][0]) && fields[5][0] != '.') return;

    haveVTG = true;
    lastVTGms = millis();

    if (fields[1][0]) {
        vtg_cog_deg = parseFloat(fields[1]);
        haveVTGcog = true;
    } else {
        haveVTGcog = false;
    }
}

static void handleDPT(char* fields[], uint8_t n) {
    if (n < 2 || !fields[1][0]) return;
    float d = parseFloat(fields[1]);
    if (d <= 0.0f) return;

    bool accept = true;
#if ENABLE_DPT_GLITCH_FILTER
    float glitch = d - depth_m;
    if (glitch >= DPT_GLITCH_M) {
        if (dptGlitchCount < 0) dptGlitchCount = 0;
        dptGlitchCount++;
        accept = (dptGlitchCount >= DPT_GLITCH_NUM_SAMPLES);
        if (accept) dptGlitchCount = 0;
    } else if (glitch <= -DPT_GLITCH_M) {
        if (dptGlitchCount > 0) dptGlitchCount = 0;
        dptGlitchCount--;
        accept = (dptGlitchCount <= -DPT_GLITCH_NUM_SAMPLES);
        if (accept) dptGlitchCount = 0;
    } else {
        dptGlitchCount = 0;
    }
#if DEBUG_FOLLOW || DEBUG_SURVEY
    {
        char dbg[48];
        char dRaw[8], dGl[8];
        dtostrf(d, 6, 2, dRaw);
        dtostrf(glitch, 6, 2, dGl);
        snprintf_P(dbg, sizeof(dbg), PSTR("DPT raw=%s gl=%s cnt=%d acc=%d"),
                 dRaw, dGl, (int)dptGlitchCount, (int)accept);
        sendStatusText(MAV_SEVERITY_DEBUG, dbg);
    }
#endif
#endif

    haveDPT = true;
    lastDPTms = millis();

    if (accept) {
        depth_m = d;
        depthEma_m = haveDptEma ? (depthEma_m * (1.0f - DPT_EMA_ALPHA) + d * DPT_EMA_ALPHA) : d;
        haveDptEma = true;
    }
    depthLost = false;
}

static uint32_t lastNmeaByteMs = 0;

static void readNMEA() {
    uint8_t guard = 0;
    while (nmeaSerial.available() && guard < 64) {
        guard++;
        wdt_reset();
        char c = (char)nmeaSerial.read();
        lastNmeaByteMs = millis();

        if (c == '$') {
            nmeaIdx = 0;
            nmeaBuf[nmeaIdx++] = c;
            continue;
        }
        if (nmeaIdx == 0) continue;

        if (c == '\r' || c == '\n') {
            nmeaBuf[nmeaIdx] = '\0';
            if (nmeaIdx > 6 && nmeaChecksumValid(nmeaBuf)) {
#if DEBUG_LINK
                if (strstr(nmeaBuf, "RMB") != NULL) {
                    sendStatusText(MAV_SEVERITY_DEBUG, nmeaBuf);
                }
#endif
                char* fields[16];
                uint8_t n = splitFields(nmeaBuf, fields, 16);
                if (n > 0) {
                    const char* type = fields[0] + 2;
                    if (strncmp(type, "RMB", 3) == 0) handleRMB(fields, n);
                    else if (strncmp(type, "VTG", 3) == 0) handleVTG(fields, n);
                    else if (strncmp(type, "DPT", 3) == 0) handleDPT(fields, n);
                }
            }
            nmeaIdx = 0;
            continue;
        }

        if (nmeaIdx < (uint8_t)(sizeof(nmeaBuf) - 1)) {
            nmeaBuf[nmeaIdx++] = c;
        } else {
            nmeaIdx = 0;
        }
    }

    if (nmeaIdx > 0 && (millis() - lastNmeaByteMs) > NMEA_LINE_TIMEOUT_MS) {
        nmeaIdx = 0;
    }
}

// ---- MAVLink Send Helpers ----
#define HB_LED_PIN 13
#if DEBUG_LINK
static void dbgCmd(const char* fmt, int val) {
    char dbg[24];
    snprintf(dbg, sizeof(dbg), fmt, val);
    sendStatusText(MAV_SEVERITY_DEBUG, dbg);
}
#endif

static bool mavSend() {
    uint16_t len = mavlink_msg_to_send_buffer(mavBuf, &mavMsgTx);
    uint32_t waitStart = millis();
    while (Serial.availableForWrite() < len) {
        if (millis() - waitStart > 20) return false;
    }
    Serial.write(mavBuf, len);
    return true;
}

static void sendHeartbeat() {
    digitalWrite(HB_LED_PIN, !digitalRead(HB_LED_PIN));
    mavlink_msg_heartbeat_pack(SYS_ID, COMP_ID, &mavMsgTx, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_ACTIVE);
    mavSend();
}

static bool sendSetMode(uint8_t mode) {
    mavlink_msg_set_mode_pack(SYS_ID, COMP_ID, &mavMsgTx, TARGET_SYS, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (uint32_t)mode);
    if (!mavSend()) return false;
#if DEBUG_LINK
    dbgCmd("CMD SET_MODE=%u", mode);
#endif
    return true;
}

static bool requestModeChange(uint8_t mode, uint32_t now) {
    if (holdLatched && mode != ROVER_MODE_HOLD) {
        if (pendingMode == (int16_t)mode) pendingMode = -1;
        return false;
    }
    if (pixhawkModeKnown && pixhawkMode == ROVER_MODE_HOLD && mode != ROVER_MODE_HOLD) {
        if (pendingMode == (int16_t)mode) pendingMode = -1;
        return false;
    }
    if (pixhawkModeKnown && pixhawkMode == mode) {
        pendingMode = -1;
        return true;
    }
    if (pendingMode != (int16_t)mode) {
        pendingMode = (int16_t)mode;
        pendingStartMs = now;
        pendingLastSendMs = now - MODE_SYNC_RESEND_MS;
    }
    if (now - pendingLastSendMs >= MODE_SYNC_RESEND_MS) {
        if (sendSetMode(mode)) {
            pendingLastSendMs = now;
        }
    }
    if (mode != ROVER_MODE_HOLD && (now - pendingStartMs) >= MODE_CONFIRM_TIMEOUT_MS) {
        pendingMode = -1;
        modeConfirmFailsafe = true;
    }
    return false;
}

static bool sendPositionTarget(int32_t lat, int32_t lon) {
    mavlink_set_position_target_global_int_t sp = {};
    sp.time_boot_ms = (uint32_t)millis();
    sp.target_system = TARGET_SYS;
    sp.target_component = TARGET_COMP;
    sp.coordinate_frame = COORD_FRAME;
    sp.type_mask = TYPE_MASK;
    sp.lat_int = lat;
    sp.lon_int = lon;
    mavlink_msg_set_position_target_global_int_encode(SYS_ID, COMP_ID, &mavMsgTx, &sp);
    return mavSend();
}

static void sendVelocityTarget(float headingDeg, float speed_ms) {
    if (vescNotActive) speed_ms = 0.0f;
    float hRad = headingDeg * (float)M_PI / 180.0f;
    mavlink_set_position_target_global_int_t sp = {};
    sp.time_boot_ms = (uint32_t)millis();
    sp.target_system = TARGET_SYS;
    sp.target_component = TARGET_COMP;
    sp.coordinate_frame = COORD_FRAME;
    sp.type_mask = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << 8) | (1 << 10) | (1 << 11);
    sp.vx = speed_ms * cosf(hRad);
    sp.vy = speed_ms * sinf(hRad);
    mavlink_msg_set_position_target_global_int_encode(SYS_ID, COMP_ID, &mavMsgTx, &sp);
    mavSend();
}

static bool sendSpeedCommand(float speed_ms) {
    mavlink_msg_command_long_pack(SYS_ID, COMP_ID, &mavMsgTx, TARGET_SYS, TARGET_COMP, MAV_CMD_DO_CHANGE_SPEED, 0, 1, speed_ms, -1, 0, 0, 0, 0);
    if (!mavSend()) return false;
#if DEBUG_LINK
    dbgCmd("CMD SPEED=%d cm/s", (int)(speed_ms * 100.0f));
#endif
    return true;
}

static bool sendDisarm() {
    mavlink_msg_command_long_pack(SYS_ID, COMP_ID, &mavMsgTx, TARGET_SYS, TARGET_COMP, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0);
    if (!mavSend()) return false;
#if DEBUG_LINK
    sendStatusText_P(MAV_SEVERITY_DEBUG, PSTR("CMD DISARM"));
#endif
    return true;
}

static void sendStatusText(uint8_t severity, const char* text) {
    mavlink_msg_statustext_pack(SYS_ID, COMP_ID, &mavMsgTx, severity, text, 0, 0);
    mavSend();
}

static void sendStatusText_P(uint8_t severity, const char* text) {
    char buf[50];
    strncpy_P(buf, text, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    sendStatusText(severity, buf);
}

// ---- MAVLink Receive Processing ----
static void readMAVLink() {
    uint8_t maxBytesPerCall = 64;
    while (Serial.available() && maxBytesPerCall--) {
        uint8_t c = (uint8_t)Serial.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &mavMsg, &mavRxStatus)) {
#if DEBUG_FREE_RAM
            checkFreeRam('M');
#endif
            if (mavMsg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                lastHbSysid = mavMsg.sysid;
                hbRxCount++;
            }
            if (mavMsg.msgid == MAVLINK_MSG_ID_HEARTBEAT && mavMsg.sysid == TARGET_SYS) {
                mavlink_heartbeat_t hb;
                mavlink_msg_heartbeat_decode(&mavMsg, &hb);
                uint8_t newPixhawkMode = (uint8_t)hb.custom_mode;

                if (newPixhawkMode == ROVER_MODE_GUIDED && pixhawkMode != ROVER_MODE_GUIDED && navState == NavState::GUIDED) {
                    speedNeedsResend = true;
                }
                if (holdLatched && newPixhawkMode != ROVER_MODE_HOLD) {
                    holdLatched = false;
                }
                pixhawkMode = newPixhawkMode;
                pixhawkModeKnown = true;
            }
            if (mavMsg.msgid == MAVLINK_MSG_ID_RC_CHANNELS_RAW) {
                mavlink_rc_channels_raw_t rc;
                mavlink_msg_rc_channels_raw_decode(&mavMsg, &rc);
                static_assert(SPEED_TRIM_CHANNEL >= 1 && SPEED_TRIM_CHANNEL <= 8, "SPEED_TRIM_CHANNEL must be 1-8");
                static_assert(offsetof(mavlink_rc_channels_raw_t, chan2_raw) - offsetof(mavlink_rc_channels_raw_t, chan1_raw) == sizeof(uint16_t), "RC channel fields are not contiguous — use switch instead");
                static_assert(offsetof(mavlink_rc_channels_raw_t, chan8_raw) - offsetof(mavlink_rc_channels_raw_t, chan1_raw) == 7u * sizeof(uint16_t), "RC channel fields are not contiguous — use switch instead");

                const uint16_t* chans = &rc.chan1_raw;
                uint16_t trimChannelPW = chans[SPEED_TRIM_CHANNEL - 1];
                uint8_t trimState = 0;

                if (trimChannelPW > 1500U + SPEED_DEADBAND_US) trimState = 1;
                else if (trimChannelPW > 0 && trimChannelPW < 1500U - SPEED_DEADBAND_US) trimState = 2;

                if (trimState != prevTrimState) {
                    prevTrimState = trimState;
                    if (trimState != 0 && activeSpeedSlot != SpeedSlot::NONE &&
                        ((navState == NavState::GUIDED && rmbActive && (millis() - lastRMBms) < RMB_TIMEOUT_MS) || navState == NavState::CONTOUR_FOLLOW)) {

                        float &spd = (activeSpeedSlot == SpeedSlot::TRAVEL) ? travelSpeed : fishingSpeed;
                        if (trimState == 1) {
                            spd += SPEED_STEP_MS;
                            if (spd > MAX_SPEED_MS) spd = MAX_SPEED_MS;
                        } else {
                            spd -= SPEED_STEP_MS;
                            if (spd < MIN_SPEED_MS) spd = MIN_SPEED_MS;
                        }

                        speedDirty = true;
                        speedChangedMs = millis();

#if DEBUG_SPEED_TRIM
                        {
                            char dbg[40];
                            char spdStr[8];
                            dtostrf(spd, 4, 2, spdStr);
                            snprintf(dbg, sizeof(dbg), "TRIM st=%u slot=%d spd=%s", (unsigned)trimState, (int)(int8_t)activeSpeedSlot, spdStr);
                            sendStatusText(MAV_SEVERITY_DEBUG, dbg);
                        }
#endif
                        if (navState != NavState::CONTOUR_FOLLOW) {
                            if (sendSpeedCommand(spd)) {
                                lastSentSpeed = spd;
                                lastSpeedSendMs = millis();
                                speedNeedsResend = false;
#if DEBUG_SPEED_TRIM
                                sendStatusText_P(MAV_SEVERITY_DEBUG, PSTR("TRIM: sent immediate"));
#endif
                            }
                        }
                    }
                }
            }
            if (mavMsg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                mavlink_global_position_int_t gp;
                mavlink_msg_global_position_int_decode(&mavMsg, &gp);
                currentLat_int = gp.lat;
                currentLon_int = gp.lon;
                currentLat_deg = gp.lat * 1e-7f;
                haveCurrentPos = true;

                if (gp.hdg <= 36000) {
                    pixhawkHeadingDeg = gp.hdg * 0.01f;
                    havePixhawkHeading = true;
                } else {
                    havePixhawkHeading = false;
                }
            }
            if (mavMsg.msgid == MAVLINK_MSG_ID_NAMED_VALUE_FLOAT) {
                mavlink_named_value_float_t nv;
                mavlink_msg_named_value_float_decode(&mavMsg, &nv);
                if (strncmp(nv.name, "VESC_ST", 10) == 0) {
                    vescState = (int32_t)lroundf(nv.value);
                    lastVescStMs = millis();
                    vescStaleOrBad = false;
                    if (vescState != 4) vescDisarmSent = false;
                }
            }
        }
    }
}

// ---- Contour Control Logic ----
static void gotoLoiterReset(uint8_t breadcrumb) {
    loiterConfirmed = false;
    navState = NavState::LOITER_WP_RESET;
    EEPROM.update(EEPROM_ADDR_BREADCRUMB, breadcrumb);
}

static void setCircleAnchor(float lat, int32_t lat_int, int32_t lon_int, float headingDeg) {
#if DEBUG_FREE_RAM
    checkFreeRam('A');
#endif
    EEPROM.update(EEPROM_ADDR_BREADCRUMB, BC_SET_CIRCLE_ANCHOR);
    activationHeadingDeg = headingDeg;
    activationLat_int = lat_int;
    activationCosLat = cosf(lat * (float)M_PI / 180.0f);
    activationLon_int = lon_int;
    activationPosValid = true;

    float approachHeadingDeg = havePixhawkHeading ? pixhawkHeadingDeg : vtg_cog_deg;
    float hr = approachHeadingDeg * (float)M_PI / 180.0f;
    approachCenterLat_int = activationLat_int + (int32_t)((-sinf(hr)) * (CONTOUR_CIRCLE_RADIUS_M * 0.5f) * INT7_PER_METER_LAT);
    approachCenterLon_int = activationLon_int + (int32_t)((cosf(hr)) * (CONTOUR_CIRCLE_RADIUS_M * 0.5f) * INT7_PER_METER_LAT / activationCosLat);

    circleApproachAccumAngle = 0.0f;
    circleApproachPrevTheta = 0.0f;
    circleApproachSampled = false;
    circleCrossFound = false;
    circleBestDev = 999.0f;
    circleAccumAngle = 0.0f;
    circlePrevLat_int = lat_int;
    circlePrevLon_int = lon_int;
    circlePrevTheta = activationHeadingDeg;
    circleLastDepth = depth_m;

    depthSlopeHead = 0;
    depthSlopeCount = 0;
    depthSlopeCumDist = 0.0f;
    contourTrendRefSet = false;
    followPrevError_m = 0.0f;
    lastContourOffsetDeg = 0.0f;
    contourLastGoodMs = millis();

#if DEBUG_FREE_RAM
    checkFreeRam('G');
#endif
}

static void enterContourFollow() {
    EEPROM.update(EEPROM_ADDR_BREADCRUMB, BC_ENTER_CONTOUR_FOLLOW);
    float entryHeadingDeg = lastLegHeadingDeg;
    contourHeadingDeg = entryHeadingDeg;
    contourBaseHeadingDeg = entryHeadingDeg;
    contourTargetDepth_m = haveDptEma ? depthEma_m : depth_m;

    haveDptEma = false;
    depthEma_m = 0.0f;

    setCircleAnchor(haveCurrentPos ? currentLat_deg : 0.0f,
                    haveCurrentPos ? currentLat_int : 0,
                    haveCurrentPos ? currentLon_int : 0,
                    entryHeadingDeg);

    activationPosValid = haveCurrentPos;
    contourLastGoodValid = false;
    contourPhase = ContourPhase::CIRCLE_APPROACH;
    contourTrigger = false;
    navState = NavState::CONTOUR_FOLLOW;
    sendSpeedCommand(fishingSpeed);
    activeSpeedSlot = SpeedSlot::FISHING;

#if DEBUG_FREE_RAM
    checkFreeRam('Z');
#endif

#if DEBUG_GUIDED || DEBUG_APPROACH || DEBUG_FOLLOW || DEBUG_SURVEY
    char depthStr[8];
    dtostrf(contourTargetDepth_m, 4, 2, depthStr);
    char activMsg[40];
    snprintf(activMsg, sizeof(activMsg), "CONTOUR FOLLOW: activated %sm", depthStr);
    sendStatusText(MAV_SEVERITY_INFO, activMsg);
#endif
}

static void reenterContourFromLastGood() {
    EEPROM.update(EEPROM_ADDR_BREADCRUMB, BC_REENTER_LAST_GOOD);
    if (contourLastGoodValid) {
        float lat_deg = contourLastGoodLat_int * 1e-7f;
        contourHeadingDeg = contourLastGoodHeadingDeg;
        contourBaseHeadingDeg = contourLastGoodHeadingDeg;
        setCircleAnchor(lat_deg, contourLastGoodLat_int, contourLastGoodLon_int, contourLastGoodHeadingDeg);
        activationPosValid = true;
    } else {
        setCircleAnchor(haveCurrentPos ? currentLat_deg : 0.0f,
                        haveCurrentPos ? currentLat_int : 0,
                        haveCurrentPos ? currentLon_int : 0,
                        contourHeadingDeg);
        activationPosValid = haveCurrentPos;
    }

    contourPhase = ContourPhase::CIRCLE_SURVEY;
    sendSpeedCommand(travelSpeed);
    activeSpeedSlot = SpeedSlot::TRAVEL;

#if DEBUG_FOLLOW || DEBUG_APPROACH
    sendStatusText_P(MAV_SEVERITY_WARNING, PSTR("CONTOUR: line lost -> re-circle"));
    sendStatusText_P(MAV_SEVERITY_INFO, PSTR("CONTOUR FOLLOW: re-activated"));
#endif
}

// ---- Setup ----
void setup() {
    wdt_disable();
#if DEBUG_FREE_RAM
    uint8_t sessionOk = EEPROM.read(EEPROM_ADDR_SESSION_OK);
    crashLoopMode = (sessionOk != SESSION_OK_MAGIC);
    EEPROM.update(EEPROM_ADDR_SESSION_OK, 0);   // disarm - must be re-earned this session
#endif

    pinMode(HB_LED_PIN, OUTPUT);
    wdt_enable(WDTO_2S);
    Serial.begin(MAV_BAUD);
    nmeaSerial.begin(NMEA_BAUD);
    lastSend = millis();

    uint16_t magic = 0;
    EEPROM.get(EEPROM_ADDR_MAGIC, magic);
    if (magic == EEPROM_MAGIC) {
        EEPROM.get(EEPROM_ADDR_TRAVEL_SPEED, travelSpeed);
        EEPROM.get(EEPROM_ADDR_FISHING_SPEED, fishingSpeed);
        if (travelSpeed < MIN_SPEED_MS || travelSpeed > MAX_SPEED_MS) travelSpeed = DEFAULT_SPEED_MS;
        if (fishingSpeed < MIN_SPEED_MS || fishingSpeed > MAX_SPEED_MS) fishingSpeed = DEFAULT_SPEED_MS;
    } else {
        travelSpeed = DEFAULT_SPEED_MS;
        fishingSpeed = DEFAULT_SPEED_MS;
    }

    lastHeartbeat = millis();
    lastSpeedSendMs = millis() - 5001UL;
    waypointReset = true;
    lastBreadcrumb = EEPROM.read(EEPROM_ADDR_BREADCRUMB);
    EEPROM.update(EEPROM_ADDR_BREADCRUMB, BC_BOOT);

    #define SESSION_OK_LOOP_MIN 500UL
    EEPROM.get(EEPROM_ADDR_MIN_RAM, lastMinRam);
    EEPROM.get(EEPROM_ADDR_RESET_COUNT, resetCount);
    if (resetCount == 0xFFFF) resetCount = 0;
    resetCount++;
    EEPROM.put(EEPROM_ADDR_RESET_COUNT, resetCount);

#if DEBUG_FREE_RAM
    char bcMsg[40];
    snprintf_P(bcMsg, sizeof(bcMsg), PSTR("BOOT last_loc=%u cnt=%u"), (unsigned)lastBreadcrumb, (unsigned)resetCount);
    sendStatusText(MAV_SEVERITY_INFO, bcMsg);
#endif
}

// ---- Extracted Loop Handlers ----
static bool updateWatchdogAndDiagnostics() {
    wdt_reset();
    loopCount++;

#if DEBUG_FREE_RAM
    if (!sessionOkWritten && millis() >= 5000UL && loopCount >= SESSION_OK_LOOP_MIN) {
        EEPROM.update(EEPROM_ADDR_SESSION_OK, SESSION_OK_MAGIC);
        sessionOkWritten = true;
    }
    if (crashLoopMode) {
        static uint32_t lastCrashDumpMs = 0;
        if (millis() - lastCrashDumpMs >= 3000UL) {
            lastCrashDumpMs = millis();
            char dbg[64];
            snprintf(dbg, sizeof(dbg), "CRASHLOOP bc=%u minram=%d", (unsigned)lastBreadcrumb, (int)lastMinRam);
            sendStatusText(MAV_SEVERITY_CRITICAL, dbg);
        }
        return false;
    }
#endif
    return true;
}

static void handleVescMonitoring(uint32_t now) {
#if DEBUG_NO_VESC
    vescStaleOrBad = false;
    vescNotActive = false;
    bool vescFailsafe = false;
    (void)vescDisarmSent;
#else
    bool vescWasStale = vescStaleOrBad;
    if (!vescStaleOrBad && (now - lastVescStMs) >= VESC_ST_STALE_MS) {
        vescStaleOrBad = true;
    }
    if (vescStaleOrBad && !vescWasStale) {
        sendStatusText_P(MAV_SEVERITY_ERROR, PSTR("VESC: no data -> HOLD+DISARM"));
    }
    vescNotActive = vescStaleOrBad || (vescState != 2);
    bool vescFailsafe = (!vescStaleOrBad && vescState == 4);
    if (vescFailsafe || vescStaleOrBad) {
        EEPROM.update(EEPROM_ADDR_BREADCRUMB, BC_VESC_FAILSAFE);
        holdLatched = true;
        requestModeChange(ROVER_MODE_HOLD, now);
        if (!vescDisarmSent && sendDisarm()) {
            vescDisarmSent = true;
            if (vescFailsafe) {
                sendStatusText_P(MAV_SEVERITY_CRITICAL, PSTR("VESC: FAILSAFE -> DISARM"));
            }
        }
    }
#endif
}

static void handleModeConfirmFailsafe(uint32_t now) {
    if (modeConfirmFailsafe) {
        modeConfirmFailsafe = false;
        if (!(pixhawkModeKnown && pixhawkMode == ROVER_MODE_HOLD)) {
            EEPROM.update(EEPROM_ADDR_BREADCRUMB, BC_MODE_CONFIRM_TIMEOUT);
            holdLatched = true;
            sendStatusText_P(MAV_SEVERITY_CRITICAL, PSTR("MODE CONFIRM TIMEOUT->HOLD"));
        }
    }
    if (holdLatched && !(pixhawkModeKnown && pixhawkMode == ROVER_MODE_HOLD)) {
        requestModeChange(ROVER_MODE_HOLD, now);
    }
}

static void handleGuidedState(uint32_t now, bool rmbFresh) {
    if (rmbFresh) {
        if (pixhawkModeKnown && pixhawkMode != ROVER_MODE_GUIDED) {
            requestModeChange(ROVER_MODE_GUIDED, now);
        } else if (pixhawkModeKnown) {
            waypointReset = false;
            if (pendingMode == (int16_t)ROVER_MODE_GUIDED) pendingMode = -1;

            if (contourTrigger && (!haveDPT || !haveVTGcog)) {
                if (!posTargetSentForDest && sendPositionTarget(rmb_lat, rmb_lon)) {
                    posTargetSentForDest = true;
                }
            } else {
                float commandedSpeed = vescNotActive ? 0.0f :
                                       (activeSpeedSlot == SpeedSlot::TRAVEL) ? travelSpeed : (activeSpeedSlot == SpeedSlot::FISHING) ? fishingSpeed : DEFAULT_SPEED_MS;

                if (speedNeedsResend) {
                    if (sendSpeedCommand(commandedSpeed)) {
                        speedNeedsResend = false;
                        lastSpeedSendMs = now;
                        lastSentSpeed = commandedSpeed;
#if DEBUG_SPEED_TRIM
                        sendStatusText_P(MAV_SEVERITY_DEBUG, PSTR("TRIM: resend-flag send"));
#endif
                    }
                } else {
                    if (!posTargetSentForDest && sendPositionTarget(rmb_lat, rmb_lon)) {
                        posTargetSentForDest = true;
                    }
                    bool speedChanged = fabsf(commandedSpeed - lastSentSpeed) > SPEED_RESEND_THRESHOLD_MS;
                    if (speedChanged || (now - lastSpeedSendMs >= SPEED_KEEPALIVE_MS)) {
                        if (sendSpeedCommand(commandedSpeed)) {
#if DEBUG_SPEED_TRIM
                            {
                                char dbg[40];
                                char csStr[8], lsStr[8];
                                dtostrf(commandedSpeed, 4, 2, csStr);
                                dtostrf(lastSentSpeed, 4, 2, lsStr);
                                snprintf(dbg, sizeof(dbg), "TRIM chg=%d cs=%s ls=%s", (int)speedChanged, csStr, lsStr);
                                sendStatusText(MAV_SEVERITY_DEBUG, dbg);
                            }
#endif
                            lastSpeedSendMs = now;
                            lastSentSpeed = commandedSpeed;
                        }
                    }
                }
            }
        }
    } else if (!waypointReset && rmbEverActive) {
        if (contourTrigger && haveDPT && haveVTGcog) {
            enterContourFollow();
        } else if (pixhawkMode == ROVER_MODE_HOLD) {
            // Guard: Intentionally do nothing to prevent LOITER reset while in HOLD mode
        } else {
            strncpy(lastLoiterResetDestId, prevDestWpId, sizeof(lastLoiterResetDestId) - 1);
            lastLoiterResetDestId[sizeof(lastLoiterResetDestId) - 1] = '\0';
            gotoLoiterReset(BC_LOITER_WP_RESET_RMB);
#if DEBUG_GUIDED || DEBUG_APPROACH || DEBUG_SURVEY
            sendStatusText_P(MAV_SEVERITY_WARNING, PSTR("RMB lost -> LOITER wp reset"));
#endif
        }
    }
}

static void handleLoiterWpResetState(uint32_t now, bool rmbFresh) {
    if (!loiterConfirmed) {
        if (pixhawkModeKnown && pixhawkMode == ROVER_MODE_LOITER) {
            loiterConfirmed = true;
            loiterStartMs = now;
        } else {
            requestModeChange(ROVER_MODE_LOITER, now);
        }
    } else if (rmbFresh) {
        if (requestModeChange(ROVER_MODE_GUIDED, now)) {
            navState = NavState::GUIDED;
            activeSpeedSlot = routeIsMultiWP ? SpeedSlot::FISHING : SpeedSlot::TRAVEL;
            speedNeedsResend = true;
        }
    } else if ((now - loiterStartMs) >= LOITER_RESET_MS) {
        if (requestModeChange(ROVER_MODE_GUIDED, now)) {
            navState = NavState::GUIDED;
            waypointReset = true;
            posTargetSentForDest = false;
            speedNeedsResend = true;
        }
    }
}

static void handleContourDepthLossRecovery(uint32_t now) {
    bool dptStaleNow = (now - lastDPTms) >= CONTOUR_DPT_LOSS_MS;

    if (dptStaleNow && !depthLost) {
        depthLost = true;
        contourHoldTargetSent = false;
        if (haveCurrentPos) {
            contourHoldLat_int = currentLat_int;
            contourHoldLon_int = currentLon_int;
        }
        EEPROM.update(EEPROM_ADDR_BREADCRUMB, BC_CONTOUR_DEPTH_LOST);
#if DEBUG_APPROACH || DEBUG_SURVEY || DEBUG_FOLLOW
        sendStatusText_P(MAV_SEVERITY_WARNING, PSTR("CONTOUR: depth lost -> hold"));
#endif
    } else if (!dptStaleNow && depthLost) {
        depthLost = false;
        if (contourPhase == ContourPhase::CIRCLE_SURVEY) {
            contourPhase = ContourPhase::CIRCLE_APPROACH;
        }
        if (contourPhase == ContourPhase::CIRCLE_APPROACH) {
            circleCrossFound = false;
            circleBestDev = 999.0f;
            circleAccumAngle = 0.0f;
            depthSlopeHead = 0;
            depthSlopeCount = 0;
            depthSlopeCumDist = 0.0f;
            surveyDptWasValid = false;
            sendSpeedCommand(travelSpeed);
            activeSpeedSlot = SpeedSlot::TRAVEL;
        }
        EEPROM.update(EEPROM_ADDR_BREADCRUMB, BC_CONTOUR_DEPTH_RECOVERED);
#if DEBUG_APPROACH || DEBUG_SURVEY || DEBUG_FOLLOW
        sendStatusText_P(MAV_SEVERITY_INFO, PSTR("CONTOUR: depth back ->  recircle"));
#endif
    }
}

static void handleContourApproachPhase(uint32_t now) {
    if (haveDPT && depth_m < CONTOUR_SAFETY_DEPTH_M) {
        gotoLoiterReset(BC_LOITER_WP_RESET_APPROACH);
        sendStatusText_P(MAV_SEVERITY_WARNING, PSTR("CONTOUR: shallow depth -> LOITER (approach)"));
    } else if (haveCurrentPos && activationPosValid) {
        float cosLat = activationCosLat;
        float dN = (currentLat_int - approachCenterLat_int) * METERS_PER_INT7_LAT;
        float dE = (currentLon_int - approachCenterLon_int) * METERS_PER_INT7_LAT * cosLat;
        float theta = atan2f(dE, dN) * 57.29577951f;
        if (theta < 0.0f) theta += 360.0f;

        if (!circleApproachSampled) {
            circleApproachPrevTheta = theta;
            circleApproachAccumAngle = 0.0f;
            circleApproachSampled = true;
        } else {
            float dTheta = theta - circleApproachPrevTheta;
            while (dTheta > 180.0f) dTheta -= 360.0f;
            while (dTheta < -180.0f) dTheta += 360.0f;
            circleApproachAccumAngle += dTheta;
            circleApproachPrevTheta = theta;
        }

        if (fabsf(circleApproachAccumAngle) >= 180.0f) {
            circlePrevLat_int = currentLat_int;
            circlePrevLon_int = currentLon_int;
            circleAccumAngle = 0.0f;
            circleCrossFound = false;
            circleBestDev = 999.0f;
            depthSlopeCount = 0;
            depthSlopeHead = 0;
            depthSlopeCumDist = 0.0f;
            circleLastDepth = haveDPT ? depth_m : contourTargetDepth_m;
            surveyDptWasValid = haveDPT;
            circleApproachSampled = false;  
            contourPhase = ContourPhase::CIRCLE_SURVEY;  
            activeSpeedSlot = SpeedSlot::TRAVEL;  
            EEPROM.update(EEPROM_ADDR_BREADCRUMB, BC_SURVEY_ENTER);
#if DEBUG_GUIDED || DEBUG_APPROACH || DEBUG_SURVEY
            sendStatusText_P(MAV_SEVERITY_INFO, PSTR("CONTOUR: survey phase entered"));
#endif
        } else {
            float bearingTangentDeg = theta + 90.0f;
            if (bearingTangentDeg < 0.0f) bearingTangentDeg += 360.0f;
            if (bearingTangentDeg >= 360.0f) bearingTangentDeg -= 360.0f;
            sendVelocityTarget(bearingTangentDeg, activeSpeedSlot == SpeedSlot::FISHING ? fishingSpeed : travelSpeed);
        }
    } else {
        sendVelocityTarget(contourHeadingDeg, activeSpeedSlot == SpeedSlot::FISHING ? fishingSpeed : travelSpeed);
    }
}

static void handleContourSurveyPhase(uint32_t now) {
    if (haveDPT && depth_m < CONTOUR_SAFETY_DEPTH_M) {
        gotoLoiterReset(BC_LOITER_WP_RESET_SURVEY);
        sendStatusText_P(MAV_SEVERITY_WARNING, PSTR("CONTOUR: shallow depth -> LOITER (survey)"));
    } else if (haveCurrentPos && activationPosValid) {
        bool posIsNew = (currentLat_int != circlePrevLat_int) || (currentLon_int != circlePrevLon_int);
        float cosLat = activationCosLat;
        float dN = (currentLat_int - activationLat_int) * METERS_PER_INT7_LAT;
        float dE = (currentLon_int - activationLon_int) * METERS_PER_INT7_LAT * cosLat;
        float dist = sqrtf(dN * dN + dE * dE);
        if (dist < 0.01f) dist = 0.01f;

        float theta = atan2f(dE, dN) * 180.0f / (float)M_PI;
        if (theta < 0.0f) theta += 360.0f;

        float dTheta = posIsNew ? (theta - circlePrevTheta) : 0.0f;
        while (dTheta > 180.0f) dTheta -= 360.0f;
        while (dTheta < -180.0f) dTheta += 360.0f;

        circleAccumAngle += dTheta;
        circlePrevTheta = theta;

        if (haveDPT) {
            if (!surveyDptWasValid) {
                circleLastDepth = depth_m;
                depthSlopeHead = 0;
                depthSlopeCount = 0;
                depthSlopeCumDist = 0.0f;
            }
            surveyDptWasValid = true;
        } else {
            surveyDptWasValid = false;
        }

        if (haveDPT && posIsNew) {
            float arcStep = fabsf(dTheta) * (float)M_PI / 180.0f * CONTOUR_CIRCLE_RADIUS_M;
            depthSlopeCumDist += arcStep;
            depthSlopeBufDepth[depthSlopeHead] = (int16_t)(depth_m * 100.0f);
            depthSlopeBufDist[depthSlopeHead] = (int16_t)(depthSlopeCumDist * 100.0f);
            depthSlopeHead = (depthSlopeHead + 1) % DEPTH_SLOPE_SAMPLES;
            if (depthSlopeCount < DEPTH_SLOPE_SAMPLES) depthSlopeCount++;

            float prevErr = circleLastDepth - contourTargetDepth_m;
            float currErr = depth_m - contourTargetDepth_m;
            bool crossed = (prevErr > 0.0f && currErr <= 0.0f) || (prevErr < 0.0f && currErr >= 0.0f) || (prevErr == 0.0f && currErr != 0.0f);

            if (crossed) {
                bool deepSide = false, deepSideValid = false;
                if (depthSlopeCount >= 2) {
                    uint8_t oldIdx = (depthSlopeHead + DEPTH_SLOPE_SAMPLES - depthSlopeCount) % DEPTH_SLOPE_SAMPLES;
                    float distSpan = depthSlopeCumDist - (depthSlopeBufDist[oldIdx] * 0.01f);
                    if (distSpan >= DEPTH_SLOPE_MIN_DIST_M) {
                        float slope = (depth_m - (depthSlopeBufDepth[oldIdx] * 0.01f)) / distSpan;
                        deepSideValid = fabsf(slope) >= DEPTH_SLOPE_CONF_THR;
                        deepSide = (slope > 0.0f);
                    }
                }
#if DEBUG_CONTOUR_TUNING
                {
                    char dbg[48];
                    char pe[8], ce[8];
                    dtostrf(prevErr, 6, 2, pe);
                    dtostrf(currErr, 6, 2, ce);
                    snprintf(dbg, sizeof(dbg), "XING pe=%s ce=%s slV=%d cnt=%d", pe, ce, (int)deepSideValid, (int)depthSlopeCount);
                    sendStatusText(MAV_SEVERITY_DEBUG, dbg);
                }
#endif
                if (deepSideValid) {
                    float dirA = theta;
                    float dirB = theta + 180.0f;
                    if (dirB >= 360.0f) dirB -= 360.0f;

                    float devA = dirA - activationHeadingDeg;
                    while (devA > 180.0f) devA -= 360.0f;
                    while (devA < -180.0f) devA += 360.0f;
                    devA = fabsf(devA);

                    float devB = dirB - activationHeadingDeg;
                    while (devB > 180.0f) devB -= 360.0f;
                    while (devB < -180.0f) devB += 360.0f;
                    devB = fabsf(devB);

                    float crossFrac = prevErr / (prevErr - currErr);
                    if (crossFrac < 0.0f) crossFrac = 0.0f;
                    if (crossFrac > 1.0f) crossFrac = 1.0f;

                    int32_t crossLat_int = circlePrevLat_int + (int32_t)((currentLat_int - circlePrevLat_int) * crossFrac);
                    int32_t crossLon_int = circlePrevLon_int + (int32_t)((currentLon_int - circlePrevLon_int) * crossFrac);
                    bool acceptedA = false, acceptedB = false;

                    if (devA <= devB) {
                        if (devA < circleBestDev) {
                            circleBestDev = devA;
                            circleBestHeading = dirA;
                            circleBestDeepRight = deepSide;
                            circleBestLat_int = crossLat_int;
                            circleBestLon_int = crossLon_int;
                            circleCrossFound = true;
                            acceptedA = true;
                        }
                    } else {
                        if (devB < circleBestDev) {
                            circleBestDev = devB;
                            circleBestHeading = dirB;
                            circleBestDeepRight = !deepSide;
                            circleBestLat_int = crossLat_int;
                            circleBestLon_int = crossLon_int;
                            circleCrossFound = true;
                            acceptedB = true;
                        }
                    }
#if DEBUG_CONTOUR_TUNING
                    {
                        char dbg[48];
                        char bd[8];
                        dtostrf(circleBestDev, 6, 2, bd);
                        snprintf(dbg, sizeof(dbg), "XING acc=%d dev=%s side=%d", (int)(acceptedA || acceptedB), bd, (int)circleBestDeepRight);
                        sendStatusText(MAV_SEVERITY_DEBUG, dbg);
                    }
#endif
                }
            }
            circleLastDepth = depth_m;
        }

        circlePrevLat_int = currentLat_int;
        circlePrevLon_int = currentLon_int;

        float radErr = dist - CONTOUR_CIRCLE_RADIUS_M;
        float tangHead = theta + 90.0f;
        if (tangHead >= 360.0f) tangHead -= 360.0f;

        float radCorr = radErr * CIRCLE_RADIUS_GAIN;
        if (radCorr > CIRCLE_RADIUS_MAX_CORR_DEG) radCorr = CIRCLE_RADIUS_MAX_CORR_DEG;
        if (radCorr < -CIRCLE_RADIUS_MAX_CORR_DEG) radCorr = -CIRCLE_RADIUS_MAX_CORR_DEG;

        float corrHead = tangHead + radCorr;
        while (corrHead < 0.0f) corrHead += 360.0f;
        while (corrHead >= 360.0f) corrHead -= 360.0f;
        sendVelocityTarget(corrHead, activeSpeedSlot == SpeedSlot::FISHING ? fishingSpeed : travelSpeed);

        if (circleAccumAngle >= 360.0f || circleAccumAngle <= -360.0f) {
            if (circleCrossFound) {
                contourBaseHeadingDeg = circleBestHeading;
                contourHeadingDeg = circleBestHeading;
                contourKpSigned = circleBestDeepRight ? 1.0f : -1.0f;
                followPrevError_m = contourTargetDepth_m - (haveDPT ? depth_m : contourTargetDepth_m);
                contourTrendRefSet = false;

                contourLastGoodLat_int = haveCurrentPos ? currentLat_int : circleBestLat_int;
                contourLastGoodLon_int = haveCurrentPos ? currentLon_int : circleBestLon_int;
                contourLastGoodHeadingDeg = circleBestHeading;
                contourLastGoodMs = millis();
                contourLastGoodValid = true;

                contourPhase = ContourPhase::FOLLOW;
                EEPROM.update(EEPROM_ADDR_BREADCRUMB, BC_FOLLOW_ENTER);
                sendSpeedCommand(fishingSpeed);
                activeSpeedSlot = SpeedSlot::FISHING;
#if DEBUG_SURVEY || DEBUG_FOLLOW
                sendStatusText_P(MAV_SEVERITY_INFO, PSTR("CONTOUR: survey done -> follow"));
#endif
            } else {
#if DEBUG_FREE_RAM
                checkFreeRam('L');
#endif
#if DEBUG_SURVEY
                sendStatusText_P(MAV_SEVERITY_WARNING, PSTR("CONTOUR: lap no cross -> retry"));
#endif
                circleAccumAngle = 0.0f;
                circleCrossFound = false;
                circleBestDev = 999.0f;
                depthSlopeCount = 0;
                depthSlopeHead = 0;
                depthSlopeCumDist = 0.0f;
                circleLastDepth = haveDPT ? depth_m : contourTargetDepth_m;
                surveyDptWasValid = haveDPT;
            }
        }
    } else {  
        sendVelocityTarget(contourHeadingDeg, activeSpeedSlot == SpeedSlot::FISHING ? fishingSpeed : travelSpeed);  
    }  
}  
  
static void handleContourFollowPhase(uint32_t now) {
    if (haveDPT && depth_m < CONTOUR_SAFETY_DEPTH_M) {
        gotoLoiterReset(BC_LOITER_WP_RESET_FOLLOW);
        sendStatusText_P(MAV_SEVERITY_WARNING, PSTR("CONTOUR: shallow depth -> LOITER (follow)"));
    } else {
        EEPROM.update(EEPROM_ADDR_BREADCRUMB, BC_FOLLOW_LOOP_TOP);
        if (haveDPT) {
            float error = contourTargetDepth_m - depth_m;
            if (fabsf(error) - fabsf(followPrevError_m) > CONTOUR_FLIP_ABORT_M && (now - lastSideFlipMs) >= CONTOUR_FLIP_GATE_MS) {
                contourKpSigned = -contourKpSigned;
                contourBaseHeadingDeg = contourHeadingDeg - lastContourOffsetDeg;
                lastSideFlipMs = now;
#if DEBUG_FOLLOW
                sendStatusText_P(MAV_SEVERITY_WARNING, PSTR("CONTOUR: side flip"));
#endif
            }
            followPrevError_m = error;

            float offset = contourKpSigned * CONTOUR_GAIN_DEG_PER_M * error;
            if (offset > CONTOUR_MAX_OFFSET_DEG) offset = CONTOUR_MAX_OFFSET_DEG;
            if (offset < -CONTOUR_MAX_OFFSET_DEG) offset = -CONTOUR_MAX_OFFSET_DEG;

            lastContourOffsetDeg = offset;
            contourHeadingDeg = contourBaseHeadingDeg + offset;

            while (contourHeadingDeg < 0.0f) contourHeadingDeg += 360.0f;
            while (contourHeadingDeg >= 360.0f) contourHeadingDeg -= 360.0f;

            EEPROM.update(EEPROM_ADDR_BREADCRUMB, BC_FOLLOW_HEADING_DONE);
            bool nowInGood = fabsf(error) <= CONTOUR_CORRECT_DEPTH_M;

            if (nowInGood && haveCurrentPos) {
                contourLastGoodLat_int = currentLat_int;
                contourLastGoodLon_int = currentLon_int;
                contourLastGoodHeadingDeg = contourHeadingDeg;
                contourLastGoodValid = true;
                contourLastGoodMs = now;
            }

            if (haveCurrentPos && contourLastGoodValid) {
                float dNs = (currentLat_int - contourLastGoodLat_int) * METERS_PER_INT7_LAT;
                float dEs = (currentLon_int - contourLastGoodLon_int) * METERS_PER_INT7_LAT * cosf(currentLat_deg * (float)M_PI / 180.0f);
                float satDist = sqrtf(dNs * dNs + dEs * dEs);

                if ((now - contourLastGoodMs) >= CONTOUR_LOST_TIMEOUT_MS || satDist >= CONTOUR_LOST_DIST_M) {
                    EEPROM.update(EEPROM_ADDR_BREADCRUMB, BC_FOLLOW_REENTER);
                    reenterContourFromLastGood();
                    sendVelocityTarget(contourHeadingDeg, travelSpeed);
                    return;
                }
            }
        }
        if (haveCurrentPos && contourTrendRefSet) {
            float dN2 = (currentLat_int - contourTrendRefLat_int) * METERS_PER_INT7_LAT;
            float dE2 = (currentLon_int - contourTrendRefLon_int) * METERS_PER_INT7_LAT * contourTrendRefCosLat;

            if (dN2 * dN2 + dE2 * dE2 >= CONTOUR_TREND_DIST_M * CONTOUR_TREND_DIST_M) {
                float ddepth = depth_m - contourTrendRefDepth;
                if (fabsf(ddepth) > TREND_DEPTH_MIN_M) {
                    float nudge = contourKpSigned * (-ddepth * TREND_NUDGE_GAIN);
                    if (nudge > TREND_NUDGE_MAX_DEG) nudge = TREND_NUDGE_MAX_DEG;
                    if (nudge < -TREND_NUDGE_MAX_DEG) nudge = -TREND_NUDGE_MAX_DEG;

                    contourBaseHeadingDeg += nudge;
                    while (contourBaseHeadingDeg < 0.0f) contourBaseHeadingDeg += 360.0f;
                    while (contourBaseHeadingDeg >= 360.0f) contourBaseHeadingDeg -= 360.0f;
                }
                contourTrendRefLat_int = currentLat_int;
                contourTrendRefLon_int = currentLon_int;
                contourTrendRefDepth = depth_m;
                contourTrendRefCosLat = cosf(currentLat_deg * (float)M_PI / 180.0f);
            }
        } else if (haveCurrentPos && !contourTrendRefSet) {
            contourTrendRefLat_int = currentLat_int;
            contourTrendRefLon_int = currentLon_int;
            contourTrendRefDepth = depth_m;
            contourTrendRefCosLat = cosf(currentLat_deg * (float)M_PI / 180.0f);
            contourTrendRefSet = true;
        }
        EEPROM.update(EEPROM_ADDR_BREADCRUMB, BC_FOLLOW_SEND_DONE);
        sendVelocityTarget(contourHeadingDeg, fishingSpeed);
    }
}

static void handleContourFollowState(uint32_t now, bool rmbFresh) {
    handleContourDepthLossRecovery(now);

    if (rmbFresh && !rmbArrived) {
        if (requestModeChange(ROVER_MODE_GUIDED, now)) {
#if DEBUG_GUIDED || DEBUG_APPROACH || DEBUG_SURVEY || DEBUG_FOLLOW
            sendStatusText_P(MAV_SEVERITY_INFO, PSTR("CONTOUR: new dest -> guided"));
#endif
            navState = NavState::GUIDED;
            waypointReset = false;
            posTargetSentForDest = false;
            contourTrigger = false;
            rmbArrived = false;
            speedNeedsResend = true;
        }
    } else if (pixhawkMode == ROVER_MODE_HOLD) {
        // Guard: Maintain HOLD mode without falling through to GUIDED or depthLost actions
    } else if (pixhawkMode != ROVER_MODE_GUIDED) {
        sendStatusText_P(MAV_SEVERITY_WARNING, PSTR("CONTOUR: mode changed ext -> GUIDED"));
        navState = NavState::GUIDED;
        waypointReset = true;
        routeIsMultiWP = false;
        prevDestWpId[0] = '\0';
        posTargetSentForDest = false;
        activeSpeedSlot = SpeedSlot::NONE;
        contourTrigger = false;
        rmbArrived = false;
        speedNeedsResend = true;
    } else if (depthLost) {
        if (haveCurrentPos) {
            if (!contourHoldTargetSent) {
                sendPositionTarget(contourHoldLat_int, contourHoldLon_int);
                contourHoldTargetSent = true;
            }
        } else {
            sendVelocityTarget(contourHeadingDeg, 0.0f);
        }
    } else if (contourPhase == ContourPhase::CIRCLE_APPROACH) {
        handleContourApproachPhase(now);
    } else if (contourPhase == ContourPhase::CIRCLE_SURVEY) {
        handleContourSurveyPhase(now);
    } else if (contourPhase == ContourPhase::FOLLOW) {
        handleContourFollowPhase(now);
    }
}

// ---- Main Loop ----
void loop() {
    if (!updateWatchdogAndDiagnostics()) return;

    readNMEA();
    readMAVLink();

    if (speedDirty && (millis() - speedChangedMs) >= EEPROM_WRITE_DELAY_MS) {
        EEPROM.put(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
        EEPROM.put(EEPROM_ADDR_TRAVEL_SPEED, travelSpeed);
        EEPROM.put(EEPROM_ADDR_FISHING_SPEED, fishingSpeed);
        speedDirty = false;
    }

    unsigned long now = millis();

#if DEBUG_FREE_RAM
    static unsigned long lastRamCheckMs = 0;
    if (now - lastRamCheckMs >= 200) {
        lastRamCheckMs = now;
        checkFreeRam('T');
    }
#endif

    if (now - lastHeartbeat >= HB_INTERVAL_MS) {
        sendHeartbeat();
        lastHeartbeat = now;
#if DEBUG_LINK
        char st[48];
        snprintf(st, sizeof(st), "PM=%u PK=%u SY=%u HB=%u AF=%d",
                 (unsigned)pixhawkMode, (unsigned)pixhawkModeKnown,
                 (unsigned)lastHbSysid, (unsigned)hbRxCount,
                 (int)Serial.availableForWrite());
        sendStatusText(MAV_SEVERITY_DEBUG, st);
#endif
    }

    handleVescMonitoring(now);
    handleModeConfirmFailsafe(now);

    if (now - lastSend >= SEND_INTERVAL_MS) {
        lastSend = now;
        if (haveVTG && (now - lastVTGms) >= VTG_TIMEOUT_MS) {
            haveVTG = false;
            haveVTGcog = false;
        }
        if (haveDPT && (now - lastDPTms) >= DPT_TIMEOUT_MS) {
            haveDPT = false;
#if ENABLE_DPT_GLITCH_FILTER
            dptGlitchCount = 0;
#endif
            circleLastDepth = contourTargetDepth_m;
            contourTrendRefSet = false;
        }

        bool rmbFresh = rmbActive && ((now - lastRMBms) < RMB_TIMEOUT_MS);

        if (navState == NavState::GUIDED) {
            handleGuidedState(now, rmbFresh);
        } else if (navState == NavState::LOITER_WP_RESET) {
            handleLoiterWpResetState(now, rmbFresh);
        } else if (navState == NavState::CONTOUR_FOLLOW) {
            handleContourFollowState(now, rmbFresh);
        }
    }
}
