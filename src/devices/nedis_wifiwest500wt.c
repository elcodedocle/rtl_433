/** @file
    Nedis WIFIWEST500WT weather station sensor.

    Copyright (C) 2026 elcodedocle

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
/**
    Nedis WIFIWEST500WT (Tuya compatible) weather station sensor decoder.

    17-byte Datagram format (~60s period):

    FH HN IIIIII BTTT hh AAA wgGG DRRR 00 00 CKXK

    - FH: Fixed header byte (0xF0). A change in this byte makes the stock
          receiver drop the packet. The stock transmitter always sends this
          value (no pairing/negotiation observed, constant across restarts
          and battery changes), so it is treated as a fixed header.
    - HN: N = 3-bit rolling counter (0-7). A change in the H (0x5) nibble
          makes the stock receiver drop the packet (same as FH).
    - IIIIII: 24-bit device ID (unique per sensor unit)
    - B: Battery charge indicator nibble (high nibble of b[5]).
         The stock receiver low battery icon only lights up when the least
         significant bit of the nibble is 0, so battery_ok = bit 4 of b[5].
         The stock transmitter sends 0x7 (ok) / 0x0 (low); the meaning of the
         three upper bits is unknown.
    - TTT: Temperature (12-bit, bits 0-3 of b[5] + all of b[6]).
           Format is (temp_c * 10). Two's complement for < 0.
    - hh: Humidity (%RH)
    - AAA: 12-bit average wind speed, 0.1 m/s per count
           (b[8] << 4 | b[9] >> 4)
    - wgGG: 12-bit wind gust, 0.1 m/s per count
            (b[9] & 0x0F) << 8 | b[10]
            w = the 2 most significant bits. In the STOCK receiver a non-zero
            w also multiplies the displayed AVERAGE wind speed by 10, which
            looks like a stock decoder bug (unconfirmed: the stock transmitter
            never sets these bits). This decoder reports the plain 12-bit
            value and logs an anomaly note when w != 0.
            g = the next 2 bits, GG = gust LSB.
    - DRRR: D = Gust wind direction (high nibble of b[11]), 16-point compass,
          22.5 deg per step.
            RRR = Rain counter (12-bit, low nibble of b[11] is the high
            nibble of the count, b[12] is the low byte), 0.35 mm per tip.
    - 00 00: Padding/reserved (b[13], b[14]; does not affect the stock
             receiver, always 0 on the stock transmitter).
    - CK: Checksum byte
    - XK: Ones-complement of CK

    NOTE: there is no average-wind-direction field in this format. The only
    direction transmitted is the gust/instantaneous one in b[11].

    Original Model Radio Spec:
    Frequency   : 868.4 MHz FSK (868.375 MHz measured on the stock tx)
    Modulation  : FSK-PWM over FSK carrier
                  Each bit is one 7-chip symbol at 122 us/chip (854 us/symbol):
                    Bit 1 = F1 long (610 us) + F2 short (244 us)
                    Bit 0 = F1 short (244 us) + F2 long (610 us)
                  Preamble: ~66 equal-duty symbols (~850 us each) before each
                  packet.
    Bit rate    : ~1170 baud
    Packet      : 17 bytes, repeated 8x per burst.
    Burst cycle : Device sends one burst per transmission cycle. All sensor data
                  is included in every packet (no channel multiplexing).

    -- Integrity check --

    Each burst repeats the same 17-byte packet up to 8x (~120-130ms apart),
    incrementing the 3-bit rolling counter (byte[1] low nibble) by one on
    every repeat: 8 packets in one burst, counter running 0..7. All other
    fields are constant across the burst.

    A single flipped chip during PWM-decode passes the F1-ones-count slicer
    silently (it just yields the "wrong side" bit), so every packet is
    validated on its own: the fixed header must match (b[0] == 0xF0, b[1]
    high nibble == 0x5) and two checks must pass:

    - CK/XK: b[15]+b[16]==0xFF. This is self-consistent by construction and
      does NOT depend on b[0..14].
    - For b[0..14] the CHECKSUM is:

            CK = 0x2D XOR M(b[0]) XOR M(b[1]) XOR ... XOR M(b[14])

        Where M(x) is a single fixed linear (GF(2)) map applied
        independently to every payload byte (no CRC-style chained state
        between bytes). M(x) is the carry-less product of x and 0x83
        (0x83 << i) for every bit i set in x. M is fully determined by 8
        basis values:

            M(1)=0x83  M(2)=0x06  M(4)=0x0c  M(8)=0x18
            M(16)=0x30 M(32)=0x60 M(64)=0xc0 M(128)=0x80

    A packet failing any of these is dropped; the remaining repeats of the
    burst are decoded independently.

    -- Packet location --

    The fixed header doubles as packet sync: the decoder tries every chip
    offset of the row and only treats a window as a packet candidate if it
    starts with the fixed header. Candidates are then validated with the
    checks above. A valid packet is skipped as a whole, a failed candidate
    is not, so a chance header match in noise cannot hide the real packet.

*/

#include "decoder.h"

#define NEDIS_PACKET_BYTES  17
#define NEDIS_CHIPS_PER_BIT 7
#define NEDIS_PACKET_CHIPS  (NEDIS_PACKET_BYTES * 8 * NEDIS_CHIPS_PER_BIT) // 952

#define NEDIS_HEADER_B0     0xF0 // fixed header, b[0]
#define NEDIS_HEADER_B1H    0x5  // fixed header, high nibble of b[1]

// Wind direction lookup: 16-point compass rose
static char const *const nedis_wind_dir_str[] = {
        "N",
        "NNE",
        "NE",
        "ENE",
        "E",
        "ESE",
        "SE",
        "SSE",
        "S",
        "SSW",
        "SW",
        "WSW",
        "W",
        "WNW",
        "NW",
        "NNW",
};

/**
 * GF(2) linear map used by the payload checksum: M(x) is the low byte of
 * the carry-less (XOR, no reduction) product of x and 0x83, i.e. XOR
 * together (0x83 << i) for every bit i set in x.
 */
static uint8_t nedis_M(uint8_t x)
{
    uint16_t product = 0;
    for (int i = 0; i < 8; i++) {
        if (x & (1u << i)) {
            product ^= (0x83u << i);
        }
    }
    return (uint8_t)product;
}

/**
 * Payload checksum: CK = 0x2D XOR M(b[0]) XOR M(b[1]) XOR ... XOR M(b[14]).
 */
static uint8_t nedis_checksum(uint8_t const *b)
{
    uint8_t ck = 0x2D;
    for (int i = 0; i < 15; i++) {
        ck ^= nedis_M(b[i]);
    }
    return ck;
}

/**
 * Fixed header: b[0] == 0xF0 and b[1] high nibble == 0x5.
 */
static int nedis_header_ok(uint8_t const *b)
{
    return b[0] == NEDIS_HEADER_B0 && (b[1] >> 4) == NEDIS_HEADER_B1H;
}

/**
 * Validate packet integrity.
 * - b[0] and b[1] high nibble must be the fixed header.
 * - CK (b[15]) and XK (b[16]) must be ones-complements of each other.
 * - CK must match the checksum over b[0..14].
 */
static int nedis_check(uint8_t const *b)
{
    if (!nedis_header_ok(b)) {
        return 0;
    }
    if (((b[15] + b[16]) & 0xFF) != 0xFF) {
        return 0;
    }
    return b[15] == nedis_checksum(b);
}

/**
 * Field extraction and output.
 *
 * @return 1 if data was successfully output, 0 on data_make() failure.
 */
static int nedis_output(r_device *decoder, uint8_t const *b)
{
    uint8_t counter    = b[1] & 0x07;
    uint32_t device_id = (b[2] << 16) | (b[3] << 8) | b[4];

    // Battery nibble: only bit 0 of the nibble drives the stock low-battery
    // icon (0 = low). The remaining 3 bits are unknown.
    int battery_ok = (b[5] >> 4) & 0x01;

    int temp_raw = ((b[5] & 0x0F) << 8) | b[6];
    if (temp_raw & 0x0800) {
        temp_raw -= 0x1000;
    }
    float temp_c = temp_raw * 0.1f;

    int humidity = b[7];

    // Average wind: 12-bit, 0.1 m/s per count.
    int wind_raw       = (b[8] << 4) | ((b[9] >> 4) & 0x0F);
    float wind_avg_ms  = wind_raw * 0.1f;
    float wind_avg_kmh = wind_avg_ms * 3.6f;

    // Gust: 12-bit, 0.1 m/s per count.
    int gust_raw   = ((b[9] & 0x0F) << 8) | b[10];
    float gust_ms  = gust_raw * 0.1f;
    float gust_kmh = gust_ms * 3.6f;

    // Only the gust/instantaneous direction is transmitted.
    int dir_gust_idx   = (b[11] >> 4) & 0x0F;
    float wind_dir_deg = dir_gust_idx * 22.5f;

    // 12-bit rain counter: high nibble is the low nibble of b[11] (shared
    // with the gust direction byte), low byte is b[12].
    int rain_raw  = ((b[11] & 0x0F) << 8) | b[12];
    float rain_mm = rain_raw * 0.35f;

    // Bits that are always 0 on the stock transmitter. Non-zero means either
    // a decode error or a device/firmware variant worth investigating.
    // Note: gust bits 11..10 ("w") also scale the AVERAGE wind by 10 on the
    // stock receiver, which appears to be a stock decoder bug.
    if ((gust_raw & 0x0C00) || b[13] || b[14]) {
        decoder_logf(decoder, 1, __func__,
                "unexpected non-zero reserved bits: gust_hi=%d b13=%02x b14=%02x",
                (gust_raw >> 10) & 0x03, b[13], b[14]);
    }

    data_t *data = data_make(
            "model", "", DATA_STRING, "Nedis-WIFIWEST500WT",
            "id", "Device ID", DATA_FORMAT, "%06X", DATA_INT, device_id,
            "counter", "Counter", DATA_INT, counter,
            "battery_ok", "Battery", DATA_INT, battery_ok,
            "temperature_C", "Temperature", DATA_FORMAT, "%.1f C", DATA_DOUBLE, (double)temp_c,
            "humidity", "Humidity", DATA_FORMAT, "%u %%", DATA_INT, humidity,
            "wind_dir_deg", "Wind direction", DATA_FORMAT, "%.1f deg", DATA_DOUBLE, (double)wind_dir_deg,
            "wind_dir_str", "Wind direction", DATA_STRING, nedis_wind_dir_str[dir_gust_idx],
            "wind_avg_m_s", "Wind speed (avg)", DATA_FORMAT, "%.1f m/s", DATA_DOUBLE, (double)wind_avg_ms,
            "wind_avg_km_h", "Wind speed (avg)", DATA_FORMAT, "%.1f km/h", DATA_DOUBLE, (double)wind_avg_kmh,
            "wind_max_m_s", "Wind gust", DATA_FORMAT, "%.1f m/s", DATA_DOUBLE, (double)gust_ms,
            "wind_max_km_h", "Wind gust", DATA_FORMAT, "%.1f km/h", DATA_DOUBLE, (double)gust_kmh,
            "rain_mm", "Rain total", DATA_FORMAT, "%.1f mm", DATA_DOUBLE, (double)rain_mm,
            "mic", "Integrity", DATA_STRING, "CHECKSUM",
            NULL);

    if (!data) {
        return 0;
    }
    decoder_output_data(decoder, data);
    return 1;
}

static int nedis_wifiwest500wt_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    // Reject noise and partial frames shorter than one packet
    if (bitbuffer->bits_per_row[0] < NEDIS_PACKET_CHIPS) {
        return DECODE_ABORT_LENGTH;
    }
    int row = bitbuffer_find_repeated_row(bitbuffer, 2, NEDIS_PACKET_CHIPS);
    if (row < 0) {
        // Fall back: scan all rows for a decodable one.
        for (int r = 0; r < (int)bitbuffer->num_rows; r++) {
            if (bitbuffer->bits_per_row[r] >= NEDIS_PACKET_CHIPS) {
                row = r;
                break;
            }
        }
    }
    if (row < 0) {
        return DECODE_ABORT_LENGTH;
    }

    uint8_t const *chips = bitbuffer->bb[row];
    int nchips           = bitbuffer->bits_per_row[row];

    // PWM-decode chip stream to bytes.
    // Try every chip offset as a packet start and keep the ones that yield
    // the fixed header.
#define CHIP_BIT(pos) ((chips[(pos) >> 3] >> (7 - ((pos) & 7))) & 1)

    uint8_t b[NEDIS_PACKET_BYTES];
    int found  = 0;
    int events = 0;

    for (int chip = 0; chip + NEDIS_PACKET_CHIPS <= nchips; chip++) {
        memset(b, 0, sizeof(b));

        for (int sym = 0; sym < NEDIS_PACKET_BYTES * 8; sym++) {
            int base = chip + sym * NEDIS_CHIPS_PER_BIT;

            // Count F1-high chips in this 7-chip window:
            // ~5 ones = long F1 = bit 1, ~2 ones = short F1 = bit 0
            int ones = 0;
            for (int k = 0; k < NEDIS_CHIPS_PER_BIT; k++) {
                ones += CHIP_BIT(base + k);
            }
            if (ones >= 4) {
                b[sym >> 3] |= 1 << (7 - (sym & 7));
            }
        }

        // Require the fixed header before treating this window as packet
        // candidate (nedis_check() does the full validation)
        if (!nedis_header_ok(b)) {
            continue;
        }

        found = 1;

        decoder_logf(decoder, 1, __func__,
                "candidate pkt at chip %d: "
                "%02x %02x %02x %02x %02x %02x %02x %02x "
                "%02x %02x %02x %02x %02x %02x %02x %02x %02x",
                chip,
                b[0], b[1], b[2], b[3],
                b[4], b[5], b[6], b[7],
                b[8], b[9], b[10], b[11],
                b[12], b[13], b[14], b[15],
                b[16]);

        if (nedis_check(b) && nedis_output(decoder, b)) {
            events++;
            // Skip the packet and continue (multiple packets in bitbuffer).
            // Only done for valid packets: a chance header match in noise
            // must not swallow the real packet that follows it.
            chip += NEDIS_PACKET_CHIPS;
        }
    }

#undef CHIP_BIT

    if (events) {
        return events;
    }
    return found ? DECODE_FAIL_MIC : DECODE_FAIL_SANITY;
}

static char const *nedis_wifiwest500wt_output_fields[] = {
        "model",
        "id",
        "counter",
        "battery_ok",
        "temperature_C",
        "humidity",
        "wind_dir_deg",
        "wind_dir_str",
        "wind_avg_m_s",
        "wind_avg_km_h",
        "wind_max_m_s",
        "wind_max_km_h",
        "rain_mm",
        "mic",
        NULL,
};

r_device const nedis_wifiwest500wt = {
        .name        = "Nedis WIFIWEST500WT",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 122,  // chip width in us
        .long_width  = 122,  // same (NRZ chip stream; PWM decoded in software above)
        .reset_limit = 8000, // inter-packet gap > 8 ms
        .tolerance   = 30,   // Prevents row resets during long chip runs
        .decode_fn   = &nedis_wifiwest500wt_decode,
        .disabled    = 0,
        .fields      = nedis_wifiwest500wt_output_fields,
};