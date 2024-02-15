//
// Copyright (c) dushin.net
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#include <stdlib.h>

#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <rom/ets_sys.h>

#include <context.h>
#include <defaultatoms.h>
#include <esp32_sys.h>
#include <interop.h>
#include <nifs.h>
#include <port.h>
#include <term.h>

#include "atomvm_dht.h"

//#define ENABLE_TRACE
#include "trace.h"

#define TAG "atomvm_dht"

// References
// https://www.electronicwings.com/sensors-modules/dht11
// https://components101.com/sites/default/files/component_datasheet/DHT11-Temperature-Sensor.pdf
// https://cdn-shop.adafruit.com/datasheets/Digital+humidity+and+temperature+sensor+AM2302.pdf

//
// Handshake Send:
//
// ---+                      +---  1 (high)
//     \                    /      |
//      \                  /       |
//       \                /        |
//        +--------------+         0 (low)
//        |              |
//        |<--- 18ms --->|
//
//
//
// Handshake Receive:
//
//
//                            |<---- 80us ---->|
//                            |                |
// ---+                       +----------------+            1 (high)
//     \                     /                  \           |
//      \                   /                    \          |
//       \                 /                      \         |
//        +---------------+                        +------  0 (low)
//        |               |
//        | <--- 80us --->|
//
//
// for each bit (of 40):
//
// Data Receive 0:
//
//                            |<- 28us ->|
//                            |          |
// ---+                       +----------+            1 (high)
//     \                     /            \           |
//      \                   /              \          |
//       \                 /                \         |
//        +---------------+                  +------  0 (low)
//        |               |
//        | <--- 54us --->|
//
//
// Data Receive 1:
//
//                            |<---- 70us ---->|
//                            |                |
// ---+                       +----------------+            1 (high)
//     \                     /                  \           |
//      \                   /                    \          |
//       \                 /                      \         |
//        +---------------+                        +------  0 (low)
//        |               |
//        | <--- 54us --->|
//

#define HANDSHAKE_SEND_LOW_US (18000)
#define HANDSHAKE_RECV_LOW_US (80)
#define HANDSHAKE_RECV_HIGH_US (80)
#define DATA_RECV_LOW_US (54)
#define DATA_RECV_HIGH_ONE_US (70)
#define DATA_RECV_HIGH_ZERO_US (28)

#define MAX_WAIT_US (1000)
#define TIMEOUT (-1)
#define PROTOCOL_ERROR (-1)

#define DHT_OK 0
#define DHT_CHECKSUM_ERROR -1
#define DHT_TIMEOUT_ERROR -2
float humidity = 0.;
float temperature = 0.;
unsigned long lastReadTime;

static const char *const dht_bad_read = "\x8"
                                        "bad_read";
//                                                      123456789ABCDEF01
// this banging code https://github.com/beegee-tokyo/DHTesp/blob/master/DHTesp.cpp works on the wokwi sim.
// for reasons, not to be investigated
//
static int read_into(avm_int_t pin, uint8_t *buf)
{
    int64_t startTime = esp_timer_get_time();
    if (lastReadTime && (unsigned long) (startTime - lastReadTime) < 2000000) {
        printf("Too fast\n");
        return DHT_CHECKSUM_ERROR;
    }
    lastReadTime = startTime;

    uint16_t rawHumidity = 0;
    uint16_t rawTemperature = 0;
    uint16_t data = 0;

    // == Send start signal to DHT sensor ===========

    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    // pull down for 3 ms for a smooth and nice wake up
    gpio_set_level(pin, 0);
    esp_rom_delay_us(2000);
    // pull up for 25 us for a gentile asking for data
    gpio_set_direction(pin, GPIO_MODE_INPUT); // change to input mode
    gpio_set_level(pin, 1);
    esp_rom_delay_us(25);

    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    // go critical as timings will be very sensitive
    portENTER_CRITICAL(&mux);

    for (int8_t i = -3; i < 2 * 40; i++) {
        uint8_t age = 0;
        startTime = esp_timer_get_time();
        do {
            age = (unsigned long) (esp_timer_get_time() - startTime);
            if (age > 90) {
                portEXIT_CRITICAL(&mux);
                return DHT_TIMEOUT_ERROR;
            }
        } while (gpio_get_level(pin) == (i & 1) ? 1 : 0);

        if (i >= 0 && (i & 1)) {
            // Now we are being fed our 40 bits
            data <<= 1;

            // A zero max 30 usecs, a one at least 68 usecs.
            if (age > 30) {
                data |= 1; // we got a one
            }
        }

        switch (i) {
            case 31:
                rawHumidity = data;
                break;
            case 63:
                rawTemperature = data;
                data = 0;
                break;
        }
    }
    portEXIT_CRITICAL(&mux);

    if ((uint8_t) ((uint16_t) (rawHumidity + (rawHumidity >> 8) + rawTemperature + (rawTemperature >> 8))) != data) {
        return DHT_CHECKSUM_ERROR;
    }
    humidity = rawHumidity * 0.1;
    if (rawTemperature & 0x8000) {
        rawTemperature = -(int16_t) (rawTemperature & 0x7FFF);
    }
    temperature = ((int16_t) rawTemperature) * 0.1;

    buf[0] = (uint8_t) ((rawHumidity >> 8) & 0xFF);
    buf[1] = (uint8_t) (rawHumidity & 0xFF);
    buf[3] = (uint8_t) (rawTemperature & 0xFF);
    buf[2] = (uint8_t) ((rawTemperature >> 8) & 0xFF);
    buf[4] = data;
    return DHT_OK;
}

static term nif_dht_read(Context *ctx, int argc, term argv[])
{
    UNUSED(argc);
    term pin = argv[0];
    VALIDATE_VALUE(pin, term_is_integer);

    uint8_t buf[5];
    memset(buf, 0, 5);
    int err = read_into(term_to_int(pin), buf);
    if (err == PROTOCOL_ERROR) {
        if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(2)) != MEMORY_GC_OK)) {
            RAISE_ERROR(OUT_OF_MEMORY_ATOM);
        } else {
            term error_tuple = term_alloc_tuple(2, &ctx->heap);
            term_put_tuple_element(error_tuple, 0, ERROR_ATOM);
            term_put_tuple_element(error_tuple, 1, globalcontext_make_atom(ctx->global, dht_bad_read));
            return error_tuple;
        }
    } else {
        if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(2) + term_binary_heap_size(5)) != MEMORY_GC_OK)) {
            RAISE_ERROR(OUT_OF_MEMORY_ATOM);
        } else {
            term ok_tuple = term_alloc_tuple(2, &ctx->heap);
            term_put_tuple_element(ok_tuple, 0, OK_ATOM);
            term_put_tuple_element(ok_tuple, 1, term_from_literal_binary(buf, 5, &ctx->heap, ctx->global));
            return ok_tuple;
        }
    }
}


static const struct Nif dht_read_nif =
{
    .base.type = NIFFunctionType,
    .nif_ptr = nif_dht_read
};

void atomvm_dht_init(GlobalContext *global)
{
    // no-op
}

const struct Nif *atomvm_dht_get_nif(const char *nifname)
{
    if (strcmp("dht:read/1", nifname) == 0) {
        TRACE("Resolved platform nif %s ...\n", nifname);
        return &dht_read_nif;
    }
    return NULL;
}

#include <sdkconfig.h>
#ifdef CONFIG_AVM_DHT_ENABLE
REGISTER_NIF_COLLECTION(atomvm_dht, atomvm_dht_init, NULL, atomvm_dht_get_nif)
#endif
