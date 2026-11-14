#include "maint_radio.h"

#include <array>
#include <cstring>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hardware_pins.h"
#include "maint_app.h"
#include "maint_beacon.h"
#include "maint_usb.h"
#include "ra01.h"
#include "task_cores.h"

namespace {
constexpr char kLogTag[] = "maint_radio";
constexpr std::size_t kMaxQueuedFrame = 24;
constexpr std::size_t kQueueDepth = 4;
constexpr std::uint32_t kLoopMs = 5;

struct QueuedFrame {
    std::uint8_t length;
    std::uint8_t data[kMaxQueuedFrame];
};

Ra01 g_radio;
bool g_ready = false;
QueueHandle_t g_queue = nullptr;

// Battery (VSENSE divider 15k/5k on GPIO1 = ADC1 channel 0), read once per
// beacon; 0 when the ADC is unavailable.
adc_oneshot_unit_handle_t g_adc = nullptr;
adc_cali_handle_t g_cali = nullptr;
bool g_adc_tried = false;

std::uint16_t battery_mv()
{
    if (!g_adc_tried) {
        g_adc_tried = true;
        adc_oneshot_unit_init_cfg_t unit{};
        unit.unit_id = ADC_UNIT_1;
        adc_oneshot_chan_cfg_t channel{};
        channel.atten = ADC_ATTEN_DB_12;
        channel.bitwidth = ADC_BITWIDTH_DEFAULT;
        adc_cali_curve_fitting_config_t cali{};
        cali.unit_id = ADC_UNIT_1;
        cali.chan = ADC_CHANNEL_0;
        cali.atten = ADC_ATTEN_DB_12;
        cali.bitwidth = ADC_BITWIDTH_DEFAULT;
        if (adc_oneshot_new_unit(&unit, &g_adc) != ESP_OK ||
            adc_oneshot_config_channel(g_adc, ADC_CHANNEL_0, &channel) != ESP_OK ||
            adc_cali_create_scheme_curve_fitting(&cali, &g_cali) != ESP_OK) {
            ESP_LOGW(kLogTag, "battery ADC unavailable; beacon reports 0 mV");
            g_cali = nullptr;
        }
    }
    if (g_adc == nullptr || g_cali == nullptr) return 0;
    std::int64_t sum = 0;
    constexpr int kSamples = 16;
    for (int i = 0; i < kSamples; ++i) {
        int raw = 0;
        if (adc_oneshot_read(g_adc, ADC_CHANNEL_0, &raw) != ESP_OK) return 0;
        sum += raw;
    }
    int sense_mv = 0;
    if (adc_cali_raw_to_voltage(g_cali, static_cast<int>(sum / kSamples), &sense_mv) != ESP_OK) return 0;
    const std::uint32_t mv = static_cast<std::uint32_t>(sense_mv) * (15000U + 5000U) / 5000U;
    return static_cast<std::uint16_t>(mv > 0xFFFFU ? 0xFFFFU : mv);
}

std::size_t build_beacon(std::uint8_t *frame)
{
    maint::Shared &shared = maint::shared();
    maint_beacon::Status s;
    s.mode = shared.mode.load();
    std::uint8_t flags = shared.flags.load();
    if (g_ready) flags |= IRIS_MAINT_FLAG_RADIO_OK;
    s.flags = flags;
    s.maint_image_id = shared.own_image_id;
    s.flight_image_id = shared.flight_image_id;
    s.battery_mv = battery_mv();
    s.uptime_ms = static_cast<std::uint32_t>(esp_timer_get_time() / 1000ULL);
    s.log_files = shared.log_files.load();
    return maint_beacon::encode(s, frame);
}

void handle_packet(const std::uint8_t *packet, const std::size_t length)
{
    if (length == IRIS_PACKET_COMMAND_FRAME_LENGTH && packet[0] == IRIS_PACKET_ID_COMMAND &&
        packet[length - 1] == IRIS_PACKET_EOF_VALUE) {
        maint::handle_command(packet, length, true);
    } else {
        ESP_LOGD(kLogTag, "radio RX ignored: %u bytes, id 0x%02X", static_cast<unsigned>(length), packet[0]);
    }
}

void radio_task(void *)
{
    TickType_t next_beacon = xTaskGetTickCount();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(kLoopMs));
        const TickType_t now = xTaskGetTickCount();
        const bool beacon_due = static_cast<std::int32_t>(now - next_beacon) >= 0;
        std::array<std::uint8_t, maint_beacon::kFrameLength> beacon{};
        if (beacon_due) {
            next_beacon = now + pdMS_TO_TICKS(IRIS_MAINT_BEACON_PERIOD_MS);
            build_beacon(beacon.data());
            maint_usb::write(beacon.data(), beacon.size());
        }
        if (!g_ready) continue;

        esp_err_t error = ESP_OK;
        (void)g_radio.poll_transmit(&error);
        if (g_radio.transmit_in_progress()) continue;

        // Acks first, then the beacon.
        QueuedFrame frame;
        if (xQueueReceive(g_queue, &frame, 0) == pdTRUE) {
            if (g_radio.start_transmit(frame.data, frame.length) != ESP_OK) {
                ESP_LOGW(kLogTag, "ack transmit failed");
            }
            continue;
        }
        if (beacon_due) {
            if (g_radio.start_transmit(beacon.data(), beacon.size()) != ESP_OK) {
                ESP_LOGW(kLogTag, "beacon transmit failed");
            }
            continue;
        }
        if (gpio_get_level(HW_PIN_RAD_DIO1) == 0) continue;
        std::array<std::uint8_t, 255> packet{};
        std::size_t length = 0;
        const esp_err_t result = g_radio.receive(packet.data(), packet.size(), length);
        if (result == ESP_OK && length > 0) handle_packet(packet.data(), length);
    }
}
}  // namespace

namespace maint_radio {

esp_err_t start(const int dbm, const bool front_end)
{
    g_queue = xQueueCreate(kQueueDepth, sizeof(QueuedFrame));
    if (g_queue == nullptr) return ESP_ERR_NO_MEM;

    // The IMU and barometer share SPI3 with the radio on the board; they are
    // not driven here, so keep their chip selects deasserted.
    gpio_config_t cs{};
    cs.pin_bit_mask = (1ULL << HW_PIN_LSM_CS) | (1ULL << HW_PIN_MS56_CS);
    cs.mode = GPIO_MODE_OUTPUT;
    (void)gpio_config(&cs);
    gpio_set_level(HW_PIN_LSM_CS, 1);
    gpio_set_level(HW_PIN_MS56_CS, 1);

    if constexpr (HW_RADIO_ENABLED != 0) {
        spi_bus_config_t bus{};
        bus.mosi_io_num = HW_PIN_SPI3_COPI;
        bus.miso_io_num = HW_PIN_SPI3_CIPO;
        bus.sclk_io_num = HW_PIN_SPI3_CLOCK;
        bus.quadwp_io_num = -1;
        bus.quadhd_io_num = -1;
        bus.max_transfer_sz = 512;
        bus.flags = SPICOMMON_BUSFLAG_MASTER;
        esp_err_t result = spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_CH_AUTO);
        if (result == ESP_OK) {
            if (dbm != 127) (void)g_radio.set_tx_power_dbm(dbm);
            g_radio.set_front_end_enabled(front_end);
            result = g_radio.initialize(SPI3_HOST);
        }
        g_ready = result == ESP_OK;
        if (!g_ready) {
            g_radio.release();
            ESP_LOGW(kLogTag, "RA-01 unavailable (%s); no LoRa link", esp_err_to_name(result));
        }
    }
    if (xTaskCreatePinnedToCore(radio_task, "maint_radio", 4096, nullptr, 4, nullptr, task_cores::kSystem) !=
        pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return g_ready ? ESP_OK : ESP_FAIL;
}

bool ready()
{
    return g_ready;
}

bool queue_frame(const std::uint8_t *frame, const std::size_t length)
{
    if (!g_ready || g_queue == nullptr || length == 0 || length > kMaxQueuedFrame) return false;
    QueuedFrame queued{};
    queued.length = static_cast<std::uint8_t>(length);
    std::memcpy(queued.data, frame, length);
    return xQueueSend(g_queue, &queued, 0) == pdTRUE;
}

bool set_power(const int dbm, const std::uint8_t front_end_code)
{
    if (front_end_code > IRIS_RADIO_FRONT_END_OFF || !g_radio.set_tx_power_dbm(dbm)) return false;
    if (front_end_code != IRIS_RADIO_FRONT_END_UNCHANGED) {
        g_radio.set_front_end_enabled(front_end_code == IRIS_RADIO_FRONT_END_ON);
    }
    return true;
}

int tx_power_dbm()
{
    return g_radio.tx_power_dbm();
}

bool front_end_on()
{
    return g_radio.front_end_enabled();
}

}  // namespace maint_radio
