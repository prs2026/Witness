#include "maintenance_mode.h"

#include "esp_attr.h"

namespace {

constexpr std::uint32_t kMagic = 0x4D41494EU;   // "MAIN"

// Not initialised at boot: survives esp_restart() and panic resets; random
// after power-on, which the magic and its complement reject.
RTC_NOINIT_ATTR std::uint32_t s_magic;
RTC_NOINIT_ATTR std::uint32_t s_magic_check;
RTC_NOINIT_ATTR std::int32_t s_tx_dbm;
RTC_NOINIT_ATTR std::uint32_t s_minutes;

bool s_active = false;
int s_boot_tx_dbm = 0;
std::uint32_t s_boot_minutes = 0;

}  // namespace

namespace maintenance_mode {

void consume_boot_request()
{
    s_active = s_magic == kMagic && s_magic_check == ~kMagic;
    if (s_active) {
        s_boot_tx_dbm = static_cast<int>(s_tx_dbm);
        s_boot_minutes = s_minutes;
    }
    cancel();   // one boot only
}

bool active() { return s_active; }
int requested_tx_dbm() { return s_boot_tx_dbm; }
std::uint32_t requested_minutes() { return s_boot_minutes; }

void request(const int tx_dbm, const std::uint32_t minutes)
{
    s_tx_dbm = tx_dbm;
    s_minutes = minutes;
    s_magic = kMagic;
    s_magic_check = ~kMagic;
}

void cancel()
{
    s_magic = 0;
    s_magic_check = 0;
}

}  // namespace maintenance_mode
