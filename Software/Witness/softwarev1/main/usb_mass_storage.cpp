#include "usb_mass_storage.h"

#include <algorithm>
#include <cstring>

#include "flash_logger.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tusb.h"

namespace {
UsbMassStorage *g_storage = nullptr;

void put16(std::uint8_t *p, std::uint16_t v) { p[0] = v; p[1] = v >> 8; }
void put32(std::uint8_t *p, std::uint32_t v) {
    p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}

constexpr tusb_desc_device_t kDeviceDescriptor = {
    .bLength = sizeof(tusb_desc_device_t), .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200, .bDeviceClass = 0, .bDeviceSubClass = 0,
    .bDeviceProtocol = 0, .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = TINYUSB_ESPRESSIF_VID, .idProduct = 0x4012,
    .bcdDevice = 0x0100, .iManufacturer = 1, .iProduct = 2,
    .iSerialNumber = 3, .bNumConfigurations = 1,
};
constexpr std::uint8_t kConfigurationDescriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_MSC_DESCRIPTOR(0, 4, 0x01, 0x81, 64),
};
const char *kStrings[] = {
    (const char[]){0x09, 0x04}, "Witness", "Witness Read-Only Logs",
    "0001", "Log storage",
};
} // namespace

esp_err_t UsbMassStorage::build_index() {
    file_count_ = std::min<std::size_t>(logger_.export_file_count(), 32);
    std::uint32_t next = 2;
    for (std::size_t i = 0; i < file_count_; ++i) {
        const std::uint32_t bytes = logger_.export_file(i).size;
        const std::uint32_t clusters =
            (bytes + kSectorSize * kSectorsPerCluster - 1) /
            (kSectorSize * kSectorsPerCluster);
        if (next + clusters > 0xFFF0U) return ESP_ERR_NO_MEM;
        first_cluster_[i] = clusters ? static_cast<std::uint16_t>(next) : 0;
        cluster_count_[i] = static_cast<std::uint16_t>(clusters);
        next += clusters;
    }
    return ESP_OK;
}

esp_err_t UsbMassStorage::start() {
    esp_err_t r = build_index(); if (r != ESP_OK) return r;
    g_storage = this;
    tinyusb_config_t config = TINYUSB_DEFAULT_CONFIG();
    config.descriptor.device = &kDeviceDescriptor;
    config.descriptor.string = kStrings;
    config.descriptor.string_count = sizeof(kStrings) / sizeof(kStrings[0]);
    config.descriptor.full_speed_config = kConfigurationDescriptor;
    return tinyusb_driver_install(&config);
}

void UsbMassStorage::make_boot_sector(std::uint8_t *d) const {
    std::memset(d, 0, kSectorSize);
    d[0] = 0xEB; d[1] = 0x3C; d[2] = 0x90;
    std::memcpy(d + 3, "WITNESS ", 8); put16(d + 11, kSectorSize);
    d[13] = kSectorsPerCluster; put16(d + 14, kReservedSectors); d[16] = 1;
    put16(d + 17, kRootEntries); put16(d + 19, 0); d[21] = 0xF8;
    put16(d + 22, kFatSectors); put16(d + 24, 32); put16(d + 26, 64);
    put32(d + 28, 0); put32(d + 32, kSectorCount);
    d[36] = 0x80; d[38] = 0x29; put32(d + 39, 0x57544E53);
    std::memcpy(d + 43, "WITNESSLOGS", 11); std::memcpy(d + 54, "FAT16   ", 8);
    d[510] = 0x55; d[511] = 0xAA;
}

void UsbMassStorage::make_fat_sector(std::uint32_t sector, std::uint8_t *d) const {
    std::memset(d, 0, kSectorSize);
    const std::uint32_t first_entry = (sector - kFatStart) * (kSectorSize / 2);
    for (std::uint32_t j = 0; j < kSectorSize / 2; ++j) {
        const std::uint32_t cluster = first_entry + j;
        std::uint16_t value = cluster == 0 ? 0xFFF8 : (cluster == 1 ? 0xFFFF : 0);
        for (std::size_t i = 0; i < file_count_ && value == 0; ++i) {
            const std::uint32_t first = first_cluster_[i];
            const std::uint32_t count = cluster_count_[i];
            if (count && cluster >= first && cluster < first + count)
                value = cluster + 1 == first + count ? 0xFFFF : static_cast<std::uint16_t>(cluster + 1);
        }
        put16(d + j * 2, value);
    }
}

void UsbMassStorage::make_root_sector(std::uint32_t sector, std::uint8_t *d) const {
    std::memset(d, 0, kSectorSize);
    const std::uint32_t first = (sector - kRootStart) * 16;
    for (std::uint32_t j = 0; j < 16; ++j) {
        std::uint8_t *entry = d + j * 32;
        const std::uint32_t index = first + j;
        if (index == 0) { std::memcpy(entry, "WITNESSLOGS", 11); entry[11] = 0x08; continue; }
        const std::size_t file_index = index - 1;
        if (file_index >= file_count_) break;
        const auto &file = logger_.export_file(file_index);
        std::memset(entry, ' ', 11);
        const char *dot = std::strchr(file.name, '.');
        const std::size_t base = dot ? static_cast<std::size_t>(dot - file.name) : std::strlen(file.name);
        std::memcpy(entry, file.name, std::min<std::size_t>(base, 8));
        if (dot) std::memcpy(entry + 8, dot + 1, std::min<std::size_t>(std::strlen(dot + 1), 3));
        entry[11] = 0x21; // archive + read-only
        put16(entry + 26, first_cluster_[file_index]); put32(entry + 28, file.size);
    }
}

esp_err_t UsbMassStorage::make_data_sector(std::uint32_t sector, std::uint8_t *d) {
    std::memset(d, 0, kSectorSize);
    const std::uint32_t cluster = 2 + (sector - kDataStart) / kSectorsPerCluster;
    const std::uint32_t sector_in_cluster = (sector - kDataStart) % kSectorsPerCluster;
    for (std::size_t i = 0; i < file_count_; ++i) {
        const std::uint32_t first = first_cluster_[i], count = cluster_count_[i];
        if (!count || cluster < first || cluster >= first + count) continue;
        const std::uint32_t offset = ((cluster - first) * kSectorsPerCluster + sector_in_cluster) * kSectorSize;
        const auto &file = logger_.export_file(i);
        if (offset >= file.size) return ESP_OK;
        const std::size_t amount = std::min<std::uint32_t>(kSectorSize, file.size - offset);
        return logger_.read_export_file(i, offset, d, amount);
    }
    return ESP_OK;
}

esp_err_t UsbMassStorage::read_sector(std::uint32_t sector, std::uint8_t *d) {
    if (sector >= kSectorCount) return ESP_ERR_INVALID_ARG;
    if (sector == 0) make_boot_sector(d);
    else if (sector < kRootStart) make_fat_sector(sector, d);
    else if (sector < kDataStart) make_root_sector(sector, d);
    else return make_data_sector(sector, d);
    return ESP_OK;
}

int UsbMassStorage::read(std::uint32_t lba, std::uint32_t offset, void *buffer, std::uint32_t length) {
    auto *out = static_cast<std::uint8_t *>(buffer); std::uint32_t done = 0;
    std::uint8_t sector[kSectorSize];
    while (done < length) {
        const std::uint64_t absolute = static_cast<std::uint64_t>(lba) * kSectorSize + offset + done;
        const std::uint32_t sector_number = absolute / kSectorSize;
        const std::uint32_t within = absolute % kSectorSize;
        if (read_sector(sector_number, sector) != ESP_OK) return -1;
        const std::uint32_t take = std::min(length - done, kSectorSize - within);
        std::memcpy(out + done, sector + within, take); done += take;
    }
    return static_cast<int>(done);
}

extern "C" void tud_msc_inquiry_cb(uint8_t, uint8_t vendor[8], uint8_t product[16], uint8_t revision[4]) {
    std::memcpy(vendor, "WITNESS ", 8); std::memcpy(product, "READ-ONLY LOGS  ", 16); std::memcpy(revision, "1.0 ", 4);
}
extern "C" bool tud_msc_test_unit_ready_cb(uint8_t) { return g_storage != nullptr; }
extern "C" void tud_msc_capacity_cb(uint8_t, uint32_t *blocks, uint16_t *size) { *blocks = 262144; *size = 512; }
extern "C" bool tud_msc_is_writable_cb(uint8_t) { return false; }
extern "C" int32_t tud_msc_read10_cb(uint8_t, uint32_t lba, uint32_t offset, void *buffer, uint32_t size) {
    return g_storage ? g_storage->read(lba, offset, buffer, size) : -1;
}
extern "C" int32_t tud_msc_write10_cb(uint8_t lun, uint32_t, uint32_t, uint8_t *, uint32_t) {
    tud_msc_set_sense(lun, SCSI_SENSE_DATA_PROTECT, 0x27, 0); return -1;
}
extern "C" int32_t tud_msc_scsi_cb(uint8_t lun, const uint8_t command[16], void *, uint16_t) {
    if (command[0] == SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL || command[0] == SCSI_CMD_START_STOP_UNIT) return 0;
    tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0); return -1;
}
