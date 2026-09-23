#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

class FlashLogger;

// Read-only FAT16 projection of the immutable FlashLogger snapshot.
class UsbMassStorage final {
public:
    explicit UsbMassStorage(FlashLogger &logger) : logger_(logger) {}
    esp_err_t start();
    int read(std::uint32_t lba, std::uint32_t offset,
             void *buffer, std::uint32_t length);

private:
    static constexpr std::uint32_t kSectorSize = 512;
    static constexpr std::uint32_t kSectorCount = 262144; // 128 MiB
    static constexpr std::uint32_t kSectorsPerCluster = 8;
    static constexpr std::uint32_t kReservedSectors = 1;
    static constexpr std::uint32_t kFatSectors = 128;
    static constexpr std::uint32_t kRootEntries = 512;
    static constexpr std::uint32_t kRootSectors = 32;
    static constexpr std::uint32_t kFatStart = kReservedSectors;
    static constexpr std::uint32_t kRootStart = kFatStart + kFatSectors;
    static constexpr std::uint32_t kDataStart = kRootStart + kRootSectors;
    static constexpr std::uint32_t kDataClusters =
        (kSectorCount - kDataStart) / kSectorsPerCluster;

    esp_err_t build_index();
    esp_err_t read_sector(std::uint32_t sector, std::uint8_t *data);
    void make_boot_sector(std::uint8_t *data) const;
    void make_fat_sector(std::uint32_t sector, std::uint8_t *data) const;
    void make_root_sector(std::uint32_t sector, std::uint8_t *data) const;
    esp_err_t make_data_sector(std::uint32_t sector, std::uint8_t *data);

    FlashLogger &logger_;
    std::uint16_t first_cluster_[32]{};
    std::uint16_t cluster_count_[32]{};
    std::size_t file_count_ = 0;
};
