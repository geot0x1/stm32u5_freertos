#ifndef NVS_H
#define NVS_H
#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define NVS_ERR               (-1)
#define NVS_ERR_READ_NO_SPACE (-2)
#define NVS_OK                (0)
#define NVS_EMPTY             (1)
#define NVS_NOT_READY         (2)

typedef struct
{
    uint16_t first_sector;
    uint16_t last_sector;
    uint32_t sector_size;
    uint16_t page_size;

    uint32_t          ate_write_addr;
    uint32_t          data_write_addr;
    uint32_t          ate_read_addr;
    uint32_t          start_read_addr;
    SemaphoreHandle_t semaphore;
    bool              ready;
} Nvs;

/**
 * @brief Initializes the Non-Volatile Storage (NVS).
 *
 * This function initializes the NVS by setting private values, restoring the head from flash memory,
 * and printing the head information.
 *
 * @param nvs Pointer to the `Nvs` structure representing the Non-Volatile Storage.
 * @return Returns NVS_OK on successful initialization, or an error code on failure.
 */
int nvs_init(Nvs* nvs);

/**
 * @brief Writes data to the Non-Volatile Storage (NVS).
 *
 * This function writes data to the NVS. It first checks if there is enough space in the current sector,
 * and if not, it moves to the next sector, erases it, writes a closing block, and updates the head.
 * It then generates a new ID, calculates the data offset, creates metadata, and writes both metadata and data.
 * If the read pointer is at the default state, it is set to the write pointer.
 *
 * @param nvs Pointer to the `Nvs` structure representing the Non-Volatile Storage.
 * @param data Pointer to the data to be written.
 * @param data_len Length of the data to be written.
 * @return Returns NVS_OK on successful write, or an error code on failure.
 */
int nvs_write(Nvs* nvs, void* data, size_t data_len);

/**
 * @brief Reads data from the Non-Volatile Storage (NVS) into a buffer.
 *
 * This function reads data from the NVS into the provided buffer.
 *
 * @param nvs Pointer to the `Nvs` structure representing the Non-Volatile Storage.
 * @param data Pointer to the buffer where the data will be stored.
 * @param size Size of the buffer in bytes.
 * @return Returns 0 on success, or -1 if there is no data in the flash or if the data is too big to fit in the buffer.
 */
int nvs_read(Nvs* nvs, void* data, size_t size);

/**
 * @brief Deletes data from the Non-Volatile Storage (NVS).
 *
 * This function deletes data from the NVS based on the specified number of items.
 *
 * @param nvs Pointer to the `Nvs` structure representing the Non-Volatile Storage.
 * @return Returns `NVS_OK` on success, or `NVS_ERR` if there is no data in the flash.
 */
int nvs_delete(Nvs* nvs);

/**
 * @brief Erases the Non-Volatile Storage (NVS) area.
 *
 * This function erases the NVS area by performing a 64k block erase on each sector within the
 * specified range.
 *
 * @param nvs Pointer to the `Nvs` structure representing the Non-Volatile Storage.
 */
void nvs_erase(Nvs* nvs);

/**
 * @brief Mounts a fifo area to the physical memory device.
 * 
 * @param nvs Pointer to the object containing the data management.
 */
void nvs_mount(Nvs *nvs);


#ifdef __cplusplus
}
#endif
#endif