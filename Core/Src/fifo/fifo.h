#ifndef FIFO_H
#define FIFO_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * Circular FIFO buffer structure.
 * Stores up to size elements, using a count variable to track occupancy.
 */
typedef struct {
    uint8_t* buffer;          // User-provided buffer
    uint16_t size;            // Buffer size (must be >= 1 and <= 65535)
    volatile uint16_t head;   // Write index
    volatile uint16_t tail;   // Read index
    volatile uint16_t count;  // Number of elements in the buffer
} Fifo;

/**
 * Initializes a FIFO buffer with a user-provided buffer and size.
 * @param fifo Pointer to the FIFO structure.
 * @param buf Pointer to the user-allocated buffer (must not be NULL).
 * @param size Size of the buffer (must be >= 1 and <= 65535).
 * @return true if initialization succeeds, false otherwise.
 */
bool fifo_init(Fifo *fifo, uint8_t *buf, size_t size);

/**
 * Pushes a single byte onto the FIFO.
 * @param fifo Pointer to the FIFO structure.
 * @param data Byte to push.
 * @return true if successful, false if the buffer is full or invalid.
 */
bool fifo_push(Fifo *fifo, uint8_t data);

/**
 * Pushes multiple bytes onto the FIFO.
 * @param fifo Pointer to the FIFO structure.
 * @param data Pointer to the bytes to push.
 * @param count Number of bytes to push.
 * @return Number of bytes successfully pushed (0 if invalid or full).
 */
uint16_t fifo_push_bytes(Fifo *fifo, const uint8_t *data, uint16_t count);

/**
 * Pops a single byte from the FIFO.
 * @param fifo Pointer to the FIFO structure.
 * @param data Pointer to store the popped byte.
 * @return true if successful, false if the buffer is empty or invalid.
 */
bool fifo_pop(Fifo *fifo, uint8_t *data);

/**
 * Pops multiple bytes from the FIFO.
 * @param fifo Pointer to the FIFO structure.
 * @param data Pointer to store the popped bytes.
 * @param count Number of bytes to pop.
 * @return Number of bytes successfully popped (0 if invalid or empty).
 */
uint16_t fifo_pop_bytes(Fifo *fifo, uint8_t *data, uint16_t count);

/**
 * Checks if the FIFO is empty.
 * @param fifo Pointer to the FIFO structure.
 * @return true if empty or invalid, false otherwise.
 */
bool fifo_is_empty(Fifo *fifo);

/**
 * Checks if the FIFO is full.
 * @param fifo Pointer to the FIFO structure.
 * @return true if full or invalid, false otherwise.
 */
bool fifo_is_full(Fifo *fifo);

/**
 * Returns the number of elements in the FIFO.
 * @param fifo Pointer to the FIFO structure.
 * @return Number of elements, or 0 if invalid.
 */
uint16_t fifo_count(Fifo *fifo);

/**
 * Resets the FIFO to its initial state (empty).
 * @param fifo Pointer to the FIFO structure.
 */
void fifo_reset(Fifo *fifo);

#ifdef __cplusplus
}
#endif
#endif