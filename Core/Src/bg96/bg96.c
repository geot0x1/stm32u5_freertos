#include "bg96.h"
#include "stm32u5xx_ll_gpio.h"
#include "stm32u5xx_ll_bus.h"
#include "stm32u545xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "gsm_hal.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h> // Required for atoi


typedef struct
{
    const char* command;
    char response[256];
    size_t length;
    void* context;
    // response_handler should return non-zero when it has fully processed the
    // response and the caller should stop waiting for more data.
    int (*response_handler)(void* ctx, const char* response, size_t length);
} AtHandler;


static void print_response(const char* response, size_t length);
static bool is_ok(const char* response);
static bool is_error(const char* response);
static const char* skip_spaces(const char* s);
static int send_at_command(GsmStream* stream, const char* command, AtHandler* handler, uint32_t timeout);
static const char* get_string_until(const char* start, char delimiter, char* out_buf, size_t buf_size);
static const char* get_integer_until(const char* start, char delimiter, int* out_val);



static int native_modem_serial_init(GsmStream* self, int baudrate)
{
    return gsm_hal_serial_init(baudrate);
}

static int native_modem_serial_write(GsmStream* self, const uint8_t* data, uint16_t len)
{
    return gsm_hal_serial_write(data, len);
}

static int native_modem_serial_read(GsmStream* self, uint8_t* data, uint16_t len)
{
    return gsm_hal_serial_read(data, len);
}

static int native_modem_serial_close(GsmStream* self)
{
    // If there's no close function, we can just return 0
    return 0;
}


GsmStreamVtable native_stream_vtable = {
    .open = native_modem_serial_init,
    .write = native_modem_serial_write,
    .read = native_modem_serial_read,
    .close = native_modem_serial_close
};

GsmStream native_stream = {
    .context = NULL,
    .vtable = &native_stream_vtable
};

static void at_handler_clear(AtHandler* handler)
{
    handler->length = 0;
    handler->response[0] = '\0';
}

static int at_handler_append_char(AtHandler* handler, char c)
{
    if (handler->length < sizeof(handler->response) - 1)
    {
        handler->response[handler->length++] = c;
        handler->response[handler->length] = '\0'; // Null-terminate
        return 0;
    }
    return -1; // Buffer full
}

int bg96_send_at_command(Bg96* module, AtHandler* handler, uint32_t timeout_ms)
{
    return send_at_command(&native_stream, handler->command, handler, timeout_ms);
}

static int send_at_command(GsmStream* stream, const char* command, AtHandler* handler, uint32_t timeout)
{
    if (!stream || !stream->vtable || !stream->vtable->write || !stream->vtable->read || !handler || !command)
    {
        return AT_ERR_BAD_INPUT;
    }

    at_handler_clear(handler);
    // Send the command
    printf("Sending: %s\r\n", command);
    stream->vtable->write(stream, (const uint8_t*)command, (uint16_t)strlen(command));

    // Read loop: poll for response until timeout (timeout in ms)
    uint32_t start = HAL_GetTick();

    int err = AT_ERR_TIMEOUT;
    while ((HAL_GetTick() - start) < timeout)
    {
        uint8_t ch;
        int n = stream->vtable->read(stream, &ch, 1);
        if (n > 0)
        {
            if (at_handler_append_char(handler, (char)ch) < 0)
            {
                at_handler_clear(handler);
            }
            if (handler->length >= 2 && 
                handler->response[handler->length - 2] == '\r' &&
                handler->response[handler->length - 1] == '\n')
            {
                print_response(handler->response, handler->length);
                
                if (handler->response_handler)
                {
                    size_t resp_len = handler->length;
                    err = handler->response_handler(handler->context, handler->response, resp_len);
                }
                at_handler_clear(handler);
                if (err <= AT_SUCCESS)
                {
                    return err;
                }
            }
        }
        else
        {
            // No data available — small delay
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    // Timeout — return with whatever we have
    return AT_ERR_TIMEOUT;
}

static int parse_simple_at(void* ctx, const char* response, size_t length)
{
    if (is_ok(response))
    {
        printf("Command succeeded.\n");
        return AT_SUCCESS;
    }
    else if (is_error(response))
    {
        printf("Command failed.\n");
        return AT_ERR_FAIL;
    }
    return AT_PENDING;
}


// Parse +CREG: lines. This handler follows the AtHandler response_handler signature.
// +CREG: 0,1
static int creg_response_handler(void* ctx, const char* response, size_t length)
{
    if (!ctx || !response) return 0;

    Bg96CregStatus* st = (Bg96CregStatus*)ctx;
    if (is_error(response))
    {
        return AT_ERR_FAIL;
    }

    if (st->response_found)
    {
        if (is_ok(response))
        {
            printf("Received OK response.\n");
            st->ok_found = true;
            return AT_SUCCESS;
        }
    }
    else
    {
        const char* prefix = "+CREG:";
        if (strncmp(response, prefix, strlen(prefix)) == 0 && length >= 10)
        {
            const char *comma = strchr(response, ',');
            if (comma)
            {
                const char *net_status_ptr = skip_spaces(comma + 1);
                st->response_found = true;
                st->n = *net_status_ptr - '0';
            }
            return AT_PENDING;
        }
    }
    
    return AT_PENDING;
}

static int parse_qnwinfo(void* ctx, const char* response, size_t length)
{
    if (!ctx || !response) return 0;

    Bg96NetworkInfo* info = (Bg96NetworkInfo*)ctx;
    if (is_error(response))
    {
        return AT_ERR_FAIL;
    }

    if (info->response_found)
    {
        if (is_ok(response))
        {
            printf("Received OK response.\n");
            info->ok_found = true;
            return AT_SUCCESS;
        }
    }
    else
    {
        const char* prefix = "+QNWINFO:";
        if (strncmp(response, prefix, strlen(prefix)) == 0)
        {
            // Example response: +QNWINFO: "CAT-M","Operator","B2",1234
            const char* ptr = response + strlen(prefix);
            if (ptr)
            {
                ptr = get_string_until(ptr, ',', info->radio_access_tech, sizeof(info->radio_access_tech));
                ptr = get_string_until(ptr, ',', info->operator_name, sizeof(info->operator_name));
                ptr = get_string_until(ptr, ',', info->band_name, sizeof(info->band_name));
                ptr = get_integer_until(ptr, '\r', &info->channel_number);
                info->response_found = true;
            }
            return AT_PENDING;
        }
    }
    
    return AT_PENDING;
}

int bg96_disable_echo(Bg96* module)
{
    AtHandler handler;
    memset(&handler, 0, sizeof(handler));
    handler.response_handler = parse_simple_at;
    handler.context = NULL;
    handler.command = "ATE0\r";
    for (int i = 0; i < 3; i++)
    {
        int ret = bg96_send_at_command(module, &handler, 500);
        if (ret == AT_SUCCESS)
        {
            return AT_SUCCESS;
        }
    }
    return AT_ERR_FAIL;
}

int bg96_send_simple_at(Bg96* module)
{
    AtHandler handler;
    memset(&handler, 0, sizeof(handler));
    handler.response_handler = parse_simple_at;
    handler.context = NULL;
    handler.command = "AT\r";
    for (int i = 0; i < 3; i++)
    {
        int ret = bg96_send_at_command(module, &handler, 500);
        if (ret == AT_SUCCESS)
        {
            return AT_SUCCESS;
        }
    }
    return AT_ERR_FAIL;
}

int bg96_query_network_info(Bg96* module, Bg96NetworkInfo* info)
{
    AtHandler handler;
    memset(&handler, 0, sizeof(handler));
    handler.response_handler = parse_qnwinfo;
    handler.context = info;
    handler.command = "AT+QNWINFO\r";
    for (int i = 0; i < 3; i++)
    {
        memset(info, 0, sizeof(Bg96NetworkInfo));
        int ret = bg96_send_at_command(module, &handler, 2000);
        if (ret == AT_SUCCESS)
        {
            return AT_SUCCESS;
        }
    }
    return AT_ERR_FAIL;
}

// AT+QCFG="iotopmode",0,1
int bg96_set_iotopmode(Bg96* module)
{
    AtHandler handler;
    memset(&handler, 0, sizeof(handler));
    handler.response_handler = parse_simple_at;
    handler.context = NULL;
    handler.command = "AT+QCFG=\"iotopmode\",1\r";

    for (int i = 0; i < 3; i++)
    {
        int ret = bg96_send_at_command(module, &handler, 2000);
        if (ret == AT_SUCCESS)
        {
            return AT_SUCCESS;
        }
    }
    return AT_ERR_FAIL;
}

int bg96_initialize(Bg96* module)
{
    if (bg96_send_simple_at(module) != AT_SUCCESS)
    {
        return AT_ERR_FAIL;
    }
    if (bg96_disable_echo(module) != AT_SUCCESS)
    {
        return AT_ERR_FAIL;
    }
    if (bg96_get_network_registration(module, 10000) != AT_SUCCESS)
    {
        return AT_ERR_FAIL;
    }
    Bg96NetworkInfo info;
    if (bg96_query_network_info(module, &info) != AT_SUCCESS)
    {
        return AT_ERR_FAIL;
    }
    return AT_SUCCESS;
}

// int bg96_

int bg96_connect(Bg96* module)
{
    // if (bg96_send_simple_at(module) != AT_SUCCESS)
    // {
    //     return AT_ERR_FAIL;
    // }
    // if (bg96_disable_echo(module) != AT_SUCCESS)
    // {
    //     return AT_ERR_FAIL;
    // }
    if (bg96_get_network_registration(module, 10000) != AT_SUCCESS)
    {
        return AT_ERR_FAIL;
    }
    Bg96NetworkInfo info;
    if (bg96_query_network_info(module, &info) != AT_SUCCESS)
    {
        return AT_ERR_FAIL;
    }
    printf("Connected to network: RAT=%s, OP_NAME=%s, BAND=%s, CHANNEL=%d\r\n",
           info.radio_access_tech,
           info.operator_name,
           info.band_name,
           info.channel_number);
    // if (bg96_set_iotopmode(module) != AT_SUCCESS)
    // {
    //     return AT_ERR_FAIL;
    // }

    return 0;
}

int bg96_query_creg(Bg96* module, Bg96CregStatus* status)
{
    if (!status)
        return -1;

    AtHandler handler;
    memset(&handler, 0, sizeof(handler));
    handler.response_handler = creg_response_handler;
    handler.context = status;
    handler.length = 0;
    handler.command = "AT+CREG?\r";

    for (int i = 0; i < 3; i++)
    {
        memset(status, 0, sizeof(Bg96CregStatus));
        int r = bg96_send_at_command(module, &handler, 2000);
        if (r == AT_SUCCESS)
        {
            return AT_SUCCESS;
        }
    }
    return AT_ERR_FAIL;
}


int bg96_send_at(Bg96* module, const char* command, uint32_t timeout_ms)
{
    AtHandler handler;
    memset(&handler, 0, sizeof(handler));
    handler.response_handler = parse_simple_at;
    handler.context = NULL;
    handler.length = 0;

    return send_at_command(&native_stream, command, &handler, timeout_ms);
}

int bg96_get_network_registration(Bg96* module, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms)
    {
        Bg96CregStatus status;
        int r = bg96_query_creg(module, &status);
        if (r != AT_SUCCESS)
        {
            return AT_ERR_FAIL;
        }
        if (!status.response_found || !status.ok_found)
        {
            return AT_ERR_FAIL;
        }
        if (status.n == BG96_NETWORK_REG_REGISTERED_HOME || status.n == BG96_NETWORK_REG_REGISTERED_ROAMING)
        {
            return AT_SUCCESS;
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
    return AT_ERR_TIMEOUT;
}


void bg96_reset_pin_on(void)
{
    gsm_hal_reset_pin_set_high();
}

void bg96_reset_pin_off(void)
{
    gsm_hal_reset_pin_set_low();
}

void bg96_pwrkey_pin_on(void)
{
    gsm_hal_pwrkey_pin_set_high();
}

void bg96_pwrkey_pin_off(void)
{
    gsm_hal_pwrkey_pin_set_low();
}

int bg96_init(Bg96* module)
{
    gsm_hal_gpio_init();

    // Set initial states
    gsm_hal_reset_pin_set_low(); // Reset pin low
    gsm_hal_pwrkey_pin_set_high(); // Power key pin high

    gsm_hal_serial_init(115200);
    return 0;
}

void bg96_power_on(Bg96* module)
{
    bg96_pwrkey_pin_on(); // Toggle power key (board-specific polarity)
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second
    bg96_pwrkey_pin_off(); // Set power key high to complete power on
}

static void print_response(const char* response, size_t length)
{
    printf("BG96 Response: ");
    for (size_t i = 0; i < length; i++)
    {
        if (response[i] == '\r')
        {
            printf("\\r");
        }
        else if (response[i] == '\n')
        {
            printf("\\n");
        }
        else
        {
            printf("%c", response[i]);
        }
    }
    printf("\r\n");
}

static bool is_ok(const char* response)
{
    const char* s = skip_spaces(response);
    return strncmp(s, "OK\r\n", 4) == 0;
}

static bool is_error(const char* response)
{
    const char* s = skip_spaces(response);
    return strncmp(s, "ERROR\r\n", 7) == 0;
}

// Helper: trim leading spaces
static const char* skip_spaces(const char* s)
{
    while (*s == ' ' || *s == '\t') s++;
    return s;
}



/**
 * @brief Skips leading delimiters/garbage, copies characters until the next delimiter,
 * and filters specified "garbage" characters during the copy.
 *
 * @param start The pointer to the beginning of the source string.
 * @param delimiter The character that marks the end of the string segment (e.g., ',').
 * @param out_buf The output buffer for the copied string.
 * @param buf_size The maximum size of the output buffer.
 * @return const char* Pointer to the character immediately following the stopping delimiter
 * in the source string, or to the null terminator if reached.
 */
static const char* get_string_until(const char* start, char delimiter, char* out_buf, size_t buf_size)
{
    // --- Phase 1: Skip leading garbage and the initial delimiter ---

    // Define the set of characters to treat as leading garbage
    const char* garbage_chars = " \t/\\'\","; // Space, Tab, Slash, Backslash, Single-quote, Double-quote

    // 1. Skip all leading 'garbage' characters
    start = skip_spaces(start); // Use the existing skip_spaces helper (which includes tab/space)
    
    // 2. Skip the delimiter if it is the very first character (handles ",token")
    if (*start == delimiter)
    {
        start++;
    }
    
    // 3. Skip any remaining garbage characters after the delimiter
    while (*start != '\0' && strchr(garbage_chars, *start) != NULL)
    {
        start++;
    }
    
    // --- Phase 2: Copy until the next delimiter or buffer end ---
    
    size_t out_index = 0;
    const char* read_ptr = start;

    while (*read_ptr != '\0' && *read_ptr != delimiter && out_index < buf_size - 1)
    {
        // Define the set of characters to skip/filter during the copy
        // We include CR/LF which terminate the line, but also your specified garbage.
        const char* filter_chars = " \t/\\'\"\r\n"; 

        // If the character is NOT in the filter set, copy it
        if (strchr(filter_chars, *read_ptr) == NULL)
        {
            out_buf[out_index++] = *read_ptr;
        }
        else if (*read_ptr == '\r' || *read_ptr == '\n')
        {
             // If CR or LF is found, treat it as an immediate termination point.
             break;
        }
        
        read_ptr++;
    }

    out_buf[out_index] = '\0'; // Null-terminate the output string

    // Return the pointer to where copying stopped in the source string
    return read_ptr;
}

/**
 * @brief Skips leading delimiters/garbage and extracts an integer value
 * until the next delimiter, whitespace, or end-of-line is found.
 *
 * This function is non-destructive and safe for lightweight embedded parsing.
 *
 * @param start The pointer to the beginning of the source string.
 * @param delimiter The character that marks the end of the integer segment (e.g., ',').
 * @param out_val Pointer to the integer variable to store the extracted value.
 * @return const char* Pointer to the character immediately following the stopping delimiter
 * in the source string, or to the null terminator if reached.
 */
static const char* get_integer_until(const char* start, char delimiter, int* out_val)
{
    const char* read_ptr = start;
    
    // --- Phase 1: Skip leading garbage and the initial delimiter ---

    // 1. Skip all leading whitespace
    read_ptr = skip_spaces(read_ptr);
    
    // 2. Skip the delimiter if it is the very first non-space character (e.g., skips the comma in ",123")
    if (*read_ptr == delimiter)
    {
        read_ptr++;
    }
    
    // 3. Skip any remaining non-numeric, non-terminating garbage (like quotes/slashes)
    const char* non_numeric_garbage = "/\\'\","; 
    while (*read_ptr != '\0' && strchr(non_numeric_garbage, *read_ptr) != NULL)
    {
        read_ptr++;
    }
    
    // --- Phase 2: Copy the numeric part and convert ---

    const char* numeric_start = read_ptr;
    const char* numeric_end = read_ptr;
    
    // Find the end of the number (next delimiter, space, or end of line)
    while (*numeric_end != '\0' && *numeric_end != delimiter && *numeric_end != ' ' && *numeric_end != '\t' && *numeric_end != '\r' && *numeric_end != '\n')
    {
        numeric_end++;
    }

    size_t length = numeric_end - numeric_start;
    
    // Use a small, temporary buffer on the stack for the string representation
    char temp_buf[16]; // Sufficient for a 32-bit integer string representation
    
    if (length > 0 && length < sizeof(temp_buf))
    {
        strncpy(temp_buf, numeric_start, length);
        temp_buf[length] = '\0';
        
        // Convert the string buffer to an integer
        *out_val = atoi(temp_buf);
    }
    else
    {
        // Handle error or empty string case by setting the value to 0
        *out_val = 0;
    }
    
    // --- Phase 3: Advance the pointer past the delimiter ---

    read_ptr = numeric_end;

    // If the loop stopped on the delimiter (e.g., comma), advance past it.
    if (*read_ptr == delimiter)
    {
        read_ptr++;
    }
    
    // Return the pointer to the next meaningful token
    return read_ptr;
}