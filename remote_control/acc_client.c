#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lwip/udp.h"
#include "lwip/ip4.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

// Network configuration
#define SERVER_IP "192.168.48.97"  // Replace with your server's IP
#define UDP_PORT 4242

// IP address components for server
#define SERVER_IP_0 192
#define SERVER_IP_1 168
#define SERVER_IP_2 48
#define SERVER_IP_3 97

// Connection timing parameters
#define SEND_INTERVAL_MS 500
#define ACCEL_UPDATE_INTERVAL_MS 50

// Hardware configuration
#define LED_PIN 16
#define I2C_PORT i2c0
#define GPIO_SDA_PIN 4
#define GPIO_SCL_PIN 5

// Accelerometer configuration
#define LSM303_ACC_ADDR 0x19
#define ACC_CTRL_REG1_A 0x20
#define ACC_OUT_X_L_A   0x28
#define ACC_CTRL_REG4_A 0x23
#define ACC_SENSITIVITY_2G 16384.0
#define LOW_PASS_ALPHA 0.1
#define DEAD_ZONE 0.1

typedef struct {
    float x;
    float y;
    float z;
} AccelData;

typedef struct {
    struct udp_pcb *udp_pcb;
    ip_addr_t server_addr;
    bool initialized;
    uint8_t buffer[2048];
    size_t buffer_len;
    TaskHandle_t accel_task;
    uint32_t packets_sent;
} UDP_CLIENT_T;

static UDP_CLIENT_T* client_state;

// Helper function for I2C error checking
static bool check_i2c_error(int result, const char* action) {
    if (result == PICO_ERROR_GENERIC) {
        printf("I2C error during %s\n", action);
        return false;
    }
    return true;
}

// Helper function to send UDP data with improved error handling
static bool send_udp_data(UDP_CLIENT_T* state, const char* data) {
    if (!state || !state->udp_pcb || !data) {
        printf("Invalid state for sending UDP data\n");
        return false;
    }

    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%s\n", data);
    size_t len = strlen(buffer);

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if (!p) {
        printf("Failed to allocate pbuf\n");
        return false;
    }

    memcpy(p->payload, buffer, len);
    printf("Sending UDP [%zu bytes]: %s", len, buffer);

    cyw43_arch_lwip_begin();
    err_t err = udp_sendto(state->udp_pcb, p, &state->server_addr, UDP_PORT);
    cyw43_arch_lwip_end();

    if (err != ERR_OK) {
        printf("Error sending UDP packet: %d\n", err);
        pbuf_free(p);
        return false;
    }
    printf("Successfully sent UDP packet\n");

    state->packets_sent++;
    gpio_put(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(SEND_INTERVAL_MS));
    gpio_put(LED_PIN, 0);

    pbuf_free(p);
    return true;
}

// Accelerometer functions with improved error handling
static bool write_register(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return check_i2c_error(
        i2c_write_blocking(I2C_PORT, LSM303_ACC_ADDR, buf, 2, false),
        "write_register"
    );
}

static bool read_registers(uint8_t reg, uint8_t *buf, uint8_t len) {
    if (!check_i2c_error(
            i2c_write_blocking(I2C_PORT, LSM303_ACC_ADDR, &reg, 1, true),
            "read_registers (addr)")) {
        return false;
    }
    return check_i2c_error(
        i2c_read_blocking(I2C_PORT, LSM303_ACC_ADDR, buf, len, false),
        "read_registers (data)"
    );
}

static bool init_accelerometer() {
    printf("Initializing accelerometer...\n");

    // Try to read the device ID first to verify communication
    uint8_t who_am_i = 0;
    if (!read_registers(0x0F, &who_am_i, 1)) {
        printf("Failed to read accelerometer ID\n");
        return false;
    }
    printf("Accelerometer ID: 0x%02X\n", who_am_i);

    if (!write_register(ACC_CTRL_REG1_A, 0x57)) {
        printf("Failed to initialize accelerometer REG1\n");
        return false;
    }
    if (!write_register(ACC_CTRL_REG4_A, 0x00)) {
        printf("Failed to initialize accelerometer REG4\n");
        return false;
    }
    printf("Accelerometer initialized successfully\n");
    return true;
}

static bool read_accelerometer(AccelData *data) {
    uint8_t buffer[6];
    if (!read_registers(ACC_OUT_X_L_A | 0x80, buffer, 6)) {
        printf("Failed to read accelerometer data\n");
        return false;
    }

    data->x = (int16_t)(buffer[1] << 8 | buffer[0]) / ACC_SENSITIVITY_2G;
    data->y = (int16_t)(buffer[3] << 8 | buffer[2]) / ACC_SENSITIVITY_2G;
    data->z = (int16_t)(buffer[5] << 8 | buffer[4]) / ACC_SENSITIVITY_2G;
    return true;
}

static void generate_command(const AccelData *data, char *command, size_t command_size) {
    static float filtered_x = 0, filtered_y = 0;
    filtered_x = (LOW_PASS_ALPHA * data->x) + ((1.0 - LOW_PASS_ALPHA) * filtered_x);
    filtered_y = (LOW_PASS_ALPHA * data->y) + ((1.0 - LOW_PASS_ALPHA) * filtered_y);

    float x_speed = fabs(filtered_x) * 200;
    float y_speed = fabs(filtered_y) * 200;

    x_speed = x_speed > 100 ? 100 : x_speed;
    y_speed = y_speed > 100 ? 100 : y_speed;

    // Print accelerometer values for debugging
    printf("Accel: X=%.2f Y=%.2f | Filtered: X=%.2f Y=%.2f\n",
           data->x, data->y, filtered_x, filtered_y);

    if (fabs(filtered_x) > fabs(filtered_y)) {
        if (filtered_x > DEAD_ZONE) {
            snprintf(command, command_size, "forward %.0f", x_speed);
        } else if (filtered_x < -DEAD_ZONE) {
            snprintf(command, command_size, "backward %.0f", x_speed);
        } else {
            snprintf(command, command_size, "stop");
        }
    } else {
        if (filtered_y > DEAD_ZONE) {
            snprintf(command, command_size, "right %.0f", y_speed);
        } else if (filtered_y < -DEAD_ZONE) {
            snprintf(command, command_size, "left %.0f", y_speed);
        } else {
            snprintf(command, command_size, "stop");
        }
    }
}

// FreeRTOS Tasks
static void accel_task(void *params) {
    UDP_CLIENT_T* state = (UDP_CLIENT_T*)params;
    AccelData data;
    char command[32];
    char prev_command[32] = "";
    TickType_t last_send_time = 0;
    uint32_t error_count = 0;

    printf("Accelerometer task started\n");

    while (1) {
        if (state->initialized) {
            if (read_accelerometer(&data)) {
                error_count = 0;  // Reset error count on successful read
                generate_command(&data, command, sizeof(command));

                TickType_t now = xTaskGetTickCount();
                if (strcmp(command, prev_command) != 0 ||
                    (now - last_send_time) >= pdMS_TO_TICKS(ACCEL_UPDATE_INTERVAL_MS)) {

                    if (send_udp_data(state, command)) {
                        strcpy(prev_command, command);
                        last_send_time = now;
                    }
                }
            } else {
                error_count++;
                if (error_count % 100 == 0) {  // Log every 100th error
                    printf("Accelerometer read errors: %lu\n", error_count);
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void client_task(void *params) {
    UDP_CLIENT_T* state = (UDP_CLIENT_T*)params;

    printf("\nStarting UDP client...\n");
    printf("Will attempt to connect to server at %s:%d\n", SERVER_IP, UDP_PORT);

    state->udp_pcb = udp_new();
    if (!state->udp_pcb) {
        printf("Failed to create PCB\n");
        return;
    }
    printf("UDP PCB created successfully\n");

    // Set up server address using proper IP address handling
    ip4_addr_t server_ip4;
    IP4_ADDR(&server_ip4, SERVER_IP_0, SERVER_IP_1, SERVER_IP_2, SERVER_IP_3);
    ip_addr_copy_from_ip4(state->server_addr, server_ip4);

    printf("Server address configured: %s\n", ipaddr_ntoa(&state->server_addr));

    // Bind to any port
    err_t err = udp_bind(state->udp_pcb, IP_ADDR_ANY, 0);
    if (err != ERR_OK) {
        printf("Failed to bind UDP PCB: %d\n", err);
        return;
    }
    printf("Bound to local port successfully\n");

    state->initialized = true;
    printf("UDP client initialized successfully\n");

    // Send initial test message
    send_udp_data(state, "Client initialized");

    // Main task loop with connection monitoring
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main() {
    stdio_init_all();

    // Wait for USB serial to be ready
    sleep_ms(2000);
    printf("\nUDP Client starting up...\n");

    // Initialize GPIO
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialize I2C with error checking
    printf("Initializing I2C...\n");
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(GPIO_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(GPIO_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(GPIO_SDA_PIN);
    gpio_pull_up(GPIO_SCL_PIN);
    printf("I2C initialized\n");

    // Initialize accelerometer with retry mechanism
    int accel_retries = 3;
    bool accel_initialized = false;
    while (accel_retries-- && !accel_initialized) {
        if (init_accelerometer()) {
            accel_initialized = true;
            printf("Accelerometer initialized successfully\n");
        } else {
            printf("Failed to initialize accelerometer, retries left: %d\n", accel_retries);
            sleep_ms(1000);
        }
    }

    if (!accel_initialized) {
        printf("Failed to initialize accelerometer after all retries\n");
        return 1;
    }

    // Initialize WiFi with detailed error reporting
    printf("Initializing WiFi...\n");
    if (cyw43_arch_init()) {
        printf("Failed to initialize WiFi\n");
        return 1;
    }
    printf("WiFi initialized successfully\n");

    cyw43_arch_enable_sta_mode();
    printf("STA mode enabled\n");

    printf("Connecting to WiFi SSID: %s ...\n", WIFI_SSID);
    int wifi_retries = 3;
    while (wifi_retries--) {
        if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD,
                                             CYW43_AUTH_WPA2_AES_PSK, 30000) == 0) {
            printf("Connected to WiFi successfully!\n");
            break;
        } else {
            printf("Failed to connect to WiFi. Retries left: %d\n", wifi_retries);
            if (wifi_retries > 0) {
                sleep_ms(1000);
            }
        }
    }

    if (wifi_retries < 0) {
        printf("Failed to connect to WiFi after all retries\n");
        return 1;
    }

    // Print network information with proper IP handling
    ip_addr_t ip;
    ip_addr_copy_from_ip4(ip, *netif_ip4_addr(netif_default));
    printf("Device IP address: %s\n", ipaddr_ntoa(&ip));

    // Initialize client state
    client_state = calloc(1, sizeof(UDP_CLIENT_T));
    if (!client_state) {
        printf("Failed to allocate client state\n");
        return 1;
    }
    printf("Client state allocated\n");

    // Create tasks with error checking
    printf("Creating client tasks...\n");

    BaseType_t task_created;

    task_created = xTaskCreate(client_task, "UDP_Client",
                              configMINIMAL_STACK_SIZE * 4,
                              client_state, tskIDLE_PRIORITY + 1, NULL);
    if (task_created != pdPASS) {
        printf("Failed to create UDP client task\n");
        return 1;
    }
    printf("UDP client task created\n");

    task_created = xTaskCreate(accel_task, "Accel_Task",
                              configMINIMAL_STACK_SIZE * 2,
                              client_state, tskIDLE_PRIORITY + 1,
                              &client_state->accel_task);
    if (task_created != pdPASS) {
        printf("Failed to create accelerometer task\n");
        return 1;
    }
    printf("Accelerometer task created\n");

    printf("Starting FreeRTOS scheduler...\n");

    vTaskStartScheduler();

    while(1) {
        tight_loop_contents();
    }

    return 0;
}
