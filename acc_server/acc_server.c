#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lwip/udp.h"
#include "lwip/ip4.h"
#include "hardware/gpio.h"

#include "../motor/motor.h"
#include "../encoder/encoder.h"
#include "../ultrasonic/ultrasonic.h"

#define UDP_PORT 4242
#define LED_PIN 5

// Task configurations
#define TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 8)
#define MONITOR_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define CLIENT_TIMEOUT_MS 10000  // Increased timeout

typedef struct {
    struct udp_pcb *udp_pcb;
    ip_addr_t client_addr;
    u16_t client_port;
    bool has_client;
    uint8_t buffer[2048];
    size_t buffer_len;
    TickType_t last_received;
    uint32_t messages_received;
    uint32_t acks_sent;
} UDP_SERVER_T;

static UDP_SERVER_T* server_state;

// Function to display and execute received command
static void execute_command(const char* command) {
    if (!command || strlen(command) == 0) return;

    uint32_t time_ms = to_ms_since_boot(get_absolute_time());
    printf("\n----------------------------------------\n");
    printf("Time: %lu ms\n", time_ms);

    char cmd_type[32];
    float speed = 0;

    if (sscanf(command, "%s %f", cmd_type, &speed) >= 1) {
        if (strcmp(cmd_type, "forward") == 0) {
            printf("↑ FORWARD %.0f%%\n", speed);
            car_speed(speed * 40 / 100.0f); // Scale speed to motor's range (max 40cm/s)
            car_control(FORWARD);
        } else if (strcmp(cmd_type, "backward") == 0) {
            printf("↓ BACKWARD %.0f%%\n", speed);
            car_speed(speed * 40 / 100.0f);
            car_control(REVERSE);
        } else if (strcmp(cmd_type, "left") == 0) {
            printf("← LEFT %.0f%%\n", speed);
            car_speed(speed * 40 / 100.0f);
            car_control(TURN_LEFT);
        } else if (strcmp(cmd_type, "right") == 0) {
            printf("→ RIGHT %.0f%%\n", speed);
            car_speed(speed * 40 / 100.0f);
            car_control(TURN_RIGHT);
        } else if (strcmp(cmd_type, "stop") == 0) {
            printf("■ STOP\n");
            car_control(STOP);
        } else {
            printf("Unknown Command: %s\n", command);
        }
    } else {
        printf("Raw Data: %s\n", command);
    }

    server_state->messages_received++;
    printf("Total messages received: %lu\n", server_state->messages_received);

    // Flash LED for visual feedback
    gpio_put(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_put(LED_PIN, 0);
}

// UDP Server receive callback with detailed logging
static void udp_server_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    UDP_SERVER_T* state = (UDP_SERVER_T*)arg;

    if (!p) {
        printf("Null pbuf received\n");
        return;
    }

    printf("\nPacket received from %s:%d (size: %d bytes)\n", ipaddr_ntoa(addr), port, p->tot_len);

    if (p->tot_len > 0) {
        if (!state->has_client || !ip_addr_cmp(&state->client_addr, addr) || state->client_port != port) {
            ip_addr_copy(state->client_addr, *addr);
            state->client_port = port;
            state->has_client = true;
            printf("\n+++ New client connected from %s:%d +++\n", ipaddr_ntoa(&state->client_addr), state->client_port);
        }

        state->last_received = xTaskGetTickCount();

        if (p->tot_len < sizeof(state->buffer)) {
            memcpy(state->buffer, p->payload, p->tot_len);
            state->buffer[p->tot_len] = '\0';
            printf("Raw data received: '%.*s'\n", p->tot_len, (char*)p->payload);
            execute_command((char*)state->buffer);
        } else {
            printf("Received data too large for buffer (%d bytes)\n", p->tot_len);
        }
    }
    pbuf_free(p);
    printf("Packet buffer freed\n");
}

// Monitor task to display statistics and check connection
static void monitor_task(void *params) {
    UDP_SERVER_T* state = (UDP_SERVER_T*)params;

    printf("Monitor task started\n");

    while (1) {
        TickType_t now = xTaskGetTickCount();

        if (state->has_client && (now - state->last_received) > pdMS_TO_TICKS(CLIENT_TIMEOUT_MS)) {
            printf("\n!!! Client timeout - no data for %d ms !!!\n", CLIENT_TIMEOUT_MS);
            printf("Last known client: %s:%d\n", ipaddr_ntoa(&state->client_addr), state->client_port);
            state->has_client = false;
            state->messages_received = 0;
            state->acks_sent = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Main server task to set up the UDP listener
static void server_task(void *params) {
    UDP_SERVER_T* state = (UDP_SERVER_T*)params;

    printf("\nStarting UDP server...\n");

    ip_addr_t ip;
    ip_addr_copy_from_ip4(ip, *netif_ip4_addr(netif_default));
    if (ip_addr_isany(&ip)) {
        printf("No valid IP address. Check WiFi connection\n");
        return;
    }
    printf("Server IP address: %s\n", ip4addr_ntoa(&ip));

    state->udp_pcb = udp_new();
    if (!state->udp_pcb) {
        printf("Failed to create PCB\n");
        return;
    }
    printf("UDP PCB created successfully\n");

    err_t err = udp_bind(state->udp_pcb, IP_ADDR_ANY, UDP_PORT);
    if (err != ERR_OK) {
        printf("Failed to bind to port %d (error %d)\n", UDP_PORT, err);
        return;
    }
    printf("Bound to port %d successfully\n", UDP_PORT);

    udp_recv(state->udp_pcb, udp_server_recv, state);
    printf("Receive callback set up\n");

    printf("\nServer ready - listening on port %d\n", UDP_PORT);
    printf("IP Address: %s\n", ip4addr_ntoa(&ip));

    while (1) {
        cyw43_arch_poll();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int main() {
    stdio_init_all();

    // Initialize components
    motor_init();
    encoder_init();
    ultrasonic_init();

    sleep_ms(2000);
    printf("\nUDP Server starting up...\n");

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    printf("Initializing WiFi...\n");
    if (cyw43_arch_init()) {
        printf("Failed to initialize WiFi\n");
        return 1;
    }
    cyw43_arch_enable_sta_mode();
    printf("STA mode enabled\n");

    printf("Connecting to WiFi SSID: %s ...\n", WIFI_SSID);
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Failed to connect to WiFi\n");
        return 1;
    }
    printf("Connected to WiFi\n");

    server_state = calloc(1, sizeof(UDP_SERVER_T));
    if (!server_state) {
        printf("Failed to allocate server state\n");
        return 1;
    }

    xTaskCreate(server_task, "UDP_Server", TASK_STACK_SIZE, server_state, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(monitor_task, "Monitor", TASK_STACK_SIZE, server_state, MONITOR_TASK_PRIORITY, NULL);

    vTaskStartScheduler();

    while(1) {
        tight_loop_contents();
    }
}