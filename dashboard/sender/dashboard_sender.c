//
// Created by Edric Chan on 25/11/2024.
//

#include "dashboard_sender.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <lwip/pbuf.h>
#include <lwip/udp.h>
#include <pico/cyw43_arch.h>

#define UDP_PORT 4242

// Helper function to send UDP data with improved error handling
bool send_udp_data(UDP_CLIENT_T* state, const char* data) {
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

    pbuf_free(p);
    return true;
}

bool send_dashboard_distance(UDP_CLIENT_T* state, float distance) {
    char* str[100];
    sprintf(str, "Distance = %.2f", distance);
    return send_udp_data(state, str);
}

bool send_dashboard_speed(UDP_CLIENT_T* state, float speed) {
    char* str[100];
    sprintf(str, "Speed = %.2f", speed);
    return send_udp_data(state, str);
}

bool send_dashboard_duty(UDP_CLIENT_T* state, float left_cycle, float right_cycle) {
    char* str[100];
    sprintf(str, "Duty cycle: L=%.2f, R=%.2f", left_cycle, right_cycle);
    return send_udp_data(state, str);
}

bool send_dashboard_cmd(UDP_CLIENT_T* state, char* direction) {
    char* str[100];
    sprintf(str, "Direction: %s", direction);
    return send_udp_data(state, str);
}
