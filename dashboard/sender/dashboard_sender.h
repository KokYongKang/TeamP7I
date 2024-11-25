//
// Created by Edric Chan on 25/11/2024.
//

#ifndef DASHBOARD_SENDER_H
#define DASHBOARD_SENDER_H
#include <FreeRTOS.h>
#include <task.h>
#include <lwip/ip_addr.h>

typedef struct {
    struct udp_pcb *udp_pcb;
    ip_addr_t server_addr;
    bool initialized;
    uint8_t buffer[2048];
    size_t buffer_len;
    TaskHandle_t accel_task;
    uint32_t packets_sent;
} UDP_CLIENT_T;

bool send_dashboard_distance(UDP_CLIENT_T* state, float distance);

bool send_dashboard_speed(UDP_CLIENT_T* state, float speed);

bool send_dashboard_duty(UDP_CLIENT_T* state, float left_cycle, float right_cycle);

bool send_dashboard_cmd(UDP_CLIENT_T* state, char* direction);
#endif //DASHBOARD_SENDER_H
