/*
 * gps.h
 *
 *  Created on: Nov 15, 2019
 *      Author: Bulanov Konstantin
 */

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#ifndef GPS_H_
#define GPS_H_

#define GPS_DEBUG             0
#define GPS_BUF_SIZE        128                             // GPS parser buffer size
#define GPS_MAX_SENTENCE     10                             // Max number of sentences in GPS message
#define RX_BUF_SIZE         GPS_MAX_SENTENCE*GPS_BUF_SIZE   // UART receive buffer size


typedef struct {
    UART_HandleTypeDef *uart;

    // calculated values
    float dec_longitude;
    float dec_latitude;
    float altitude_ft;

    // GGA - Global Positioning System Fixed Data
    float nmea_longitude;
    float nmea_latitude;
    float utc_time;
    char ns, ew;
    int lock;
    int satelites;
    float hdop;
    float msl_altitude;
    char msl_units;

    // RMC - Recommended Minimmum Specific GNS Data
    char rmc_status;
    float speed_k;
    float course_d;
    int date;

    // GLL
    char gll_status;
    bool has_fix_once;

    // VTG - Course over ground, ground speed
    float course_t; // ground speed true
    char course_t_unit;
    float course_m; // magnetic
    char course_m_unit;
    char speed_k_unit;
    float speed_km; // speek km/hr
    char speed_km_unit;
} GPS_t;




extern GPS_t    GPS;
extern uint8_t  rx_buffer[RX_BUF_SIZE];




#if (GPS_DEBUG == 1)
void GPS_print(char *data);
#endif

void    GPS_Init(GPS_t *GPS, UART_HandleTypeDef *uart);
void    GPS_UART_Receive(GPS_t *GPS);
void    GPS_USBPrint();
void    GPS_UART_CallBack(GPS_t *GPS);
int     GPS_validate(char *nmeastr);
void    GPS_parse(GPS_t *GPS, char *GPSstrParse);
void    GPS_parse_GLL(GPS_t *GPS, char **elems);
void    GPS_parse_GGA(GPS_t *GPS, char **elems);
void    GPS_parse_VTG(GPS_t *GPS, char **elems);
float   GPS_nmea_to_dec(float deg_coord, char nsew);

#endif /* GPS_H_ */