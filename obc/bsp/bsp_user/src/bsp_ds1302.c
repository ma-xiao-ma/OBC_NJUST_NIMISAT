/**
 * @file ds1302.c
 * Bit banging serial driver for the Maxim DS1302 RTC
 *
 * @note Data is transmitted LSB first. Write data is
 * sampled on the rising edge of SCLK. Read data is
 * output on the falling edge of SCLK.
 *
 * @author Jeppe Ledet-Pedersen
 * Copyright 2011 GomSpace ApS. All rights reserved.
 */

#include <stdio.h>
#include <stdint.h>
#include <time.h>

#include "FreeRTOS.h"
#include "task.h"
#include "portable.h"

#include "bsp_delay.h"
#include "bsp_ds1302.h"
#include "bsp_pca9665.h"

#include "ctrl_cmd_types.h"
#include "crc.h"
#include "contrl.h"
#include "if_adcs.h"

#include "stm32f4xx.h"

static timestamp_t offset = {0, 0};

static int ds1302_write(uint8_t cmd, uint8_t *in, int inlen);

#define read_pin_sclk()			(GPIO_PORTB_DS1302->IDR&DS1302_BPIN_SCLK)
#define read_pin_ce()			(GPIO_PORTB_DS1302->IDR&DS1302_BPIN_CE)
#define read_pin_io()			(GPIO_PORTG_DS1302->IDR&DS1302_GPIN_IO)

#define gpio_sclk_low()			(GPIO_PORTB_DS1302->BSRRH = DS1302_BPIN_SCLK)
#define gpio_sclk_high()		(GPIO_PORTB_DS1302->BSRRL = DS1302_BPIN_SCLK)
#define gpio_ce_low()			(GPIO_PORTB_DS1302->BSRRH = DS1302_BPIN_CE)
#define gpio_ce_high()			(GPIO_PORTB_DS1302->BSRRL = DS1302_BPIN_CE)
#define gpio_io_low()			(GPIO_PORTG_DS1302->BSRRH = DS1302_GPIN_IO)
#define gpio_io_high()			(GPIO_PORTG_DS1302->BSRRL = DS1302_GPIN_IO)

#define DS1302_BIT_DELAY_US		5

int bsp_InitDS1302(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	uint8_t trickle = DS1302_TRICKLE_TCS | DS1302_TRICKLE_DS_1 | DS1302_TRICKLE_RS_2K;

	RCC_AHB1PeriphClockCmd(RCC_DS1302B_PORT | RCC_DS1302G_PORT, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		//输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		//开漏模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	//上下拉电阻不使能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//IO口最大速度

	GPIO_InitStructure.GPIO_Pin = DS1302_BPIN_SCLK | DS1302_BPIN_CE;
	GPIO_Init(GPIO_PORTB_DS1302, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = DS1302_GPIN_IO;
	GPIO_Init(GPIO_PORTG_DS1302, &GPIO_InitStructure);

	/* Set trickle charge register */
	ds1302_write(DS1302_CMD_CLK_TRICKLE, &trickle, sizeof(trickle));

	/* Resume clock */
	ds1302_clock_resume();

	/* Start in WP mode */
	ds1302_wp_enable();

	return 0;
}

int ds1302_io_output(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		//输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		//开漏模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	//上下拉电阻不使能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//IO口最大速度
	GPIO_InitStructure.GPIO_Pin = DS1302_GPIN_IO;
	GPIO_Init(GPIO_PORTG_DS1302, &GPIO_InitStructure);

	return 0;
}

int ds1302_io_input(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		//input
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	//上下拉电阻不使能
	GPIO_InitStructure.GPIO_Pin = DS1302_GPIN_IO;
	GPIO_Init(GPIO_PORTG_DS1302, &GPIO_InitStructure);

	return 0;
}

/* Convert from BCD values to something sane */
static inline uint8_t bcd_to_sane(uint8_t bcd) {
	return (bcd >> 4) * 10 + (bcd & 0x0F);
}

static inline uint8_t sane_to_bcd(uint8_t sane) {
	return ((sane / 10) << 4) + (sane % 10);
}

static inline uint8_t reverse_bits(uint8_t bits) {
	unsigned int b = bits;
	b = ((b * 0x0802LU & 0x22110LU) | (b * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
	return (uint8_t) b;
}

static void ds1302_enable(void) {
	gpio_ce_high();
}

static void ds1302_disable(void) {
	gpio_ce_low();
}

static void ds1302_sclk_high(void) {
	gpio_sclk_high();
}

static void ds1302_sclk_low(void) {
	gpio_sclk_low();
}

static void ds1302_io_high(void) {
	gpio_io_high();
}

static void ds1302_io_low(void) {
	gpio_io_low();
}

static int ds1302_io_read(void) {
	return (read_pin_io() >> 11);
}

static void ds1302_io_write(int val) {
	if (val)
		ds1302_io_high();
	else 
		ds1302_io_low();
}

static void ds1302_write_byte(uint8_t byte) {
	int i;

	for (i = 0; i < 8; i++) {
		ds1302_io_write(byte & 0x01);
		delay_us(DS1302_BIT_DELAY_US);
		ds1302_sclk_high();
		delay_us(DS1302_BIT_DELAY_US);
		byte = byte >> 1;
		ds1302_sclk_low();
		delay_us(DS1302_BIT_DELAY_US);
	}
}

static int ds1302_write(uint8_t cmd, uint8_t *in, int inlen) {
	int i;

	ds1302_enable();

	/* Send command */
	ds1302_write_byte(cmd | DS1302_CMD_WRITE);

	/* Send data */
	for (i = 0; i < inlen; i++)
		ds1302_write_byte(in[i]);

	ds1302_disable();

	return 0;
}

static uint8_t ds1302_read_byte(void) {
	int i;
	uint8_t byte = 0;

	for (i = 0; i < 8; i++) {
		byte = byte << 1;
		byte |= ds1302_io_read();
		delay_us(DS1302_BIT_DELAY_US);
		ds1302_sclk_high();
		delay_us(DS1302_BIT_DELAY_US);
		ds1302_sclk_low();
		delay_us(DS1302_BIT_DELAY_US);
	}

	return reverse_bits(byte);
}

static int ds1302_read(uint8_t cmd, uint8_t *out, int outlen) {
	int i;

	ds1302_enable();

	/* Send command */
	ds1302_write_byte(cmd | DS1302_CMD_READ);
	
	/* Read data */
	ds1302_io_input();
	for (i = 0; i < outlen; i++)
		out[i] = ds1302_read_byte();
	ds1302_io_output();

	ds1302_disable();

	return 0;
}

int ds1302_clock_halt(void) {
	uint8_t seconds;
	ds1302_read(DS1302_CMD_CLK_SECONDS, &seconds, sizeof(seconds));
	seconds |= DS1302_CH_MASK;
	ds1302_write(DS1302_CMD_CLK_SECONDS, &seconds, sizeof(seconds));
	return 0;
}

int ds1302_clock_resume(void) {
	uint8_t seconds;
	ds1302_read(DS1302_CMD_CLK_SECONDS, &seconds, sizeof(seconds));
	seconds &= ~DS1302_CH_MASK;
	ds1302_write(DS1302_CMD_CLK_SECONDS, &seconds, sizeof(seconds));
	return 0;
}

int ds1302_wp_enable(void) {
	uint8_t cr = DS1302_WP_MASK;
	ds1302_write(DS1302_CMD_CLK_WP, &cr, sizeof(cr));
	return 0;
}

int ds1302_wp_disable(void) {
	uint8_t cr = 0;
	ds1302_write(DS1302_CMD_CLK_WP, &cr, sizeof(cr));
	return 0;
}

uint8_t ds1302_get_seconds(void) {
	uint8_t seconds;
	ds1302_read(DS1302_CMD_CLK_SECONDS, &seconds, sizeof(seconds));
	return bcd_to_sane(seconds & 0x7f);
}

int ds1302_set_seconds(uint8_t seconds) {
	uint8_t ch;
	if (seconds > 59)
		return -1;
	ds1302_read(DS1302_CMD_CLK_SECONDS, &ch, sizeof(ch));
	seconds = sane_to_bcd(seconds & 0x7f) | (ch & 0x80);
	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_CLK_SECONDS, &seconds, sizeof(seconds));
	ds1302_wp_enable();
	return 0;
}

uint8_t ds1302_get_minutes(void) {
	uint8_t minutes;
	ds1302_read(DS1302_CMD_CLK_MINUTES, &minutes, sizeof(minutes));
	return bcd_to_sane(minutes);
}

int ds1302_set_minutes(uint8_t minutes) {
	if (minutes > 59)
		return -1;
	minutes = sane_to_bcd(minutes);
	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_CLK_MINUTES, &minutes, sizeof(minutes));
	ds1302_wp_enable();
	return 0;
}

uint8_t ds1302_get_hour(void) {
	uint8_t hour;
	ds1302_read(DS1302_CMD_CLK_HOUR, &hour, sizeof(hour));
	return bcd_to_sane(hour);
}

int ds1302_set_hour(uint8_t hour) {
	if (hour > 23)
		return -1;
	hour = sane_to_bcd(hour);
	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_CLK_MINUTES, &hour, sizeof(hour));
	ds1302_wp_enable();
	return 0;
}

int ds1302_set_12hr(void) {
	uint8_t hours;
	ds1302_read(DS1302_CMD_CLK_HOUR, &hours, sizeof(hours));
	hours |= DS1302_12HR_MASK;
	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_CLK_HOUR, &hours, sizeof(hours));
	ds1302_wp_enable();
	return 0;
}

int ds1302_set_24hr(void) {
	uint8_t hours;
	ds1302_read(DS1302_CMD_CLK_HOUR, &hours, sizeof(hours));
	hours &= ~DS1302_12HR_MASK;
	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_CLK_HOUR, &hours, sizeof(hours));
	ds1302_wp_enable();
	return 0;
}

uint8_t ds1302_get_date(void) {
	uint8_t date;
	ds1302_read(DS1302_CMD_CLK_MONTH, &date, sizeof(date));
	return bcd_to_sane(date);
}

int ds1302_set_date(uint8_t date) {
	if (date < 1 || date > 31)
		return -1;
	date = sane_to_bcd(date);
	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_CLK_MONTH, &date, sizeof(date));
	ds1302_wp_enable();
	return 0;
}

uint8_t ds1302_get_month(void) {
	uint8_t month;
	ds1302_read(DS1302_CMD_CLK_MONTH, &month, sizeof(month));
	return bcd_to_sane(month);
}

int ds1302_set_month(uint8_t month) {
	if (month < 1 || month > 12)
		return -1;
	month = sane_to_bcd(month);
	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_CLK_MONTH, &month, sizeof(month));
	ds1302_wp_enable();
	return 0;
}

uint8_t ds1302_get_day(void) {
	uint8_t day;
	ds1302_read(DS1302_CMD_CLK_DAY, &day, sizeof(day));
	return bcd_to_sane(day);
}

int ds1302_set_day(uint8_t day) {
	if (day < 1 || day > 7)
		return -1;
	day = sane_to_bcd(day);
	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_CLK_DAY, &day, sizeof(day));
	ds1302_wp_enable();
	return 0;
}

uint8_t ds1302_get_year(void) {
	uint8_t year;
	ds1302_read(DS1302_CMD_CLK_YEAR, &year, sizeof(year));
	return bcd_to_sane(year);
}

int ds1302_set_year(uint8_t year) {
	if (year > 99)
		return -1;
	year = sane_to_bcd(year);
	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_CLK_YEAR, &year, sizeof(year));
	ds1302_wp_enable();
	return 0;
}

int ds1302_clock_write_burst(struct ds1302_clock *clock) {
	uint8_t old_minutes;

	if (!clock)
		return -1;

	/* Read minutes to get CH value */
	old_minutes = ds1302_get_minutes();

	clock->seconds 	= sane_to_bcd(clock->seconds & 0x7f) | (old_minutes & 0x80);
	clock->minutes	= sane_to_bcd(clock->minutes);
	clock->hour 	= sane_to_bcd(clock->hour);
	clock->date 	= sane_to_bcd(clock->date);
	clock->month 	= sane_to_bcd(clock->month);
	clock->day 		= sane_to_bcd(clock->day);
	clock->year 	= sane_to_bcd(clock->year);
	clock->wp		= 0;

	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_CLK_BURST, (uint8_t *) clock, sizeof(*clock));
	ds1302_wp_enable();

	return 0;
}

int ds1302_clock_read_burst(struct ds1302_clock *clock) {
	if (!clock)
		return -1;
	
	ds1302_read(DS1302_CMD_CLK_BURST, (uint8_t *) clock, sizeof(*clock));
	
	clock->seconds 	= bcd_to_sane(clock->seconds & 0x7f);
	clock->minutes 	= bcd_to_sane(clock->minutes);
	clock->hour 	= bcd_to_sane(clock->hour);
	clock->date 	= bcd_to_sane(clock->date);
	clock->month 	= bcd_to_sane(clock->month);
	clock->day 		= bcd_to_sane(clock->day);
	clock->year 	= bcd_to_sane(clock->year);
	clock->wp		= 0;

	return 0;
}

int ds1302_write_ram(uint8_t address, uint8_t *data, int datalen) {
	int i;

	if (address + datalen > DS1302_RAM_BYTES)
		return -1;

	ds1302_wp_disable();
	for (i = 0; i < datalen; i++)
		ds1302_write(address + 2 * i, data + i, sizeof(data[i]));
	ds1302_wp_enable();

	return 0;
}

int ds1302_read_ram(uint8_t address, uint8_t *data, int datalen) {
	int i;

	if (address + datalen > DS1302_RAM_BYTES)
		return -1;

	for (i = 0; i < datalen; i++)
		ds1302_read(address + 2 * i, data + i, sizeof(data[i]));

	return 0;
}

int ds1302_read_ram_burst(uint8_t data[DS1302_RAM_BYTES]) {
	if (!data)
		return -1;

	ds1302_read(DS1302_CMD_RAM_BURST, data, DS1302_RAM_BYTES);
	return 0;
}

int ds1302_write_ram_burst(uint8_t data[DS1302_RAM_BYTES]) {
	if (!data)
		return -1;

	ds1302_wp_disable();
	ds1302_write(DS1302_CMD_RAM_BURST, data, DS1302_RAM_BYTES);
	ds1302_wp_enable();

	return 0;
}

int ds1302_time_to_clock(time_t *time, struct ds1302_clock *clock) {
	struct tm *t;

	t = gmtime(time);

	if (t && clock) {
		clock->seconds 	= t->tm_sec;
		clock->minutes 	= t->tm_min;
		clock->hour 	= t->tm_hour;
		clock->date 	= t->tm_mday;
		clock->month 	= t->tm_mon + 1;
		clock->day 		= t->tm_wday;
		clock->year 	= t->tm_year;
		return 0;
	} else {
		return -1;
	}
}

int ds1302_clock_to_time(time_t *time, struct ds1302_clock *clock) {
	struct tm t;

	if (!time || !clock)
		return -1;

	t.tm_sec = clock->seconds;
	t.tm_min = clock->minutes;
	t.tm_hour = clock->hour;
	t.tm_mday = clock->date;
	t.tm_mon = clock->month - 1;
	t.tm_year = clock->year;
	t.tm_isdst = 0;

	*time = mktime(&t);

	return 0;
}

void clock_get_time(timestamp_t * time) {

    TickType_t clocks = xTaskGetTickCount();

	time->tv_sec = offset.tv_sec + clocks / configTICK_RATE_HZ;
	time->tv_nsec = (offset.tv_nsec + (clocks % configTICK_RATE_HZ) * 1000000) % 1000000000;

}

uint32_t clock_get_time_nopara(void) {

	timestamp_t  time;
	TickType_t clocks = xTaskGetTickCount();

	time.tv_sec = offset.tv_sec + clocks / configTICK_RATE_HZ;
	time.tv_nsec = (offset.tv_nsec + (clocks % configTICK_RATE_HZ) * 1000000) % 1000000000;

	return time.tv_sec;
}

void clock_set_time(timestamp_t * time) {

    TickType_t clocks = xTaskGetTickCount();

	offset.tv_sec = time->tv_sec - clocks / configTICK_RATE_HZ;
	offset.tv_nsec = (time->tv_nsec - (clocks % configTICK_RATE_HZ) * 1000000) % 1000000000;
}

void clock_get_monotonic(timestamp_t * time) {

    TickType_t clocks = xTaskGetTickCount();

	time->tv_sec = clocks / configTICK_RATE_HZ;
	time->tv_nsec = (clocks % configTICK_RATE_HZ) * (1000000000/configTICK_RATE_HZ);
}

void rtc_time_get(timestamp_t * timestamp){
	struct ds1302_clock clock;

	/* Get time from RTC */
	ds1302_clock_read_burst(&clock);
	ds1302_clock_to_time((time_t *) &timestamp->tv_sec, &clock);
	timestamp->tv_nsec = 0;
}

int timesync_nopara(void){
	int result = 0;

	struct ds1302_clock clock;
	timestamp_t timestamp;
	TickType_t tick_count = xTaskGetTickCount();

	/* Get time from RTC */
	ds1302_clock_read_burst(&clock);
	ds1302_clock_to_time((time_t *) &timestamp.tv_sec, &clock);
	timestamp.tv_nsec = 0;

	offset.tv_sec = timestamp.tv_sec - tick_count/configTICK_RATE_HZ;
	offset.tv_nsec = timestamp.tv_nsec;

	result = adcstimesync(timestamp.tv_sec);

	return result;
}

int timesync(time_t sec){
	int result = 0;

	timestamp_t  time;
	struct ds1302_clock clock;

	time.tv_sec = sec;
	time.tv_nsec = 0;

	/* Sync */
	clock_set_time(&time);		//鏍℃椂SYSTIME

	ds1302_time_to_clock(&sec, &clock);
	ds1302_clock_write_burst(&clock);		//鏍℃椂RTCTIME

	result = adcstimesync(time.tv_sec);

	return result;
}

int obc_timesync(void){

	struct ds1302_clock clock;
	timestamp_t timestamp;
	TickType_t tick_count = xTaskGetTickCount();

	/* Get time from RTC */
	ds1302_clock_read_burst(&clock);
	ds1302_clock_to_time((time_t *) &timestamp.tv_sec, &clock);
	timestamp.tv_nsec = 0;

	offset.tv_sec = timestamp.tv_sec - tick_count/configTICK_RATE_HZ;
	offset.tv_nsec = timestamp.tv_nsec;

	return 0;
}

