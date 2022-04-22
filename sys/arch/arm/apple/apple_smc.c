/*	$NetBSD: apple_smc.c,v 1.1 2022/05/10 08:09:57 skrll Exp $	*/
/*	$OpenBSD: apple_smc.c,v 1.11 2022/03/25 15:52:03 kettenis Exp $	*/

/*-
 * Copyright (c) 2022 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Nick Hudson
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Copyright (c) 2021 Mark Kettenis <kettenis@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <sys/param.h>

#include <sys/device.h>
#include <sys/kernel.h>
#include <sys/kmem.h>

#include <dev/fdt/fdtvar.h>

#include <arm/apple/apple_rtkit.h>

//#include <sys/sensors.h>

#if 0
#include <machine/apmvar.h>
#include <machine/bus.h>
#include <machine/fdt.h>

#include <dev/clock_subr.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_gpio.h>
#include <dev/ofw/ofw_misc.h>
#include <dev/ofw/fdt.h>

#include <arm64/dev/aplmbox.h>
#include <arm64/dev/rtkit.h>
#endif


//#include "apm.h"

#define SMALL_KERNEL

extern void (*cpuresetfn)(void);
extern void (*powerdownfn)(void);

/* SMC mailbox endpoint */
#define SMC_EP			32

/* SMC commands */
#define SMC_READ_KEY		0x10
#define SMC_WRITE_KEY		0x11
#define SMC_GET_KEY_BY_INDEX	0x12
#define SMC_GET_KEY_INFO	0x13
#define SMC_GET_SRAM_ADDR	0x17
#define  SMC_SRAM_SIZE		0x4000

#define SMC_DATA		__BITS(63, 32)
#define SMC_WLEN		__BITS(31, 24)
#define SMC_LENGTH		__BITS(23, 16)
#define SMC_ID			__BITS(15, 12)
#define SMC_CMD			__BITS(7, 0)

/* SMC errors */
#define SMC_ERROR_MASK		__BITS(7, 0)
#define SMC_ERROR(d)		__SHIFTOUT((d), SMC_ERROR_MASK)
#define SMC_OK			0x00
#define SMC_KEYNOTFOUND		0x84

/* SMC keys */
#define SMC_KEY(s)	((s[0] << 24) | (s[1] << 16) | (s[2] << 8) | s[3])

struct smc_key_info {
	uint8_t		size;
	uint8_t		type[4];
	uint8_t		flags;
};

/* SMC GPIO commands */
#define SMC_GPIO_CMD_OUTPUT	(0x01 << 24)

/* RTC related constants */
#define RTC_OFFSET_LEN		6
#define SMC_CLKM_LEN		6

#if 0
struct apple_smc_sensor {
	const char	*key;
	const char	*key_type;
	enum sensor_type type;
	int		scale;
	const char	*desc;
	int		flags;
};
#endif

#define APLSMC_BE		__BIT(0)
#define APLSMC_HIDDEN		__BIT(1)

#define APLSMC_MAX_SENSORS	19

struct apple_smc_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	bus_space_handle_t	sc_sram_bsh;

	struct rtkit_state	*sc_rs;
	uint8_t			sc_msgid;
	uint64_t		sc_data;

	kmutex_t                sc_mutex;
	kcondvar_t		sc_cv;
	bool			sc_done;

	void			*sc_ih;

//	struct gpio_controller	sc_gc;

	int			sc_rtc_node;
	struct todr_chip_handle sc_todr;

	int			sc_reboot_node;
#if 0
	struct apple_smc_sensor	*sc_smcsensors[APLSMC_MAX_SENSORS];
//	struct ksensor		sc_sensors[APLSMC_MAX_SENSORS];
	int			sc_nsensors;
//	struct ksensordev	sc_sensordev;
#endif
};

struct apple_smc_gpio_pin {
	u_int		pin_no;
	u_int		pin_flags;
	bool		pin_actlo;
};

struct apple_smc_softc *apple_smc_sc;

#ifndef SMALL_KERNEL

struct apple_smc_sensor apple_smc_sensors[] = {
	{ "ACDI", "ui16", SENSOR_INDICATOR, 1, "power supply" },
	{ "B0RM", "ui16", SENSOR_AMPHOUR, 1000, "remaining battery capacity",
	  APLSMC_BE },
	{ "B0FC", "ui16", SENSOR_AMPHOUR, 1000, "last full battery capacity" },
	{ "B0DC", "ui16", SENSOR_AMPHOUR, 1000, "battery design capacity" },
	{ "B0AV", "ui16", SENSOR_VOLTS_DC, 1000, "battery" },
	{ "B0CT", "ui16", SENSOR_INTEGER, 1, "battery discharge cycles" },
	{ "B0TF", "ui16", SENSOR_INTEGER, 1, "battery time-to-full",
	  APLSMC_HIDDEN },
	{ "B0TE", "ui16", SENSOR_INTEGER, 1, "battery time-to-empty",
	  APLSMC_HIDDEN },
	{ "F0Ac", "flt ", SENSOR_FANRPM, 1, "" },
	{ "ID0R", "flt ", SENSOR_AMPS, 1000000, "input" },
	{ "PDTR", "flt ", SENSOR_WATTS, 1000000, "input" },
	{ "PSTR", "flt ", SENSOR_WATTS, 1000000, "system" },
	{ "TB0T", "flt ", SENSOR_TEMP, 1000000, "battery" },
	{ "TCHP", "flt ", SENSOR_TEMP, 1000000, "charger" },
	{ "TW0P", "flt ", SENSOR_TEMP, 1000000, "wireless" },
	{ "Ts0P", "flt ", SENSOR_TEMP, 1000000, "palm rest" },
	{ "Ts1P", "flt ", SENSOR_TEMP, 1000000, "palm rest" },
	{ "VD0R", "flt ", SENSOR_VOLTS_DC, 1000000, "input" },
};

#endif

void	apple_smc_callback(void *, uint64_t);
int	apple_smc_send_cmd(struct apple_smc_softc *, uint8_t, uint32_t, uint16_t);
int	apple_smc_wait_cmd(struct apple_smc_softc *sc);
int	apple_smc_read_key(struct apple_smc_softc *, uint32_t, void *, size_t);
void	apple_smc_refresh_sensors(void *);
//int	apple_smc_apminfo(struct apm_power_info *);
//void	apple_smc_set_pin(void *, uint32_t *, int);
int	apple_smc_gettime(struct todr_chip_handle *, struct timeval *);
int	apple_smc_settime(struct todr_chip_handle *, struct timeval *);
void	apple_smc_reset(void);
void	apple_smc_powerdown(void);
void	apple_smc_reboot_attachhook(device_t);


static const struct device_compatible_entry compat_data[] = {
	{ .compat = "apple,smc" },
	DEVICE_COMPAT_EOL
};

void
apple_smc_callback(void *arg, uint64_t data)
{
	struct apple_smc_softc * const sc = arg;

	mutex_enter(&sc->sc_mutex);
	sc->sc_data = data;
	sc->sc_done = true;
	cv_signal(&sc->sc_cv);
	mutex_exit(&sc->sc_mutex);
}

int
apple_smc_send_cmd(struct apple_smc_softc *sc, uint8_t cmd, uint32_t key,
    uint16_t len)
{
	uint64_t data =
	    __SHIFTIN(key, SMC_DATA) |
	    __SHIFTIN(cmd, SMC_CMD) |
	    __SHIFTIN(len, SMC_LENGTH) |
	    __SHIFTIN((sc->sc_msgid++ & 0xf), SMC_ID)
	    ;

	return rtkit_send_endpoint(sc->sc_rs, SMC_EP, data);
}

int
apple_smc_wait_cmd(struct apple_smc_softc *sc)
{
	int error;
	if (cold) {
		int timo;

		/* Poll for completion. */
		for (timo = 1000; timo > 0; timo--) {
			error = rtkit_poll(sc->sc_rs);
			if (error == 0)
				return 0;
			delay(10);
		}

		return EWOULDBLOCK;
	}

	mutex_enter(&sc->sc_mutex);
	sc->sc_done = false;
	while (!sc->sc_done) {
		error = cv_timedwait(&sc->sc_cv, &sc->sc_mutex, hz / 10);
		if (error)
			break;
	}
	mutex_exit(&sc->sc_mutex);

	return error;
}

int
apple_smc_read_key(struct apple_smc_softc *sc, uint32_t key, void *data, size_t len)
{
	int error;

	apple_smc_send_cmd(sc, SMC_READ_KEY, key, len);
	error = apple_smc_wait_cmd(sc);
	if (error)
		return error;
	switch (SMC_ERROR(sc->sc_data)) {
	case SMC_OK:
		break;
	case SMC_KEYNOTFOUND:
		return EINVAL;
		break;
	default:
		return EIO;
		break;
	}

	len = MIN(len, (sc->sc_data >> 16) & 0xffff);
	if (len > sizeof(uint32_t)) {
		bus_space_read_region_1(sc->sc_bst, sc->sc_sram_bsh, 0,
		    data, len);
	} else {
		uint32_t tmp = (sc->sc_data >> 32);
		memcpy(data, &tmp, len);
	}

	return 0;
}

static int
apple_smc_write_key(struct apple_smc_softc *sc, uint32_t key, void *data, size_t len)
{
	bus_space_write_region_1(sc->sc_bst, sc->sc_sram_bsh, 0, data, len);
	bus_space_barrier(sc->sc_bst, sc->sc_sram_bsh, 0, len,
	    BUS_SPACE_BARRIER_WRITE);
	apple_smc_send_cmd(sc, SMC_WRITE_KEY, key, len);

	return apple_smc_wait_cmd(sc);
}

#ifndef SMALL_KERNEL

void
apple_smc_refresh_sensors(void *arg)
{
	extern int hw_power;
	struct apple_smc_softc *sc = arg;
	struct apple_smc_sensor *sensor;
	int64_t value;
	uint32_t key;
	int i, error;

	for (i = 0; i < sc->sc_nsensors; i++) {
		sensor = sc->sc_smcsensors[i];
		key = SMC_KEY(sensor->key);

		if (strcmp(sensor->key_type, "ui8 ") == 0) {
			uint8_t ui8;

			error = apple_smc_read_key(sc, key, &ui8, sizeof(ui8));
			value = (int64_t)ui8 * sensor->scale;
		} else if (strcmp(sensor->key_type, "ui16") == 0) {
			uint16_t ui16;

			error = apple_smc_read_key(sc, key, &ui16, sizeof(ui16));
			if (sensor->flags & APLSMC_BE)
				ui16 = betoh16(ui16);
			value = (int64_t)ui16 * sensor->scale;
		} else if (strcmp(sensor->key_type, "flt ") == 0) {
			uint32_t flt;
			int64_t mant;
			int sign, exp;

			error = apple_smc_read_key(sc, key, &flt, sizeof(flt));
			if (sensor->flags & APLSMC_BE)
				flt = betoh32(flt);

			/*
			 * Convert floating-point to integer, trying
			 * to keep as much resolution as possible
			 * given the scaling factor for this sensor.
			 */
			sign = (flt >> 31) ? -1 : 1;
			exp = ((flt >> 23) & 0xff) - 127;
			mant = (flt & 0x7fffff) | 0x800000;
			mant *= sensor->scale;
			if (exp < 23)
				value = sign * (mant >> (23 - exp));
			else
				value = sign * (mant << (exp - 23));
		}

		/* Apple reports temperatures in degC. */
		if (sensor->type == SENSOR_TEMP)
			value += 273150000;

		if (error) {
			sc->sc_sensors[i].flags |= SENSOR_FUNKNOWN;
		} else {
			sc->sc_sensors[i].flags &= ~SENSOR_FUNKNOWN;
			sc->sc_sensors[i].value = value;
		}

		if (strcmp(sensor->key, "ACDI") == 0)
			hw_power = (value > 0);
	}
}

#if NAPM > 0

int
apple_smc_apminfo(struct apm_power_info *info)
{
	struct apple_smc_sensor *sensor;
	struct ksensor *ksensor;
	struct apple_smc_softc *sc = apple_smc_sc;
	int remaining = -1, capacity = -1, i;

	info->battery_state = APM_BATT_UNKNOWN;
	info->ac_state = APM_AC_UNKNOWN;
	info->battery_life = 0;
	info->minutes_left = -1;

	for (i = 0; i < sc->sc_nsensors; i++) {
		sensor = sc->sc_smcsensors[i];
		ksensor = &sc->sc_sensors[i];

		if (ksensor->flags & SENSOR_FUNKNOWN)
			continue;

		if (strcmp(sensor->key, "ACDI") == 0) {
			info->ac_state = ksensor->value ?
				APM_AC_ON : APM_AC_OFF;
		} else if (strcmp(sensor->key, "B0RM") == 0)
			remaining = ksensor->value;
		else if (strcmp(sensor->key, "B0FC") == 0)
			capacity = ksensor->value;
		else if ((strcmp(sensor->key, "B0TE") == 0) &&
			 (ksensor->value != 0xffff))
			info->minutes_left = ksensor->value;
		else if ((strcmp(sensor->key, "B0TF") == 0) &&
			 (ksensor->value != 0xffff)) {
			info->battery_state = APM_BATT_CHARGING;
			info->minutes_left = ksensor->value;
		}
	}

	/* calculate remaining battery if we have sane values */
	if (remaining > -1 && capacity > 0) {
		info->battery_life = ((remaining * 100) / capacity);
		if (info->battery_state != APM_BATT_CHARGING) {
			if (info->battery_life > 50)
				info->battery_state = APM_BATT_HIGH;
			else if (info->battery_life > 25)
				info->battery_state = APM_BATT_LOW;
			else
				info->battery_state = APM_BATT_CRITICAL;
		}
	}

	return 0;
}

#endif
#endif


static void *
apple_smc_gpio_acquire(device_t dev, const void *data, size_t len, int flags)
{
	struct apple_smc_gpio_pin *pin;
	const u_int *gpio = data;

	if (len != 12)
		return NULL;

	const u_int pinno = be32toh(gpio[1]);
	const bool actlo = be32toh(gpio[2]) & 1;

	// XXXNH magic number
	if (pinno >= 256)
		return NULL;

	pin = kmem_alloc(sizeof(*pin), KM_SLEEP);
	pin->pin_no = pinno;
	pin->pin_flags = flags;
	pin->pin_actlo = actlo;

	return pin;
}


static void
apple_smc_gpio_release(device_t dev, void *priv)
{
	struct apple_smc_gpio_pin *pin = priv;

	kmem_free(pin, sizeof(*pin));
}

static void
apple_smc_gpio_write(device_t dev, void *priv, int val, bool raw)
{
	struct apple_smc_softc * const sc = device_private(dev);
	struct apple_smc_gpio_pin *pin = priv;
	const u_int pn = pin->pin_no;
	static const char *digits = "0123456789abcdef";
	uint32_t key = SMC_KEY("gP\0\0");
	uint32_t data;

	key |= __SHIFTIN(digits[__SHIFTOUT(pn, __BITS(3, 0))], __BITS(7, 0));
	key |= __SHIFTIN(digits[__SHIFTOUT(pn, __BITS(7, 4))], __BITS(15, 8));

	if (pin->pin_actlo)
		val = !val;
	data = SMC_GPIO_CMD_OUTPUT | !!val;

	apple_smc_write_key(sc, key, &data, sizeof(data));
}

#if 0
int
apple_smc_gettime(struct todr_chip_handle *handle, struct timeval *tv)
{
	struct apple_smc_softc *sc = handle->cookie;
	uint8_t data[8] = {};
	uint64_t offset, time;
	int error;

	error = nvmem_read_cell(sc->sc_rtc_node, "rtc_offset", &data,
	    RTC_OFFSET_LEN);
	if (error)
		return error;
	offset = lemtoh64(data);

	error = apple_smc_read_key(sc, SMC_KEY("CLKM"), &data, SMC_CLKM_LEN);
	if (error)
		return error;
	time = lemtoh64(data) + offset;

	tv->tv_sec = (time >> 15);
	tv->tv_usec = (((time & 0x7fff) * 1000000) >> 15);
	return 0;
}

int
apple_smc_settime(struct todr_chip_handle *handle, struct timeval *tv)
{
	struct apple_smc_softc *sc = handle->cookie;
	uint8_t data[8] = {};
	uint64_t offset, time;
	int error;

	error = apple_smc_read_key(sc, SMC_KEY("CLKM"), &data, SMC_CLKM_LEN);
	if (error)
		return error;

	time = ((uint64_t)tv->tv_sec << 15);
	time |= ((uint64_t)tv->tv_usec << 15) / 1000000;
	offset = time - lemtoh64(data);

	htolem64(data, offset);
	return nvmem_write_cell(sc->sc_rtc_node, "rtc_offset", &data,
	    RTC_OFFSET_LEN);
}

void
apple_smc_reboot_attachhook(device_t self)
{
	struct apple_smc_softc *sc = (struct apple_smc_softc *)self;
	uint8_t count = 0;

	/* Reset error counters. */
	nvmem_write_cell(sc->sc_reboot_node, "boot_error_count",
	    &count, sizeof(count));
	nvmem_write_cell(sc->sc_reboot_node, "panic_count",
	    &count, sizeof(count));
}

void
apple_smc_reset(void)
{
	struct apple_smc_softc *sc = apple_smc_sc;
	uint32_t key = SMC_KEY("MBSE");
	uint32_t rest = SMC_KEY("rest");
	uint32_t phra = SMC_KEY("phra");
	uint8_t boot_stage = 0;

	apple_smc_write_key(sc, key, &rest, sizeof(rest));
	nvmem_write_cell(sc->sc_reboot_node, "boot_stage",
	    &boot_stage, sizeof(boot_stage));
	apple_smc_write_key(sc, key, &phra, sizeof(phra));
}

void
apple_smc_powerdown(void)
{
	struct apple_smc_softc *sc = apple_smc_sc;
	uint32_t key = SMC_KEY("MBSE");
	uint32_t offw = SMC_KEY("offw");
	uint32_t off1 = SMC_KEY("off1");
	uint8_t boot_stage = 0;
	uint8_t shutdown_flag = 1;

	apple_smc_write_key(sc, key, &offw, sizeof(offw));
	nvmem_write_cell(sc->sc_reboot_node, "boot_stage",
	    &boot_stage, sizeof(boot_stage));
	nvmem_write_cell(sc->sc_reboot_node, "shutdown_flag",
	    &shutdown_flag, sizeof(shutdown_flag));
	apple_smc_write_key(sc, key, &off1, sizeof(off1));
}
#endif

static int
apple_smc_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}


static struct fdtbus_gpio_controller_func apple_smc_gpio_funcs = {
	.acquire = apple_smc_gpio_acquire,
	.release = apple_smc_gpio_release,
//	.read = apple_smc_gpio_read,
	.write = apple_smc_gpio_write
};



static void
apple_smc_attach(device_t parent, device_t self, void *aux)
{
	struct apple_smc_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
//	uint8_t data[SMC_CLKM_LEN];
	bus_addr_t addr;
	bus_size_t size;
	int error;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;
	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}

	mutex_init(&sc->sc_mutex, MUTEX_DEFAULT, IPL_NONE);
	cv_init(&sc->sc_cv, "applesmc");

	sc->sc_rs = rtkit_init(phandle, NULL);
	if (sc->sc_rs == NULL) {
		aprint_error(": can't map mailbox channel\n");
		return;
	}

	error = rtkit_boot(sc->sc_rs);
	if (error) {
		aprint_error(": can't boot firmware\n");
		return;
	}

	error = rtkit_start_endpoint(sc->sc_rs, SMC_EP, apple_smc_callback, sc);
	if (error) {
		aprint_error(": can't start SMC endpoint\n");
		return;
	}

	apple_smc_send_cmd(sc, SMC_GET_SRAM_ADDR, 0, 0);
	error = apple_smc_wait_cmd(sc);
	if (error) {
		aprint_error(": can't get SRAM address\n");
		return;
	}

	if (bus_space_map(sc->sc_bst, sc->sc_data, SMC_SRAM_SIZE, 0,
	    &sc->sc_sram_bsh)) {
		aprint_error(": can't map SRAM\n");
		return;
	}

	aprint_naive("\n");
	aprint_normal(": Apple SMC\n");

	apple_smc_sc = sc;

	const int gpio = of_find_firstchild_byname(phandle, "gpio");
	if (gpio > 0) {
		fdtbus_register_gpio_controller(self, gpio,
		    &apple_smc_gpio_funcs);
	}

#if notyet
	/*
	 * Only provide TODR implementation if the "CLKM" key is
	 * supported by the SMC firmware.
	 */
	error = apple_smc_read_key(sc, SMC_KEY("CLKM"), &data, SMC_CLKM_LEN);
	node = OF_getnodebyname(phandle, "rtc");
	if (node && error == 0) {
		sc->sc_rtc_node = node;
		sc->sc_todr.cookie = sc;
		sc->sc_todr.todr_gettime = apple_smc_gettime;
		sc->sc_todr.todr_settime = apple_smc_settime;
		todr_attach(&sc->sc_todr);
	}

	node = OF_getnodebyname(phandle, "reboot");
	if (node) {
		sc->sc_reboot_node = node;
		cpuresetfn = apple_smc_reset;
		powerdownfn = apple_smc_powerdown;
		config_mountroot(self, apple_smc_reboot_attachhook);
	}

#ifndef SMALL_KERNEL

	for (u_int i = 0; i < nitems(apple_smc_sensors); i++) {
		struct smc_key_info info;

		rtc_offset(sc, SMC_GET_KEY_INFO,
		    SMC_KEY(apple_smc_sensors[i].key), 0);
		error = apple_smc_wait_cmd(sc);
		if (error || SMC_ERROR(sc->sc_data) != SMC_OK)
			continue;

		bus_space_read_region_1(sc->sc_bst, sc->sc_sram_bsh, 0,
		    (uint8_t *)&info, sizeof(info));

		/* Skip if the key type doesn't match. */
		if (memcmp(apple_smc_sensors[i].key_type, info.type,
		    sizeof(info.type)) != 0)
			continue;

		if (sc->sc_nsensors >= APLSMC_MAX_SENSORS) {
			aprint_error_dev(self,
			    "maximum number of sensors exceeded\n");
			break;
		}

		sc->sc_smcsensors[sc->sc_nsensors] = &apple_smc_sensors[i];
		strlcpy(sc->sc_sensors[sc->sc_nsensors].desc,
		    apple_smc_sensors[i].desc, sizeof(sc->sc_sensors[0].desc));
		sc->sc_sensors[sc->sc_nsensors].type = apple_smc_sensors[i].type;
		if (!(apple_smc_sensors[i].flags & APLSMC_HIDDEN)) {
			sensor_attach(&sc->sc_sensordev,
			    &sc->sc_sensors[sc->sc_nsensors]);
		}
		sc->sc_nsensors++;
	}

	apple_smc_refresh_sensors(sc);

	strlcpy(sc->sc_sensordev.xname, sc->sc_dev.dv_xname,
	    sizeof(sc->sc_sensordev.xname));
	sensordev_install(&sc->sc_sensordev);
	sensor_task_register(sc, apple_smc_refresh_sensors, 5);

#if NAPM > 0
	apm_setinfohook(apple_smc_apminfo);
#endif

#endif
#endif
}



CFATTACH_DECL_NEW(apple_rtkitsmc, sizeof(struct apple_smc_softc),
    apple_smc_match, apple_smc_attach, NULL, NULL);
