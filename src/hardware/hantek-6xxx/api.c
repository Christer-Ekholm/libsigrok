/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2015 Chriter <christerekholm@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <config.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <inttypes.h>
#include <glib.h>
#include <libusb.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "protocol.h"



/* Max time in ms before we want to check on USB events */
/* TODO tune this properly */
#define TICK 10

static const uint32_t scanopts[] = {
	SR_CONF_CONN,
};

static const uint32_t drvopts[] = {
	SR_CONF_OSCILLOSCOPE,
};

static const uint32_t devopts[] = {
	SR_CONF_CONTINUOUS,
	SR_CONF_CONN             | SR_CONF_GET,
	SR_CONF_SAMPLERATE       | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_NUM_VDIV         | SR_CONF_GET,
	SR_CONF_LIMIT_SAMPLES    | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_LIMIT_MSEC       | SR_CONF_GET | SR_CONF_SET,

};

static const uint32_t devopts_cg[] = {
	SR_CONF_VDIV | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
};

static const char *channel_names[] = {
	"CH1", "CH2",
};

static const struct hantek_6xxx_profile dev_profiles[] = {
	{
		0x04b4, 0x6022, 0x04b5, 0x6022,
		"Hantek", "6022be",
		FIRMWARE_DIR "/hantek-6022be.fw"
	},
	{ 0, 0, 0, 0, 0, 0, 0 },
};


static const uint64_t samplerates[] = {
	SAMPLERATE_VALUES
};


static const uint64_t vdivs[][2] = {
	VDIV_VALUES
};

SR_PRIV struct sr_dev_driver hantek_6xxx_driver_info;

static int dev_acquisition_stop(struct sr_dev_inst *sdi, void *cb_data);

static struct sr_dev_inst *hantek_6xxx_dev_new(const struct hantek_6xxx_profile *prof)
{
	struct sr_dev_inst *sdi;
	struct sr_channel *ch;
	struct sr_channel_group *cg;
	struct drv_context *drvc;
	struct dev_context *devc;
	unsigned int i;

	sdi = g_malloc0(sizeof(struct sr_dev_inst));
	sdi->status = SR_ST_INITIALIZING;
	sdi->vendor = g_strdup(prof->vendor);
	sdi->model  = g_strdup(prof->model);
	sdi->driver = &hantek_6xxx_driver_info;

	/*
	 * Add only the real channels -- EXT isn't a source of data, only
	 * a trigger source internal to the device.
	 */
	for (i = 0; i < ARRAY_SIZE(channel_names); i++) {
		ch = sr_channel_new(sdi, i, SR_CHANNEL_ANALOG, TRUE, channel_names[i]);
		cg = g_malloc0(sizeof(struct sr_channel_group));
		cg->name = g_strdup(channel_names[i]);
		cg->channels = g_slist_append(cg->channels, ch);
		sdi->channel_groups = g_slist_append(sdi->channel_groups, cg);
	}

	devc = g_malloc0(sizeof(struct dev_context));

	devc->profile       = prof;
	devc->dev_state     = IDLE;
	devc->samplerate    = DEFAULT_SAMPLERATE;
	devc->ch_enabled[0] = TRUE;
	devc->ch_enabled[1] = TRUE;
	devc->voltage[0]    = DEFAULT_VOLTAGE;
	devc->voltage[1]    = DEFAULT_VOLTAGE;

	sdi->priv           = devc;
	drvc                = sdi->driver->context;
	drvc->instances     = g_slist_append(drvc->instances, sdi);

	return sdi;
}


static int configure_channels(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	const GSList *l;
	int p;
	struct sr_channel *ch;
	struct sr_channel *chans[2];
	devc = sdi->priv;

	g_slist_free(devc->enabled_channels);
	devc->enabled_channels = NULL;
	devc->ch_enabled[0] = devc->ch_enabled[1] = FALSE;
	for (l = sdi->channels, p = 0; l; l = l->next, p++) {
		ch = l->data;

		if(p <= 1)
		{
			devc->ch_enabled[p] = ch->enabled;
			chans[p] = ch;
		}
	}

	if(devc->ch_enabled[1])
	{
		devc->ch_enabled[0] = 1; // Enable CH1 if CH2 is used.
		devc->enabled_channels = g_slist_append(devc->enabled_channels, chans[0]);
		devc->enabled_channels = g_slist_append(devc->enabled_channels, chans[1]);
	}
	else
	{
		devc->ch_enabled[0] = 1; // Enable CH1 allways
		devc->enabled_channels = g_slist_append(devc->enabled_channels, chans[0]);
	}

	return SR_OK;
}

static void clear_dev_context(void *priv)
{
	struct dev_context *devc;

	devc = priv;
	g_slist_free(devc->enabled_channels);

}

static int dev_clear(const struct sr_dev_driver *di)
{
	return std_dev_clear(di, clear_dev_context);
}

static int init(struct sr_dev_driver *di, struct sr_context *sr_ctx)
{
	return std_init(sr_ctx, di, LOG_PREFIX);
}

static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_dev_inst *sdi;
	struct sr_usb_dev_inst *usb;
	struct sr_config *src;
	const struct hantek_6xxx_profile *prof;
	GSList *l, *devices, *conn_devices;
	struct libusb_device_descriptor des;
	libusb_device **devlist;
	int ret, i, j;
	const char *conn;
	char connection_id[64];

	drvc = di->context;

	devices = 0;

	conn = NULL;
	for (l = options; l; l = l->next) {
		src = l->data;
		if (src->key == SR_CONF_CONN) {
			conn = g_variant_get_string(src->data, NULL);
			break;
		}
	}
	if (conn)
		conn_devices = sr_usb_find(drvc->sr_ctx->libusb_ctx, conn);
	else
		conn_devices = NULL;

	/* Find all Hantek DSO devices and upload firmware to all of them. */
	libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	for (i = 0; devlist[i]; i++) {
		if (conn) {
			usb = NULL;
			for (l = conn_devices; l; l = l->next) {
				usb = l->data;
				if (usb->bus == libusb_get_bus_number(devlist[i])
					&& usb->address == libusb_get_device_address(devlist[i]))
					break;
			}
			if (!l)
				/* This device matched none of the ones that
				 * matched the conn specification. */
				continue;
		}

		if ((ret = libusb_get_device_descriptor(devlist[i], &des))) {
			sr_err("Failed to get device descriptor: %s.",
					libusb_error_name(ret));
			continue;
		}

		usb_get_port_path(devlist[i], connection_id, sizeof(connection_id));

		prof = NULL;
		for (j = 0; dev_profiles[j].orig_vid; j++) {
			if (des.idVendor == dev_profiles[j].orig_vid
				&& des.idProduct == dev_profiles[j].orig_pid) {
				/* Device matches the pre-firmware profile. */
				prof = &dev_profiles[j];
				sr_dbg("Found a %s %s.", prof->vendor, prof->model);
				sdi = hantek_6xxx_dev_new(prof);
				sdi->connection_id = g_strdup(connection_id);
				devices = g_slist_append(devices, sdi);
				devc = sdi->priv;
				if (ezusb_upload_firmware(devlist[i], USB_CONFIGURATION,
						prof->firmware) == SR_OK)
					/* Remember when the firmware on this device was updated */
					devc->fw_updated = g_get_monotonic_time();
				else
					sr_err("Firmware upload failed");
				/* Dummy USB address of 0xff will get overwritten later. */
				sdi->conn = sr_usb_dev_inst_new(
						libusb_get_bus_number(devlist[i]), 0xff, NULL);
				break;
			} else if (des.idVendor == dev_profiles[j].fw_vid
				&& des.idProduct == dev_profiles[j].fw_pid) {
				/* Device matches the post-firmware profile. */
				prof = &dev_profiles[j];
				sr_dbg("Found a %s %s.", prof->vendor, prof->model);
				sdi = hantek_6xxx_dev_new(prof);
				sdi->connection_id = g_strdup(connection_id);
				sdi->status = SR_ST_INACTIVE;
				devices = g_slist_append(devices, sdi);
				sdi->inst_type = SR_INST_USB;
				sdi->conn = sr_usb_dev_inst_new(
						libusb_get_bus_number(devlist[i]),
						libusb_get_device_address(devlist[i]), NULL);
				break;
			}
		}
		if (!prof)
			/* not a supported VID/PID */
			continue;
	}
	libusb_free_device_list(devlist, 1);

	return devices;
}

static GSList *dev_list(const struct sr_dev_driver *di)
{
	return ((struct drv_context *)(di->context))->instances;
}

static int dev_open(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	int64_t timediff_us, timediff_ms;
	int err;

	devc = sdi->priv;
	usb = sdi->conn;

	/*
	 * If the firmware was recently uploaded, wait up to MAX_RENUM_DELAY_MS
	 * for the FX2 to renumerate.
	 */
	err = SR_ERR;
	if (devc->fw_updated > 0) {
		sr_info("Waiting for device to reset.");
		/* Takes >= 300ms for the FX2 to be gone from the USB bus. */
		g_usleep(300 * 1000);
		timediff_ms = 0;
		while (timediff_ms < MAX_RENUM_DELAY_MS) {
			if ((err = hantek_6xxx_open(sdi)) == SR_OK)
				break;
			g_usleep(100 * 1000);
			timediff_us = g_get_monotonic_time() - devc->fw_updated;
			timediff_ms = timediff_us / 1000;
			sr_spew("Waited %" PRIi64 " ms.", timediff_ms);
		}
		if(timediff_ms < MAX_RENUM_DELAY_MS)
			sr_info("Device came back after %"PRIu64" ms.", timediff_ms);
	} else {
		err = hantek_6xxx_open(sdi);
	}

	if (err != SR_OK) {
		sr_err("Unable to open device.");
		return SR_ERR;
	}

	err = libusb_claim_interface(usb->devhdl, USB_INTERFACE);
	if (err != 0) {
		sr_err("Unable to claim interface: %s.",
			   libusb_error_name(err));
		return SR_ERR;
	}

	return SR_OK;
}

static int dev_close(struct sr_dev_inst *sdi)
{
	hantek_6xxx_close(sdi);

	return SR_OK;
}

static int cleanup(const struct sr_dev_driver *di)
{
	return dev_clear(di);
}

static int config_get(uint32_t key, GVariant **data, const struct sr_dev_inst *sdi,
		const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	char str[128];
	const uint64_t *vdiv;
	int ch_idx;

	switch (key) {

	case SR_CONF_NUM_VDIV:
		*data = g_variant_new_int32(ARRAY_SIZE(vdivs));
		break;
	}

	if (!sdi)
		return SR_ERR_ARG;

	devc = sdi->priv;
	if (!cg) {
		switch (key) {

		case SR_CONF_SAMPLERATE:
			*data = g_variant_new_uint64(devc->samplerate);
			break;
		case SR_CONF_LIMIT_MSEC:
			*data = g_variant_new_uint64(devc->limit_msec);
			break;
		case SR_CONF_LIMIT_SAMPLES:
			*data = g_variant_new_uint64(devc->limit_samples);
			break;

		case SR_CONF_CONN:
			if (!sdi->conn)
				return SR_ERR_ARG;
			usb = sdi->conn;
			if (usb->address == 255)
				/* Device still needs to re-enumerate after firmware
				 * upload, so we don't know its (future) address. */
				return SR_ERR;
			snprintf(str, 128, "%d.%d", usb->bus, usb->address);
			*data = g_variant_new_string(str);
			break;

		default:
			return SR_ERR_NA;
		}
	} else {
		if (sdi->channel_groups->data == cg)
			ch_idx = 0;
		else if (sdi->channel_groups->next->data == cg)
			ch_idx = 1;
		else
			return SR_ERR_ARG;
		switch (key) {

		case SR_CONF_VDIV:
			vdiv = vdivs[devc->voltage[ch_idx]];
			*data = g_variant_new("(tt)", vdiv[0], vdiv[1]);
			break;

		}
	}

	return SR_OK;
}

static int config_set(uint32_t key, GVariant *data, const struct sr_dev_inst *sdi,
		const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	uint64_t p, q;
	int tmp_int, ch_idx, ret;
	unsigned int i;

	if (sdi->status != SR_ST_ACTIVE)
		return SR_ERR_DEV_CLOSED;

	ret = SR_OK;
	devc = sdi->priv;
	if (!cg) {
		switch (key) {

		case SR_CONF_SAMPLERATE:
			devc->samplerate = g_variant_get_uint64(data);
			hantek_6xxx_update_samplerate(sdi);
			break;

		case SR_CONF_LIMIT_MSEC:
			devc->limit_msec = g_variant_get_uint64(data);
			break;
		case SR_CONF_LIMIT_SAMPLES:
			devc->limit_samples = g_variant_get_uint64(data);
			break;

		default:
			ret = SR_ERR_NA;
			break;
		}
	} else {
		if (sdi->channel_groups->data == cg)
			ch_idx = 0;
		else if (sdi->channel_groups->next->data == cg)
			ch_idx = 1;
		else
			return SR_ERR_ARG;
		switch (key) {

		case SR_CONF_VDIV:
			g_variant_get(data, "(tt)", &p, &q);
			tmp_int = -1;
			for (i = 0; i < ARRAY_SIZE(vdivs); i++) {
				if (vdivs[i][0] == p && vdivs[i][1] == q) {
					tmp_int = i;
					break;
				}
			}
			if (tmp_int >= 0) {
				devc->voltage[ch_idx] = tmp_int;
				hantek_6xxx_update_vdiv(sdi);
			} else
				ret = SR_ERR_ARG;
			break;

		default:
			ret = SR_ERR_NA;
			break;
		}
	}

	return ret;
}

static int config_list(uint32_t key, GVariant **data, const struct sr_dev_inst *sdi,
		const struct sr_channel_group *cg)
{
	GVariant *tuple, *rational[2];
	GVariantBuilder gvb;
	unsigned int i;
	GVariant *gvar;


	if (key == SR_CONF_SCAN_OPTIONS) {
		*data = g_variant_new_fixed_array(G_VARIANT_TYPE_UINT32,
				scanopts, ARRAY_SIZE(scanopts), sizeof(uint32_t));
		return SR_OK;
	} else if (key == SR_CONF_DEVICE_OPTIONS && !sdi) {
		*data = g_variant_new_fixed_array(G_VARIANT_TYPE_UINT32,
				drvopts, ARRAY_SIZE(drvopts), sizeof(uint32_t));
		return SR_OK;
	}

	if (!sdi)
		return SR_ERR_ARG;

	if (!cg) {
		switch (key) {
		case SR_CONF_DEVICE_OPTIONS:
			*data = g_variant_new_fixed_array(G_VARIANT_TYPE_UINT32,
					devopts, ARRAY_SIZE(devopts), sizeof(uint32_t));
			break;

		case SR_CONF_SAMPLERATE:
			g_variant_builder_init(&gvb, G_VARIANT_TYPE("a{sv}"));
			gvar = g_variant_new_fixed_array(G_VARIANT_TYPE("t"),
					samplerates, ARRAY_SIZE(samplerates), sizeof(uint64_t));
			g_variant_builder_add(&gvb, "{sv}", "samplerates", gvar);
			*data = g_variant_builder_end(&gvb);
			break;

		default:
			return SR_ERR_NA;
		}
	} else {
		switch (key) {
		case SR_CONF_DEVICE_OPTIONS:
			*data = g_variant_new_fixed_array(G_VARIANT_TYPE_UINT32,
					devopts_cg, ARRAY_SIZE(devopts_cg), sizeof(uint32_t));
			break;
		case SR_CONF_VDIV:
			g_variant_builder_init(&gvb, G_VARIANT_TYPE_ARRAY);
			for (i = 0; i < ARRAY_SIZE(vdivs); i++) {
				rational[0] = g_variant_new_uint64(vdivs[i][0]);
				rational[1] = g_variant_new_uint64(vdivs[i][1]);
				tuple = g_variant_new_tuple(rational, 2);
				g_variant_builder_add_value(&gvb, tuple);
			}
			*data = g_variant_builder_end(&gvb);
			break;
		default:
			return SR_ERR_NA;
		}
	}

	return SR_OK;
}

// Minimise data amount for limit_samples and limit_msec limits
static uint32_t data_amount(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;

	uint32_t data_amount  = MAX_PACKET_SIZE;

	int channels = devc->ch_enabled[1] ? 2 : 1;

	data_amount = MIN(devc->samplerate * 2 * channels, MAX_PACKET_SIZE); // Max 2 seconds

	if(devc->limit_samples)
	{
		uint32_t data_left = (devc->limit_samples - devc->samp_received) * channels;

		data_amount = MIN(data_left, data_amount);
	}

	if(devc->limit_msec)
	{
		uint32_t time_left = devc->limit_msec - (g_get_monotonic_time() - devc->aq_started) / 1000;

		uint32_t data_left = devc->samplerate * time_left * channels / 1000;

		data_amount = MIN(data_left, data_amount);
	}

	data_amount = MAX(data_amount, MIN_PACKET_SIZE); // Driver does not handle to small buffers

	sr_spew("data_amount %d", data_amount);

	return data_amount;
}


static void send_chunk(struct sr_dev_inst *sdi, unsigned char *buf,
		int num_samples)
{
	struct sr_datafeed_packet packet;
	struct sr_datafeed_analog analog;
	struct dev_context *devc;
	float ch, range;
	int num_channels, data_offset, i;

	devc = sdi->priv;
	num_channels = (devc->ch_enabled[0] && devc->ch_enabled[1]) ? 2 : 1;
	packet.type = SR_DF_ANALOG;
	packet.payload = &analog;

	analog.channels = devc->enabled_channels;
	analog.num_samples = num_samples;
	analog.mq = SR_MQ_VOLTAGE;
	analog.unit = SR_UNIT_VOLT;
	analog.mqflags = 0;

	analog.data = g_try_malloc(analog.num_samples * sizeof(float) * num_channels);

	if(!analog.data)
	{
		sr_err("Unable to allock memory");
		devc->dev_state = STOPPING;
		return;
	}

	data_offset = 0;
	for (i = 0; i < analog.num_samples; i++) {
		/*
		 * The device always sends data for both channels. If a channel
		 * is disabled, it contains a copy of the enabled channel's
		 * data. However, we only send the requested channels to
		 * the bus.
		 *
		 * Voltage values are encoded as a value 0-255, where the value is a point in the range
		 * represented by the vdiv setting. There are 10 vertical divs,
		 * so e.g. 500mV/div represents 5V peak-to-peak where 0 = -2.5V
		 * and 255 = +2.5V.
		 */

		if (devc->ch_enabled[0]) {

			range = ((float)vdivs[devc->voltage[0]][0] / vdivs[devc->voltage[0]][1]) * VDIV_MULTIPLIER;
			ch = range / 255 * *(buf + i * 2);
			/* Value is centered around 0V. */
			ch -= range / 2;
			analog.data[data_offset++] = ch;
		}
		if (devc->ch_enabled[1]) {
			range = ((float)vdivs[devc->voltage[1]][0] / vdivs[devc->voltage[1]][1]) * VDIV_MULTIPLIER;
			ch = range / 255 * *(buf + i * 2 + 1);
			ch -= range / 2;
			analog.data[data_offset++] = ch;
		}
	}
	sr_session_send(devc->cb_data, &packet);
	g_free(analog.data);
}

/*
 * Called by libusb (as triggered by handle_event()) when a transfer comes in.
 * Only channel data comes in asynchronously, and all transfers for this are
 * queued up beforehand, so this just needs to chuck the incoming data onto
 * the libsigrok session bus.
 */
static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	uint32_t num_samples;

	sdi = transfer->user_data;
	devc = sdi->priv;


	if(devc->dev_state == FLUSH)
	{
		devc->dev_state = NEW_CAPTURE;
		return;
	}

	if(devc->dev_state != CAPTURE)
	{
		return;
	}

	sr_spew("receive_transfer(): status %d received %d bytes.",
		   transfer->status, transfer->actual_length);

	if (transfer->actual_length == 0)
		/* Nothing to send to the bus. */
		return;

	num_samples = transfer->actual_length / 2;

	if(devc->limit_samples)
	{
		num_samples = MIN(devc->limit_samples - devc->samp_received, num_samples);
	}

	devc->samp_received += num_samples;

	// I have feeling that this should be done separate thread because Hantek 6022 is data streamer and cannot handle delays
	send_chunk(sdi, transfer->buffer, num_samples);

	/* Everything in this transfer was either copied to the buffer or
	 * sent to the session bus. */
	g_free(transfer->buffer);
	libusb_free_transfer(transfer);

	devc->dev_state = NEW_CAPTURE;

	if(devc->limit_samples && devc->samp_received >= devc->limit_samples)
	{
		sr_info("Requested number of samples reached, stopping. %" PRIu64 " <= %" PRIu64 , devc->limit_samples, devc->samp_received );
		sdi->driver->dev_acquisition_stop(sdi, NULL);
	}


	if(devc->limit_msec && (g_get_monotonic_time() - devc->aq_started) / 1000 >= devc->limit_msec )
	{
		sr_info("Requested time limit reached, stopping. %d <= %d" , (uint32_t)devc->limit_msec, (uint32_t)(g_get_monotonic_time() - devc->aq_started) / 1000 );
		sdi->driver->dev_acquisition_stop(sdi, NULL);
	}

}

static int handle_event(int fd, int revents, void *cb_data)
{
	const struct sr_dev_inst *sdi;
	struct sr_datafeed_packet packet;
	struct timeval tv;
	struct sr_dev_driver *di;
	struct dev_context *devc;
	struct drv_context *drvc;

	(void)fd;
	(void)revents;

	sdi = cb_data;
	di = sdi->driver;
	drvc = di->context;
	devc = sdi->priv;

	/* Always handle pending libusb events. */
	tv.tv_sec = tv.tv_usec = 0;
	libusb_handle_events_timeout(drvc->sr_ctx->libusb_ctx, &tv);

	if (devc->dev_state == STOPPING) {
		/* We've been told to wind up the acquisition. */
		sr_dbg("Stopping acquisition.");

		hantek_6xxx_stop_data_collecting(sdi);
		/*
		 * TODO: Doesn't really cancel pending transfers so they might
		 * come in after SR_DF_END is sent.
		 */
		usb_source_remove(sdi->session, drvc->sr_ctx);

		packet.type = SR_DF_END;
		packet.payload = NULL;
		sr_session_send(sdi, &packet);

		devc->dev_state = IDLE;

		return TRUE;
	}

	if (devc->dev_state == NEW_CAPTURE)
	{
		sr_dbg("New capture request");
		hantek_6xxx_get_channeldata(sdi, receive_transfer, data_amount(sdi));
		devc->dev_state = CAPTURE;
	}

	return TRUE;
}

static int dev_acquisition_start(const struct sr_dev_inst *sdi, void *cb_data)
{
	struct dev_context *devc;
	struct sr_dev_driver *di = sdi->driver;
	struct drv_context *drvc = di->context;

	if (sdi->status != SR_ST_ACTIVE)
		return SR_ERR_DEV_CLOSED;

	devc = sdi->priv;
	devc->cb_data = cb_data;

	if (configure_channels(sdi) != SR_OK) {
		sr_err("Failed to configure channels.");
		return SR_ERR;
	}

	if (hantek_6xxx_init(sdi) != SR_OK)
		return SR_ERR;

	/* Send header packet to the session bus. */
	std_session_send_df_header(cb_data, LOG_PREFIX);

	devc->samp_received = 0;
	devc->aq_started    = g_get_monotonic_time();
	devc->dev_state     = FLUSH;
	//devc->dev_state     = CAPTURE;

	usb_source_add(sdi->session, drvc->sr_ctx, TICK, handle_event, (void *)sdi);

	hantek_6xxx_start_data_collecting(sdi);

	hantek_6xxx_get_channeldata(sdi, receive_transfer, FLUSH_PACKET_SIZE);

	return SR_OK;
}

static int dev_acquisition_stop(struct sr_dev_inst *sdi, void *cb_data)
{
	struct dev_context *devc;

	(void)cb_data;

	if (sdi->status != SR_ST_ACTIVE)
		return SR_ERR;

	devc = sdi->priv;
	devc->dev_state = STOPPING;

	return SR_OK;
}


SR_PRIV struct sr_dev_driver hantek_6xxx_driver_info = {
	.name = "hantek-6xxx",
	.longname = "Hantek 6xxx",
	.api_version = 1,
	.init = init,
	.cleanup = cleanup,
	.scan = scan,
	.dev_list = dev_list,
	.dev_clear = dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = dev_open,
	.dev_close = dev_close,
	.dev_acquisition_start = dev_acquisition_start,
	.dev_acquisition_stop = dev_acquisition_stop,
	.context = NULL,
};
