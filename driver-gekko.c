
		case MINER_MINING_DUPS:
			info->mining_state = MINER_MINING;
			if ((int)info->frequency == 200) {
				//possible terminus reset condition.
				compac_set_frequency(compac, info->frequency);
				compac_send_chain_inactive(compac);
				cgtime(&info->last_frequency_adjust);
			} else {
				//check for reset condition
				if (info->asic_type == BM1384) {
					unsigned char buffer[] = {0x84, 0x00, 0x04, 0x00};
					compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 5);
				}
				cgtime(&info->last_frequency_ping);
			}
			break;
		default:
			break;
	}
	hashes = info->hashes;
	info->hashes -= hashes;
	cgsleep_ms(info->scanhash_ms);

	return hashes;
}

static struct cgpu_info *compac_detect_one(struct libusb_device *dev, struct usb_find_devices *found)
{
	struct cgpu_info *compac;
	struct COMPAC_INFO *info;
	int err, i;
	bool exclude_me = 0;
	uint32_t baudrate = CP210X_DATA_BAUD;
	unsigned int bits = CP210X_BITS_DATA_8 | CP210X_BITS_PARITY_MARK;

	compac = usb_alloc_cgpu(&gekko_drv, 1);

	if (!usb_init(compac, dev, found)) {
		applog(LOG_INFO, "failed usb_init");
		compac = usb_free_cgpu(compac);
		return NULL;
	}

	info = cgcalloc(1, sizeof(struct COMPAC_INFO));
	compac->device_data = (void *)info;

	info->ident = usb_ident(compac);

	if (opt_gekko_gsc_detect || opt_gekko_gsd_detect || opt_gekko_gse_detect || opt_gekko_gsh_detect) {
		exclude_me  = (info->ident == IDENT_BSC && !opt_gekko_gsc_detect);
		exclude_me |= (info->ident == IDENT_GSC && !opt_gekko_gsc_detect);
		exclude_me |= (info->ident == IDENT_BSD && !opt_gekko_gsd_detect);
		exclude_me |= (info->ident == IDENT_GSD && !opt_gekko_gsd_detect);
		exclude_me |= (info->ident == IDENT_BSE && !opt_gekko_gse_detect);
		exclude_me |= (info->ident == IDENT_GSE && !opt_gekko_gse_detect);
		exclude_me |= (info->ident == IDENT_GSH && !opt_gekko_gsh_detect);
	}

	if (opt_gekko_serial != NULL && (strstr(opt_gekko_serial, compac->usbdev->serial_string) == NULL)) {
		exclude_me = true;
	}

	if (exclude_me) {
		usb_uninit(compac);
		free(info);
		compac->device_data = NULL;
		return NULL;
	}

	switch (info->ident) {
		case IDENT_BSC:
		case IDENT_GSC:
		case IDENT_BSD:
		case IDENT_GSD:
		case IDENT_BSE:
		case IDENT_GSE:
			info->asic_type = BM1384;
			info->cores = 55;
			info->max_job_id = 0x1f;
			info->rx_len = 5;
			info->task_len = 64;
			info->tx_len = 4;
			info->healthy = 0.33;

			usb_transfer_data(compac, CP210X_TYPE_OUT, CP210X_REQUEST_IFC_ENABLE, CP210X_VALUE_UART_ENABLE, info->interface, NULL, 0, C_ENABLE_UART);
			usb_transfer_data(compac, CP210X_TYPE_OUT, CP210X_REQUEST_DATA, CP210X_VALUE_DATA, info->interface, NULL, 0, C_SETDATA);
			usb_transfer_data(compac, CP210X_TYPE_OUT, CP210X_REQUEST_BAUD, 0, info->interface, &baudrate, sizeof (baudrate), C_SETBAUD);
			usb_transfer_data(compac, CP210X_TYPE_OUT, CP210X_SET_LINE_CTL, bits, info->interface, NULL, 0, C_SETPARITY);
			break;
		case IDENT_GSH:
			info->asic_type = BM1387;
			info->rx_len = 7;
			info->task_len = 54;
			if (opt_gekko_boost) {
				info->task_len += 96;
			}
			info->cores = 114;
			info->max_job_id = 0x7f;
			info->healthy = 0.75;

			compac_toggle_reset(compac);
			break;
		default:
			quit(1, "%s compac_detect_one() invalid %s ident=%d",
				compac->drv->dname, compac->drv->dname, info->ident);
	}

	info->interface = usb_interface(compac);
	info->mining_state = MINER_INIT;

	applog(LOG_DEBUG, "Using interface %d", info->interface);

	if (!add_cgpu(compac))
		quit(1, "Failed to add_cgpu in compac_detect_one");

	update_usb_stats(compac);

	for (i = 0; i < 8; i++) {
		compac->unique_id[i] = compac->unique_id[i+3];
	}
	compac->unique_id[8] = 0;

	applog(LOG_WARNING, "%s %d: %s (%s)", compac->drv->name, compac->device_id, compac->usbdev->prod_string, compac->unique_id);

	return compac;
}

static void compac_detect(bool __maybe_unused hotplug)
{
	usb_detect(&gekko_drv, compac_detect_one);
}

static bool compac_prepare(struct thr_info *thr)
{
	struct cgpu_info *compac = thr->cgpu;
	struct COMPAC_INFO *info = compac->device_data;
	int i;
	int read_bytes = 1;
	bool miner_ok = true;

	info->thr = thr;
	info->bauddiv = 0x19; // 115200
	//info->bauddiv = 0x0D; // 214286
	//info->bauddiv = 0x07; // 375000

	//Sanity check and abort to prevent miner thread from being created.
	if (info->asic_type == BM1387) {
		unsigned char buffer[] = { 0x58, 0x09, 0x00, 0x1C, 0x00, 0x20, 0x07, 0x00, 0x19 };
		info->bauddiv = 0x01; // 1.5Mbps baud.
		buffer[6] = info->bauddiv;
		compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 8);
		cgsleep_ms(1);
		usb_transfer(compac, FTDI_TYPE_OUT, FTDI_REQUEST_BAUD, (info->bauddiv + 1), (FTDI_INDEX_BAUD_BTS & 0xff00) | info->interface, C_SETBAUD);
		cgsleep_ms(1);

		// Ping Micro
		if (info->asic_type == BM1387) {
			info->vcore = bound(opt_gekko_gsh_vcore, 300, 810);
			info->micro_found = 1;
			if (!compac_micro_send(compac, M1_GET_TEMP, 0x00, 0x00)) {
				info->micro_found = 0;
				applog(LOG_INFO, "%s %d: micro not found : dummy mode", compac->drv->name, compac->device_id);
			} else {
				uint8_t vcc = (info->vcore / 1000.0 - 0.3) / 0.002;
				applog(LOG_INFO, "%s %d: requesting vcore of %dmV (%x)", compac->drv->name, compac->device_id, info->vcore, vcc);
				compac_micro_send(compac, M2_SET_VCORE, 0x00, vcc);   // Default 400mV
			}
		}
	}

	if (info->mining_state == MINER_INIT) {
		if (info->asic_type == BM1387) {
			unsigned char buffer[] = {0x54, 0x05, 0x00, 0x00, 0x00};
			compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 8);
			compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 8);
		} else if (info->asic_type == BM1384) {
			unsigned char buffer[] = {0x84, 0x00, 0x00, 0x00};
			compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 5);
			compac_send(compac, (char *)buffer, sizeof(buffer), 8 * sizeof(buffer) - 5);
		}

		miner_ok = false;
		while (read_bytes) {
			memset(info->rx, 0, info->rx_len);
			usb_read_timeout(compac, (char *)info->rx, info->rx_len, &read_bytes, 50, C_GETRESULTS);
			if (read_bytes > 0 && info->rx[0] == 0x13) {
				dumpbuffer(compac, LOG_INFO, "RX", info->rx, read_bytes);
				miner_ok = true;
			}
		}

		if (!miner_ok) {
			applog(LOG_WARNING, "%s %d: found 0 chip(s)", compac->drv->name, compac->device_id);
			if (info->ident == IDENT_BSD || info->ident == IDENT_GSD) {
				//Don't bother retyring, will just waste resources.
				compac->deven = DEV_DISABLED;
			}
		}
	}

	return true;
}

static void compac_statline(char *buf, size_t bufsiz, struct cgpu_info *compac)
{
	struct COMPAC_INFO *info = compac->device_data;
	if (info->chips == 0) {
		return;
	}
	if (info->asic_type == BM1387) {
		if (info->micro_found) {
			tailsprintf(buf, bufsiz, "BM1387:%i %.2fMHz (%d/%d/%d/%.0fF)", info->chips, info->frequency, info->scanhash_ms, info->task_ms, info->fullscan_ms, info->micro_temp);
		} else {
			if (opt_log_output) {
				tailsprintf(buf, bufsiz, "BM1387:%i %.2fMHz (%d/%d/%d)", info->chips, info->frequency, info->scanhash_ms, info->task_ms, info->fullscan_ms);
			} else {
				tailsprintf(buf, bufsiz, "BM1387:%i %.2fMHz", info->chips, info->frequency);
			}
		}
	} else {
		if (opt_log_output) {
			tailsprintf(buf, bufsiz, "BM1384:%i %.2fMHz (%d/%d/%d)", info->chips, info->frequency, info->scanhash_ms, info->task_ms, info->fullscan_ms);
		} else {
			tailsprintf(buf, bufsiz, "BM1384:%i %.2fMHz", info->chips, info->frequency_requested);
		}
	}
}

static struct api_data *compac_api_stats(struct cgpu_info *compac)
{
	struct COMPAC_INFO *info = compac->device_data;
	struct api_data *root = NULL;

	root = api_add_int(root, "Nonces", &info->nonces, false);
	root = api_add_int(root, "Accepted", &info->accepted, false);

	//root = api_add_temp(root, "Temp", &info->micro_temp, false);

	return root;
}

static void compac_shutdown(struct thr_info *thr)
{
	struct cgpu_info *compac = thr->cgpu;
	struct COMPAC_INFO *info = compac->device_data;
	if (!compac->usbinfo.nodev) {
		if (info->asic_type == BM1387) {
			compac_micro_send(compac, M2_SET_VCORE, 0x00, 0x00);   // 300mV
			compac_toggle_reset(compac);
		} else if (info->asic_type == BM1384 && info->frequency != 100) {
			compac_set_frequency(compac, 100);
		}
	}
	info->mining_state = MINER_SHUTDOWN;
	pthread_join(info->rthr.pth, NULL); // Let thread close.
	pthread_join(info->wthr.pth, NULL); // Let thread close.
	PTH(thr) = 0L;
}

uint64_t bound(uint64_t value, uint64_t lower_bound, uint64_t upper_bound)
{
	if (value < lower_bound)
		return lower_bound;
	if (value > upper_bound)
		return upper_bound;
	return value;
}

void stuff_reverse(unsigned char *dst, unsigned char *src, uint32_t len)
{
	uint32_t i;
	for (i = 0; i < len; i++) {
		dst[i] = src[len - i - 1];
	}
}

void stuff_lsb(unsigned char *dst, uint32_t x)
{
	dst[0] = (x >>  0) & 0xff;
	dst[1] = (x >>  8) & 0xff;
	dst[2] = (x >> 16) & 0xff;
	dst[3] = (x >> 24) & 0xff;
}

void stuff_msb(unsigned char *dst, uint32_t x)
{
	dst[0] = (x >> 24) & 0xff;
	dst[1] = (x >> 16) & 0xff;
	dst[2] = (x >>  8) & 0xff;
	dst[3] = (x >>  0) & 0xff;
}

struct device_drv gekko_drv = {
    .drv_id              = DRIVER_gekko,
    .dname               = "GekkoScience",
    .name                = "GSX",
    .hash_work           = hash_queued_work,
    .get_api_stats       = compac_api_stats,
    .get_statline_before = compac_statline,
    .drv_detect          = compac_detect,
    .scanwork            = compac_scanwork,
    .flush_work          = compac_flush_work,
    .update_work         = compac_update_work,
    .thread_prepare      = compac_prepare,
    .thread_init         = compac_init,
    .thread_shutdown     = compac_shutdown,
};
