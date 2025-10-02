/* SPDX-License-Identifier: GPL-2.0-only */

#include <stdio.h>

#include "ip804ar.h"
#include "poemgr.h"

#define C200P_NUM_PORTS 4
#define C200P_NUM_PSE_CHIPS 1
#define C200P_NUM_PSE_CHIP_IDX 0
#define PSX10_PSE_PORTMASK 0xff      /*?????????????? */
#define C200P_PSE_I2C_ADDR 0x74 // Default I2C address for IP804AR

#define C200P_OWN_POWER_BUDGET (50U)
#define C200P_POE_VOLTAGE (54UL)

static int poemgr_ar804ar_init_chip(struct poemgr_ctx *ctx) {
	struct poemgr_pse_chip *psechip = poemgr_profile_pse_chip_get(ctx->profile, C200P_NUM_PSE_CHIP_IDX);

	return ip804ar_init(psechip, 0, C200P_PSE_I2C_ADDR, C200P_PSE_PORTMASK);
}

static int poemgr_c200p_ready(struct poemgr_ctx *ctx) {
	int ret = -1;
	struct poemgr_pse_chip *psechip = poemgr_profile_pse_chip_get(ctx->profile, C200P_NUM_PSE_CHIP_IDX);

	ret = ip804ar_device_online(psechip);
	if (ret != 0) {
		perror("ip804ar_device_online");
		ret = 0;
		goto out;
	}
	ret = 1;
out:
	return ret;
}

static int poemgr_c200p_enable_chip(struct poemgr_ctx *ctx) {
	int ret;
	struct poemgr_pse_chip *psechip = poemgr_profile_pse_chip_get(ctx->profile, C200P_NUM_PSE_CHIP_IDX);

	ret = ip804ar_device_enable_set(psechip, 1);
	if (ret < 0)
		goto out;

	for (int i = 0; i < C200P_NUM_PORTS; i++) {
		ret = ip804ar_port_enable_set(psechip, i, 1);
		if (ret < 0)
			goto out;
	}

out:
	return ret;
}

static int poemgr_c200p_disable_chip(struct poemgr_ctx *ctx) {
	int ret = 0;
	struct poemgr_pse_chip *psechip = poemgr_profile_pse_chip_get(ctx->profile, C200P_NUM_PSE_CHIP_IDX);
	ret = ip804ar_device_enable_set(psechip, 0);
	for (int i = 0; i < C200P_NUM_PORTS; i++) {
		ret = ip804ar_port_enable_set(psechip, i, 0);
		if (ret < 0)
			goto out;
	}
out:
	return ret;
}

static int poemgr_c200p_update_port_status(struct poemgr_ctx *ctx, int port) {
	struct poemgr_pse_chip *psechip = poemgr_profile_pse_chip_get(ctx->profile, C200P_NUM_PSE_CHIP_IDX);
	struct poemgr_port_status *port_status = &ctx->ports[port].status;

	port_status->power = ip804ar_port_power_consumption_get(psechip, port);
	port_status->active = ip804ar_port_good_get(psechip, port);
	port_status->power_limit = ip804ar_port_power_limit_get(psechip, port);
	port_status->enabled = ip804ar_port_enable_get(psechip, port);
	port_status->faults = ip804ar_port_faults_get(psechip, port);
	port_status->poe_class = ip804ar_port_poe_class_get(psechip, port);
	if (port_status->poe_class > 5) {                                  /*?????????????????????????????????*/
		port_status->poe_class = 0;
		switch(port_status->poe_class) {
			case 5:
				port_status->faults |= POEMGR_FAULT_TYPE_CLASSIFICATION_ERROR;
				break;
			default:
				break;
		}
	}

	return 0;
}

static int poemgr_c200p_apply_config(struct poemgr_ctx *ctx) {
	struct poemgr_pse_chip *psechip = poemgr_profile_pse_chip_get(ctx->profile, C200P_NUM_PSE_CHIP_IDX);
	struct poemgr_port_settings *port_settings;
	int port_settings_available;
	int power_budget;
	int port_opmode;
	int ret = 0;

	power_budget = ctx->settings.power_budget ? ctx->settings.power_budget : C200P_OWN_POWER_BUDGET;

	ret = ip804AR_power_budget_set(psechip, power_budget);
	if (ret < 0)
		goto out;

	for (int i = 0; i < C200P_NUM_PORTS; i++) {
		port_settings = &ctx->ports[i].settings;
		port_settings_available = !!port_settings->name;

		port_opmode = 1;
		if (port_settings->disabled || !port_settings_available)
			port_opmode = 0;
		ret = ip804ar_port_enable_set(psechip, i, port_opmode);
		if (ret < 0)
			goto out;
	}
out:
	return ret;
}

static int poemgr_c200p_update_output_status(struct poemgr_ctx *ctx) {
	struct poemgr_pse_chip *psechip = poemgr_profile_pse_chip_get(ctx->profile, C200P_NUM_PSE_CHIP_IDX);
	int power_budget;

	power_budget = ctx->settings.power_budget ? ctx->settings.power_budget : C200P_OWN_POWER_BUDGET;

	ctx->output_status.power_budget = power_budget * 1000; /*?????????????????????????*/
	ctx->output_status.type = POEMGR_POE_TYPE_AT;

	ip804ar_port_clear_faults(psechip, C200P_NUM_PORTS);
	return 0;
}

static int poemgr_c200p_update_input_status(struct poemgr_ctx *ctx) {
	return 0;
}

static int poemgr_c200p_export_port_metric(struct poemgr_ctx *ctx, int port , struct poemgr_metric *output, int metric)
{
	struct poemgr_pse_chip *psechip = poemgr_profile_pse_chip_get(ctx->profile, C200P_NUM_PSE_CHIP_IDX);
	return ip804ar_export_port_metric(psechip, port, output, metric);
}

struct poemgr_profile poemgr_profile_c200p = {
	.name = "c200p",
	.num_ports = C200P_NUM_PORTS,
	.ready = &poemgr_c200p_ready,
	.enable = &poemgr_c200p_enable_chip,
	.disable = &poemgr_c200p_disable_chip,
	.init = &poemgr_c200p_init_chip,
	.apply_config = &poemgr_c200p_apply_config,
	.update_port_status = &poemgr_c200p_update_port_status,
	.export_port_metric = &poemgr_c200p_export_port_metric,
	.update_output_status = &poemgr_c200p_update_output_status,
	.update_input_status = &poemgr_c200p_update_input_status,
	.num_pse_chips = C200P_NUM_PSE_CHIPS,
};
