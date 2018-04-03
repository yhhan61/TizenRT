/****************************************************************************
 * Copyright 2018 DIGNSYS All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
****************************************************************************/

#include "includes.h"
#include "common/wpa_ctrl.h"
#include "utils/common.h"
#include "utils/edit.h"
#include "common/version.h"
#include "common/ieee802_11_defs.h"

#define DS_WPA_CLI_MSG(fmt,...)	printf(fmt, ##__VA_ARGS__);
#define CMD_BUFF_SIZE				(0x4000)
#define WLAN_IFACE_NAME				"wl1"

typedef int (*WPA_CMD_HANDLER)(struct wpa_ctrl *ctrl, int argc, char *argv[]);

struct ds_wpa_cli_command {
	const char *cmd;
	WPA_CMD_HANDLER cmd_handler;
	const char *usage;
};

static void ds_wpa_cli_print_help(const char *cmd);

static void ds_wpa_cli_usage(void)
{
	DS_WPA_CLI_MSG("----------------------------------------------------\n");
	DS_WPA_CLI_MSG("| Raspberry PI Zero W wifi application for cyw43438 |\n");
	DS_WPA_CLI_MSG("----------------------------------------------------\n");
	DS_WPA_CLI_MSG("ds_wpa_cli\n");
	DS_WPA_CLI_MSG("\t-h = help (show this usage text)\n");
	DS_WPA_CLI_MSG("\t-v = shown version information\n");
}

static int ds_wpa_control_command(struct wpa_ctrl *ctrl_conn, char *cmd)
{
	char *buffer;
	size_t length;
	int ret;

	if (ctrl_conn == NULL) {
		DS_WPA_CLI_MSG("Not connected to wpa_supplicant\n");
		return -1;
	}

	buffer = (char *)os_malloc(CMD_BUFF_SIZE);
	length = CMD_BUFF_SIZE - 1;

	DS_WPA_CLI_MSG("%s\n", cmd);

	ret = wpa_ctrl_request(ctrl_conn, cmd, os_strlen(cmd), buffer, &length, NULL);
	if (ret == -2) {
		DS_WPA_CLI_MSG("'%s' command timed out.\n", cmd);
		ret = -2;
		goto err;
	} else if (ret < 0) {
		DS_WPA_CLI_MSG("'%s' command failed.\n", cmd);
		ret = -1;
		goto err;
	}

	buffer[length] = '\0';
	DS_WPA_CLI_MSG("%s", buffer);

	os_free(buffer);

	return 0;

err:
	if (buffer)
		os_free(buffer);

	return -1;
}

static int ds_write_cmd(char *buf, size_t buflen, const char *cmd, int argc, char *argv[])
{
	int i;
	int res;
	char *pos;
	char *end;

	pos = buf;
	end = buf + buflen;

	res = os_snprintf(pos, end - pos, "%s", cmd);
	if (os_snprintf_error(end - pos, res))
		goto fail;
	pos += res;

	for (i = 0; i < argc; i++) {
		res = os_snprintf(pos, end - pos, " %s", argv[i]);
		if (os_snprintf_error(end - pos, res))
			goto fail;
		pos += res;
	}

	buf[buflen - 1] = '\0';
	return 0;

fail:
	DS_WPA_CLI_MSG("Too long command\n");
	return -1;
}


static int ds_wpa_cli_command(struct wpa_ctrl *ctrl, const char *cmd, int min_args, int argc, char *argv[])
{
	char *buffer;
	int ret;

	if (argc < min_args) {
		DS_WPA_CLI_MSG("Invalid %s command - at least %d argument%s required.\n",
				cmd, min_args, min_args > 1 ? "s are" : " is");

		return -1;
	}

	buffer = os_malloc(CMD_BUFF_SIZE);
	if (buffer == NULL)
		return -1;

	if (ds_write_cmd(buffer, CMD_BUFF_SIZE, cmd, argc, argv) < 0) {
		os_free(buffer);
		return -1;
	}

	ret = ds_wpa_control_command(ctrl, buffer);
	os_free(buffer);

	return ret;
}


static int ds_wpa_cli_command_ifname(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	return ds_wpa_control_command(ctrl, "IFNAME");
}

static int ds_wpa_cli_command_status(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	if (argc > 0 && os_strncmp(argv[0], "wps", 3) == 0)
		return ds_wpa_control_command(ctrl, "STATUS-WPS");

	if (argc > 0 && os_strncmp(argv[0], "verbose", 7) == 0)
		return ds_wpa_control_command(ctrl, "STATUS-VERBOSE");

	if (argc > 0 && os_strncmp(argv[0], "driver", 6) == 0)
		return ds_wpa_control_command(ctrl, "STATUS-DRIVER");

	return ds_wpa_control_command(ctrl, "STATUS");
}

static int ds_wpa_cli_command_ping(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	return ds_wpa_control_command(ctrl, "PING");
}


static int wpa_cli_cmd_relog(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	return ds_wpa_control_command(ctrl, "RELOG");
}


static int wpa_cli_cmd_note(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	return ds_wpa_cli_command(ctrl, "NOTE", 1, argc, argv);
}


static int ds_wpa_cli_command_mib(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	return ds_wpa_control_command(ctrl, "MIB");
}

static int ds_wpa_cli_command_dump(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	return ds_wpa_control_command(ctrl, "DUMP");
}

static int ds_wpa_cli_command_pmksa(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	return ds_wpa_control_command(ctrl, "PMKSA");
}


static int ds_wpa_cli_command_pmksa_flush(struct wpa_ctrl *ctrl, int argc,
				   char *argv[])
{
	return ds_wpa_control_command(ctrl, "PMKSA_FLUSH");
}

static int ds_wpa_cli_command_blacklist(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	return ds_wpa_cli_command(ctrl, "BLACKLIST", 0, argc, argv);
}

static int ds_wpa_cli_command_help(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	ds_wpa_cli_print_help(argc > 0 ? argv[0] : NULL);

	return 0;
}

static int ds_wpa_cli_command_level(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	return ds_wpa_cli_command(ctrl, "LEVEL", 1, argc, argv);
}

static int ds_wpa_cli_command_password(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	char *cmd;
	char *pos;
	char *end;
	int i;
	int ret;

	if (argc < 2) {
		DS_WPA_CLI_MSG("Invalid PASSWORD command: needs two arguments(network id and password).\n");

		return -1;
	}

	cmd = (char *)os_malloc(CMD_BUFF_SIZE);
	if (cmd == NULL)
		goto err;

	end = cmd + CMD_BUFF_SIZE;
	pos = cmd;
	ret = os_snprintf(pos, end - pos, WPA_CTRL_RSP "PASSWORD-%s:%s", argv[0], argv[1]);
	if (os_snprintf_error(end - pos, ret)) {
		DS_WPA_CLI_MSG("Too long PASSWORD command.\n");

		goto err;
	}

	pos += ret;
	for (i = 2; i < argc; i++) {
		ret = os_snprintf(pos, end - pos, " %s", argv[i]);
		if (os_snprintf_error(end - pos, ret)) {
			DS_WPA_CLI_MSG("Too long PASSWORD command.\n");

			goto err;
		}
		pos += ret;
	}

	ret = ds_wpa_control_command(ctrl, cmd);
	os_free(cmd);

	return ret;

err:
	if (cmd)
		os_free(cmd);

	return -1;
}

static int ds_wpa_cli_command_new_password(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	char *cmd;
	char *pos;
	char *end;
	int i;
	int ret;

	if (argc < 2) {
		DS_WPA_CLI_MSG("Invalid NEW_PASSWORD command: needs two arguments (network id and password).\n");

		return -1;
	}

	cmd = (char *)os_malloc(CMD_BUFF_SIZE);
	if (cmd == NULL)
		goto err;

	end = cmd + CMD_BUFF_SIZE;
	pos = cmd;
	ret = os_snprintf(pos, end - pos, WPA_CTRL_RSP "NEW_PASSWORD-%s:%s", argv[0], argv[1]);
	if (os_snprintf_error(end - pos, ret)) {
		DS_WPA_CLI_MSG("Too long NEW_PASSWORD command.\n");

		goto err;
	}

	pos += ret;
	for (i = 2; i < argc; i++) {
		ret = os_snprintf(pos, end - pos, " %s", argv[i]);
		if (os_snprintf_error(end - pos, ret)) {
			DS_WPA_CLI_MSG("Too long NEW_PASSWORD command.\n");

			goto err;
		}
		pos += ret;
	}

	ret = ds_wpa_control_command(ctrl, cmd);
	os_free(cmd);

	return ret;

err:
	if (cmd)
		os_free(cmd);

	return -1;
}

static int ds_wpa_cli_command_pin(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	char *cmd;
	char *pos;
	char *end;
	int i;
	int ret;

	if (argc < 2) {
		DS_WPA_CLI_MSG("Invalid PIN command: needs two arguments (network id and pin).\n");

		return -1;
	}

	cmd = (char *)os_malloc(CMD_BUFF_SIZE);
	if (cmd == NULL)
		goto err;

	end = cmd + CMD_BUFF_SIZE;
	pos = cmd;
	ret = os_snprintf(pos, end - pos, WPA_CTRL_RSP "PIN-%s:%s", argv[0], argv[1]);
	if (os_snprintf_error(end - pos, ret)) {
		DS_WPA_CLI_MSG("Too long PIN command.\n");

		goto err;
	}

	pos += ret;
	for (i = 2; i < argc; i++) {
		ret = os_snprintf(pos, end - pos, " %s", argv[i]);
		if (os_snprintf_error(end - pos, ret)) {
			DS_WPA_CLI_MSG("Too long PIN command.\n");

			goto err;
		}
		pos += ret;
	}

	ret = ds_wpa_control_command(ctrl, cmd);
	os_free(cmd);

	return ret;

err:
	if (cmd)
		os_free(cmd);

	return -1;
}

static int ds_wpa_cli_command_otp(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	char *cmd;
	char *pos;
	char *end;
	int i;
	int ret;

	if (argc < 2) {
		DS_WPA_CLI_MSG("Invalid OTP command: needs two arguments (network id and password)\n");

		return -1;
	}

	cmd = (char *)os_malloc(CMD_BUFF_SIZE);
	if (cmd == NULL)
		goto err;

	end = cmd + CMD_BUFF_SIZE;
	pos = cmd;
	ret = os_snprintf(pos, end - pos, WPA_CTRL_RSP "OTP-%s:%s", argv[0], argv[1]);
	if (os_snprintf_error(end - pos, ret)) {
		DS_WPA_CLI_MSG("Too long OTP command.\n");

		goto err;
	}

	pos += ret;
	for (i = 2; i < argc; i++) {
		ret = os_snprintf(pos, end - pos, " %s", argv[i]);
		if (os_snprintf_error(end - pos, ret)) {
			DS_WPA_CLI_MSG("Too long OTP command.\n");

			goto err;
		}
		pos += ret;
	}

	ret = ds_wpa_control_command(ctrl, cmd);
	os_free(cmd);

	return ret;

err:
	if (cmd)
		os_free(cmd);

	return -1;
}


static int ds_wpa_cli_command_passphrase(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	char *cmd;
	char *pos;
	char *end;
	int i;
	int ret;

	if (argc < 2) {
		DS_WPA_CLI_MSG("Invalid PASSPHRASE command: needs two arguments network id and passphrase)\n");

		return -1;
	}

	cmd = (char *)os_malloc(CMD_BUFF_SIZE);
	if (cmd == NULL)
		goto err;

	end = cmd + CMD_BUFF_SIZE;
	pos = cmd;
	ret = os_snprintf(pos, end - pos, WPA_CTRL_RSP "PASSPHRASE-%s:%s", argv[0], argv[1]);
	if (os_snprintf_error(end - pos, ret)) {
		DS_WPA_CLI_MSG("Too long PASSPHRASE command.\n");

		goto err;
	}

	pos += ret;
	for (i = 2; i < argc; i++) {
		ret = os_snprintf(pos, end - pos, " %s", argv[i]);
		if (os_snprintf_error(end - pos, ret)) {
			DS_WPA_CLI_MSG("Too long PASSPHRASE command.\n");

			goto err;
		}
		pos += ret;
	}

	ret = ds_wpa_control_command(ctrl, cmd);
	os_free(cmd);

	return ret;

err:
	if (cmd)
		os_free(cmd);

	return -1;
}

static int ds_wpa_cli_command_sim(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	char *cmd;
	char *pos;
	char *end;
	int i;
	int ret;

	if (argc < 2) {
		DS_WPA_CLI_MSG("Invalid SIM command: needs two arguments network id and SIM operation response)\n");

		return -1;
	}

	cmd = (char *)os_malloc(CMD_BUFF_SIZE);
	if (cmd)
		goto err;

	end = cmd + CMD_BUFF_SIZE;
	pos = cmd;
	ret = os_snprintf(pos, end - pos, WPA_CTRL_RSP "SIM-%s:%s", argv[0], argv[1]);
	if (os_snprintf_error(end - pos, ret)) {
		DS_WPA_CLI_MSG("Too long SIM command.\n");

		goto err;
	}

	pos += ret;
	for (i = 2; i < argc; i++) {
		ret = os_snprintf(pos, end - pos, " %s", argv[i]);
		if (os_snprintf_error(end - pos, ret)) {
			DS_WPA_CLI_MSG("Too long SIM command.\n");

			goto err;
		}
		pos += ret;
	}

	ret = ds_wpa_control_command(ctrl, cmd);
	os_free(cmd);

	return ret;

err:
	if (cmd)
		os_free(cmd);

	return -1;
}

static int ds_wpa_cli_command_bssid(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	if (argc < 2) {
		DS_WPA_CLI_MSG("Invalid BSSID command: needs two arguments (network id and BSSID)\n");
		return -1;
	}

	return ds_wpa_cli_command(ctrl, "BSSID", 2, argc, argv);
}


static int ds_wpa_cli_command_log_level(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	return ds_wpa_cli_command(ctrl, "LOG_LEVEL", 0, argc, argv);
}


static int ds_wpa_cli_command_list_networks(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	return ds_wpa_control_command(ctrl, "LIST_NETWORKS");
}


static int ds_wpa_cli_command_select_network(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	return ds_wpa_cli_command(ctrl, "SELECT_NETWORK", 1, argc, argv);
}


static int ds_wpa_cli_command_enable_network(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	return ds_wpa_cli_command(ctrl, "ENABLE_NETWORK", 1, argc, argv);
}


static int ds_wpa_cli_command_disable_network(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	return ds_wpa_cli_command(ctrl, "DISABLE_NETWORK", 1, argc, argv);
}


static int ds_wpa_cli_command_add_network(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	return ds_wpa_control_command(ctrl, "ADD_NETWORK");
}


static int ds_wpa_cli_command_remove_network(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	return ds_wpa_cli_command(ctrl, "REMOVE_NETWORK", 1, argc, argv);
}


static void wpa_cli_show_network_variables(void)
{
	DS_WPA_CLI_MSG("set_network variables:\n"
	       "  ssid (network name, SSID)\n"
	       "  psk (WPA passphrase or pre-shared key)\n"
	       "  key_mgmt (key management protocol)\n"
	       "  identity (EAP identity)\n"
	       "  password (EAP password)\n"
	       "  ...\n"
	       "\n"
	       "Note: Values are entered in the same format as the "
	       "configuration file is using,\n"
	       "i.e., strings values need to be inside double quotation "
	       "marks.\n"
	       "For example: set_network 0 ssid \\\"network name\\\"\n"
	       "\n"
	       "Please see wpa_supplicant.conf documentation for full list "
	       "of\navailable variables.\n");
}

static int ds_wpa_cli_command_set_network(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	if (argc == 0) {
		wpa_cli_show_network_variables();
		return 0;
	}

	if (argc < 3) {
		DS_WPA_CLI_MSG("Invalid SET_NETWORK command: needs three arguments.\n"
		       "network id, variable name, and value.\n");

		return -1;
	}

	return ds_wpa_cli_command(ctrl, "SET_NETWORK", 3, argc, argv);
}


static int ds_wpa_cli_command_get_network(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	if (argc == 0) {
		wpa_cli_show_network_variables();
		return 0;
	}

	if (argc != 2) {
		DS_WPA_CLI_MSG("Invalid GET_NETWORK command: needs two arguments\n"
		       "(network id and variable name)\n");
		return -1;
	}

	return ds_wpa_cli_command(ctrl, "GET_NETWORK", 2, argc, argv);
}


static int ds_wpa_cli_command_dup_network(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	if (argc == 0) {
		wpa_cli_show_network_variables();
		return 0;
	}

	if (argc < 3) {
		DS_WPA_CLI_MSG("Invalid DUP_NETWORK command: needs three arguments\n"
		       "(src netid, dest netid, and variable name)\n");
		return -1;
	}

	return ds_wpa_cli_command(ctrl, "DUP_NETWORK", 3, argc, argv);
}


static int ds_wpa_cli_command_scan(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	return ds_wpa_cli_command(ctrl, "SCAN", 0, argc, argv);
}


static int ds_wpa_cli_command_scan_results(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	return ds_wpa_control_command(ctrl, "SCAN_RESULTS");
}

static int ds_wpa_cli_command_identity(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	char *cmd;
	char *pos;
	char *end;
	int i;
	int ret;

	if (argc < 2) {
		DS_WPA_CLI_MSG("Invalid IDENTITY command: needs two arguments (network id and identity)\n");

		return -1;
	}

	cmd = (char *)os_malloc(CMD_BUFF_SIZE);
	if (cmd == NULL)
		goto err;

	end = cmd + CMD_BUFF_SIZE;
	pos = cmd;
	ret = os_snprintf(pos, end - pos, WPA_CTRL_RSP "IDENTITY-%s:%s", argv[0], argv[1]);
	if (os_snprintf_error(end - pos, ret)) {
		DS_WPA_CLI_MSG("Too long IDENTITY command.\n");

		goto err;
	}

	pos += ret;
	for (i = 2; i < argc; i++) {
		ret = os_snprintf(pos, end - pos, " %s", argv[i]);
		if (os_snprintf_error(end - pos, ret)) {
			DS_WPA_CLI_MSG("Too long IDENTITY command.\n");

			goto err;
		}
		pos += ret;
	}

	ret = ds_wpa_control_command(ctrl, cmd);
	os_free(cmd);

	return ret;

err:
	if (cmd)
		os_free(cmd);

	return -1;
}

static int ds_wpa_cli_command_bss(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	return ds_wpa_cli_command(ctrl, "BSS", 1, argc, argv);
}


static int ds_wpa_cli_command_terminate(struct wpa_ctrl *ctrl, int argc,
				 char *argv[])
{
	return ds_wpa_control_command(ctrl, "TERMINATE");
}


static int ds_wpa_cli_command_suspend(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	return ds_wpa_control_command(ctrl, "SUSPEND");
}

static int ds_wpa_cli_command_resume(struct wpa_ctrl *ctrl, int argc, char *argv[])
{
	return ds_wpa_control_command(ctrl, "RESUME");
}

static struct ds_wpa_cli_command wpa_cli_commands[] = {
	{ "ping",            ds_wpa_cli_command_ping,             "= pings wpa_supplicant" },
	{ "level",           ds_wpa_cli_command_level,            "<debug level> = change debug level" },
	{ "terminate",       ds_wpa_cli_command_terminate,        "= terminate wpa_supplicant" },
	{ "suspend",         ds_wpa_cli_command_suspend,          "= notification of suspend/hibernate" },
	{ "resume",          ds_wpa_cli_command_resume,           "= notification of resume/thaw" },
	{ "bss",             ds_wpa_cli_command_bss,              "<<idx> | <bssid>> = get detailed scan result info" },
	{ "scan",            ds_wpa_cli_command_scan,             "= request new BSS scan" },
	{ "scan_results",    ds_wpa_cli_command_scan_results,     "= get latest scan results" },
	{ "identity",        ds_wpa_cli_command_identity,         "<network id> <identity> = configure identity for an SSID" },
	{ "password",        ds_wpa_cli_command_password,         "<network id> <password> = configure password for an SSID" },
	{ "new_password",    ds_wpa_cli_command_new_password,     "<network id> <password> = change password for an SSID" },
	{ "pin",             ds_wpa_cli_command_pin,              "<network id> <pin> = configure pin for an SSID" },
	{ "otp",             ds_wpa_cli_command_otp,              "<network id> <password> = configure one-time-password for an SSID"	},
	{ "passphrase",      ds_wpa_cli_command_passphrase,       "<network id> <passphrase> = configure private key passphrase for an SSID" },
	{ "sim",             ds_wpa_cli_command_sim,              "<network id> <pin> = report SIM operation result" },
	{ "bssid",           ds_wpa_cli_command_bssid,            "<network id> <BSSID> = set preferred BSSID for an SSID" },
	{ "log_level",       ds_wpa_cli_command_log_level,        "<level> [<timestamp>] = update the log level/timestamp\n  log_level = display the current log level and log options" },
	{ "list_networks",   ds_wpa_cli_command_list_networks,    "= list configured networks" },
	{ "select_network",  ds_wpa_cli_command_select_network,   "<network id> = select a network (disable others)" },
	{ "enable_network",  ds_wpa_cli_command_enable_network,   "<network id> = enable a network" },
	{ "disable_network", ds_wpa_cli_command_disable_network,  "<network id> = disable a network" },
	{ "add_network",     ds_wpa_cli_command_add_network,      "= add a network" },
	{ "remove_network",  ds_wpa_cli_command_remove_network,   "<network id> = remove a network" },
	{ "set_network",     ds_wpa_cli_command_set_network,      "<network id> <variable> <value> = set network variables (shows\n  list of variables when run without arguments)" },
	{ "get_network",     ds_wpa_cli_command_get_network,      "<network id> <variable> = get network variables" },
	{ "dup_network",     ds_wpa_cli_command_dup_network,      "<src network id> <dst network id> <variable> = duplicate network variables"},
	{ "status",          ds_wpa_cli_command_status,           "[verbose] = get current WPA/EAPOL/EAP status" },
	{ "ifname",          ds_wpa_cli_command_ifname,           "= get current interface name" },
	{ "mib",             ds_wpa_cli_command_mib,              "= get MIB variables (dot1x, dot11)" },
	{ "dump",            ds_wpa_cli_command_dump,             "= dump config variables" },
	{ "pmksa",           ds_wpa_cli_command_pmksa,            "= show PMKSA cache" },
	{ "pmksa_flush",     ds_wpa_cli_command_pmksa_flush,      "= flush PMKSA cache entries" },
	{ "blacklist",       ds_wpa_cli_command_blacklist,        "<BSSID> = add a BSSID to the blacklist\nblacklist clear = clear the blacklist\nblacklist = display the blacklist" },
	{ NULL,              NULL,                                NULL},
};

static void print_cmd_help(struct ds_wpa_cli_command *cmd, const char *pad)
{
	char c;
	size_t n;

	DS_WPA_CLI_MSG("%s%s ", pad, cmd->cmd);
	for (n = 0; (c = cmd->usage[n]); n++) {
		DS_WPA_CLI_MSG("%c", c);
		if (c == '\n')
			DS_WPA_CLI_MSG("%s", pad);
	}
	DS_WPA_CLI_MSG("\n");
}


static void ds_wpa_cli_print_help(const char *cmd)
{
	int n;

	DS_WPA_CLI_MSG("commands:\n");
	for (n = 0; wpa_cli_commands[n].cmd; n++)
			print_cmd_help(&wpa_cli_commands[n], "  ");
}

static int str_match(const char *a, const char *b)
{
	return os_strncmp(a, b, os_strlen(b)) == 0;
}

static WPA_CMD_HANDLER get_cmd_handler(char* run_cmd)
{
	struct ds_wpa_cli_command *cli_cmd;

	cli_cmd = wpa_cli_commands;
	while (cli_cmd->cmd) {
		if (str_match(cli_cmd->cmd, run_cmd))
			return cli_cmd->cmd_handler;
		cli_cmd += 1;
	}

	return NULL;
}

static void proc_cli_command(struct wpa_ctrl *conn, WPA_CMD_HANDLER hanlder, int argc, char *argv[])
{
	if (argc <= 1) {
		ds_wpa_cli_usage();
		return;
	}

	hanlder(conn, argc - 2, argv + 2);
}

void ds_wpa_cli_version(void)
{
	DS_WPA_CLI_MSG("ds_wpa_cli v2.4\n");
}

int ds_wpa_cli_main(int argc, char *argv[])
{
	WPA_CMD_HANDLER handle;
	struct wpa_ctrl *ctrl_conn;

	if (argc < 2) {
		ds_wpa_cli_usage();

		return 0;
	}
	else if (argc > 1) {
		if (str_match(argv[1], "-h")) {
			ds_wpa_cli_usage();
			ds_wpa_cli_command_help(NULL, argc, argv);\

			return 0;
		}
		if (str_match(argv[1], "-v")) {
			ds_wpa_cli_version();

			return 0;
		}
	}

	handle = get_cmd_handler(argv[1]);
	if (handle == NULL)
		return 0;

	ctrl_conn = wpa_ctrl_open(WLAN_IFACE_NAME);
	if (ctrl_conn) {
		proc_cli_command(ctrl_conn, handle, argc, argv);
		wpa_ctrl_close(ctrl_conn);
	} else
		DS_WPA_CLI_MSG("Fail open %s\n", WLAN_IFACE_NAME);

	return 0;
}
