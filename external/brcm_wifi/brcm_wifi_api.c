/****************************************************************************
 *
 * Copyright 2017 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/

#include <tinyara/config.h>

#include "utils/includes.h"
#include "utils/common.h"
#include "common/wpa_ctrl.h"
#include "common/ieee802_11_defs.h"
#include "common/ieee802_11_common.h"
#include "common/defs.h"

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/stat.h>
#include <brcm_wifi/brcm_wifi_api.h>
#include <brcm_wifi/brcm_wifi_nvmap.h>

#define REMOVE_NVRAM

extern int wpa_supplicant_main(int argc, char *argv[]);
/* Reuse SSID decoder from wpa_supplicant: mind requires buf to be len+1 long to avoid mem corruption*/
extern size_t printf_decode(u8 *buf, size_t maxlen, const char *str);
extern void printf_encode(char *txt, size_t maxlen, const u8 *data, size_t len);

#ifdef CONFIG_DEBUG_WLAN_API_VERBOSE
#define VPRINT(format, ...) printf("BRCM API VERBOSE (%s): " format, __FUNCTION__, ##__VA_ARGS__)
#define BRCM_API_VERBOSE  (1)
#ifndef CONFIG_DEBUG_WLAN_API_DEBUG
#define CONFIG_DEBUG_WLAN_API_DEBUG
#endif
#else
#define VPRINT(a, ...) (void)0
#define BRCM_API_VERBOSE  (0)
#endif							//CONFIG_DEBUG_WLAN_API_VERBOSE

#ifndef CONFIG_DEBUG_WLAN_API_DEBUG
#define CONFIG_DEBUG_WLAN_API_DEBUG
#endif
#ifdef CONFIG_DEBUG_WLAN_API_DEBUG
#define DPRINT(format, ...) printf("BRCM API DEBUG (%s): " format, __FUNCTION__, ##__VA_ARGS__)
#ifndef CONFIG_DEBUG_WLAN_API_ERROR
#define CONFIG_DEBUG_WLAN_API_ERROR
#endif
#else
#define DPRINT(a, ...) (void)0
#endif							//CONFIG_DEBUG_WLAN_API_DEBUG

#ifdef CONFIG_DEBUG_WLAN_API_ERROR
#define EPRINT(format, ...) printf("BRCM API ERROR (%s): " format, __FUNCTION__, ##__VA_ARGS__)
#else
#define EPRINT(a, ...) (void)0
#endif							//CONFIG_DEBUG_WLAN_API_ERROR

// Enable DEBUG_SLEEP to allow sleeps and print out during run - can only be enabled manually
#define BRCM_WIFI_API_DEBUG_SLEEP       0

/*
 * test functions
 */
#ifdef BRCM_WIFI_API_DEBUG_SLEEP
static int sleep_count = 0;
void brcm_demo_app_sleep(int seconds, char *str)
{
	printf("BRCM API debug sleep number %d - %s\n", sleep_count, str);
	sleep(seconds);
	printf("BRCM API debug sleep number %d DONE \n", sleep_count++);
}
#endif

#define IEEE80211_CAP_IBSS              0x0002

#define WPA_COMMAND_SCAN                "SCAN"
#define WPA_COMMAND_SCAN_STOP           "ABORT_SCAN"
#define WPA_COMMAND_SCAN_INTERVAL       "SCAN_INTERVAL "
#define WPA_COMMAND_BSS_EXPIRE_AGE      "BSS_EXPIRE_AGE "
#define WPA_COMMAND_SCAN_RESULTS        "SCAN_RESULTS"
#define WPA_COMMAND_AUCOTONNECT         "STA_AUTOCONNECT "
#define WPA_COMMAND_LIST_NETWORKS       "LIST_NETWORKS"
#define WPA_COMMAND_ADD_NETWORK         "ADD_NETWORK"
#define WPA_COMMAND_REMOVE_NETWORK      "REMOVE_NETWORK "
#define WPA_COMMAND_BSS                 "BSS "
#define WPA_COMMAND_UPDATE_CONFIG       "SET update_config "
#define WPA_COMMAND_SAVE_CONFIG         "SAVE_CONFIG"

#define WPA_COMMAND_SET                 "SET "
#define WPA_COMMAND_SET_NETWORK         "SET_NETWORK "
#define WPA_PARAM_KEY_MGMT_NONE         "key_mgmt NONE"
#define WPA_PARAM_KEY_MGMT_WEP          "key_mgmt NONE"
#define WPA_PARAM_KEY_MGMT_WEP_SHARED   "key_mgmt NONE"
#define WPA_PARAM_KEY_MGMT_WPA_PSK      "key_mgmt WPA-PSK"
#define WPA_PARAM_SSID                  "ssid "
#define WPA_PARAM_SCAN_SSID             "scan_ssid "
#define WPA_PARAM_BSSID                 "bssid "
#define WPA_PARAM_PSK                   "psk "
#define WPA_PARAM_WEPKEY                "wep_key0 "
#define WPA_PARAM_FREQUENCY             "frequency "
#define WPA_PARAM_MODE_AP               "mode 2"
#define WPA_PARAM_MODE_STA              "mode 0"
#define WPA_PARAM_NETWORK_DISABLED      "disabled 0"
#define WPA_PARAM_AUTH_ALG              "auth_alg "
#define WPA_PARAM_PAIRWISE              "pairwise "
#define WPA_PARAM_CIPHER                "cipher "
#define WPA_PARAM_GROUP                 "group "
#define WPA_PARAM_PROTO                 "proto "
#define WPA_PARAM_BEACON_INT            "beacon_int "
#define WPA_PARAM_DTIM_PERIOD           "dtim_period "
#define WPA_PARAM_VSIE                  "ap_vendor_ie "
#define WPA_PARAM_DISABLE_HT            "disable_ht 1"
#define WPA_PARAM_DISABLE_SHORT_GI      "disable_sgi 1"
#define WPA_PARAM_HT_CAPAB              "ht_capab "
#define WPA_PARAM_HT_MCS                "ht_mcs "

#define WPA_COMMAND_SELECT_NETWORK      "SELECT_NETWORK "
#define WPA_COMMAND_ENABLE_NETWORK      "ENABLE_NETWORK "
#define WPA_COMMAND_DISABLE_NETWORK     "DISABLE_NETWORK all"
#define WPA_COMMAND_RECONNECT           "RECONNECT"
#define WPA_COMMAND_DISCONNECT          "DISCONNECT"
#define WPA_COMMAND_STOP_AP             "STOP_AP"

#define WPA_COMMAND_TERMINATE           "TERMINATE"

#define WPA_COMMAND_STATUS              "STATUS"
#define WPA_STATE_COMPLETED             "COMPLETED"
#define WPA_STATE_DISCONNECTED          "DISCONNECTED"
#define WPA_STATE_SCANNING              "SCANNING"
#define WPA_VALUE_SSID                  "ssid="
#define WPA_VALUE_IP_ADDRESS            "ip_address="
#define WPA_VALUE_ADDRESS               "address="
#define WPA_VALUE_BSSID                 "bssid="
#define WPA_VALUE_WPA_STATE             "wpa_state="
#define WPA_VALUE_WPS_STATE             "wps_state="
#define WPA_VALUE_BEACON_INT            "beacon_int="
#define WPA_VALUE_DTIM_PERIOD           "dtim_period="
#define WPA_VALUE_CAPABILITIES          "capabilities="
#define WPA_VALUE_LEVEL                 "level="
#define WPA_VALUE_QUAL                  "qual="
#define WPA_VALUE_FLAGS                 "flags="
#define WPA_VALUE_FREQ                  "freq="
#define WPA_VALUE_FREQUENCY             "frequency="
#define WPA_VALUE_REASON                "reason="
#define WPA_VALUE_LOACLLY_GENERATED     "locally_generated="
#define WPA_VALUE_IE                    "ie="

#define WPA_VALUE_AUTH_ALG_OPEN         "OPEN"
#define WPA_VALUE_AUTH_ALG_SHARED       "SHARED"
#define WPA_VALUE_AUTH_ALG_OPEN_SHARED  "OPEN SHARED"
#define WPA_VALUE_PAIRWISE_TKIP         "TKIP"
#define WPA_VALUE_PAIRWISE_MIXED        "CCMP TKIP"
#define WPA_VALUE_PAIRWISE_AES          "CCMP"
#define WPA_VALUE_CIPHER_NONE BIT       "NONE"
#define WPA_VALUE_CIPHER_WEP40          "WEP40"
#define WPA_VALUE_CIPHER_WEP104         "WEP104"
#define WPA_VALUE_CIPHER_TKIP           "TKIP"
#define WPA_VALUE_CIPHER_CCMP           "CCMP"
#define WPA_VALUE_CIPHER_MIXED          "TKIP CCMP"
#define WPA_VALUE_PROTO_WPA             "WPA"
#define WPA_VALUE_PROTO_RSN             "RSN"	// Same as WPA2
#define WPA_VALUE_PROTO_MIXED           "WPA RSN"
#define WPA_VALUE_GROUP_CCMP            "CCMP"
#define WPA_VALUE_GROUP_TKIP            "TKIP"
#define WPA_VALUE_GROUP_WEP40           "WEP40"
#define WPA_VALUE_GROUP_WEP104          "WEP104"

#define WPA_COMMAND_DRIVER              "DRIVER "
#define WPA_PARAM_COUNTRY               "COUNTRY"
#define WPA_PARAM_FORCE_PANIC           "FORCE_PANIC"
#define WPA_COMMAND_SET_TX_POWER        "SET set_tx_power "
#define WPA_COMMAND_GET_TX_POWER        "GET get_tx_power"

#define WPA_COMMAND_SIGNAL_POLL         "SIGNAL_POLL"
#define WPA_VALUE_RSSI                  "RSSI="
#define WPA_VALUE_LINKSPEED             "LINKSPEED="

/* Events received - used to handle connect failed cases */
#define BRCM_EVENT_ASSOCIATION_REQ_FAILED "Association request to the driver failed"

/* SSID encoded in a string - worst case is all 4-octet hex digits + '\0' */
#define WPA_MAX_SSID_LEN                (4 * 32 + 1)

#define WPA_COMMAND_MAX_SIZE            (30 + WPA_MAX_SSID_LEN)
#define WPA_BUFFER_SIZE                 4096
#define REQUEST_RETRY_COUNT             5

#define BRCM_WIFI_WEP_ASCII_KEY_MIN     5
#define BRCM_WIFI_WEP_ASCII_KEY_MAX     13
#define BRCM_WIFI_WEP_HEX_KEY_MIN       10
#define BRCM_WIFI_WEP_HEX_KEY_MAX       26
#define BRCM_WIFI_WPA_ASCII_KEY_MIN     8
#define BRCM_WIFI_WPA_ASCII_KEY_MAX     (BRCM_PASSPHRASE_LEN - 1)

/* Scan interval in specified in brcm_wifi_api.h.
 * Below values are use to boundary check */
#define BRCM_SCAN_INTERVAL_MIN   10
#define BRCM_SCAN_INTERVAL_MAX   60

typedef enum {
	BRCM_WIFIAPI_STATE_NOT_STARTED,
	BRCM_WIFIAPI_STATE_SUPPLICANT_RUNNING,	//BRCM_WIFIAPI_STATE_STA_ENABLED
	BRCM_WIFIAPI_STATE_STA_CONNECTING,
	BRCM_WIFIAPI_STATE_STA_CONNECTED,
	BRCM_WIFIAPI_STATE_STA_DISCONNECTING,
	BRCM_WIFIAPI_STATE_AP_ENABLING,
	BRCM_WIFIAPI_STATE_AP_ENABLED,
	BRCM_WIFIAPI_STATE_AP_CONNECTED,
	BRCM_WIFIAPI_STATE_AP_DISABLING,
	BRCM_WIFIAPI_STATE_TERMINATING,
	BRCM_WIFIAPI_STATE_RECOVERING,
	BRCM_WIFIAPI_STATE_COUNT
} brcm_api_states_t;

#ifdef BRCM_API_VERBOSE
char *brcm_state_strings[BRCM_WIFIAPI_STATE_COUNT] = {
	"STATE_NOT_STARTED",
	"STATE_SUPPLICANT_RUNNING",	//BRCM_WIFIAPI_STATE_STA_ENABLED
	"STATE_STA_CONNECTING",
	"STATE_STA_CONNECTED",
	"STATE_STA_DISCONNECTING",
	"STATE_AP_ENABLING",
	"STATE_AP_ENABLED",
	"STATE_AP_CONNECTED",
	"STATE_AP_DISABLING",
	"STATE_TERMINATING"
};
#endif

// BEGIN Recovery
#ifdef CONFIG_SCSC_WLAN_AUTO_RECOVERY

typedef struct brcm_recovery_data {
	brcm_api_states_t old_state;
	brcm_api_states_t recovering_state;
	WiFi_InterFace_ID_t old_interface_type;
	brcm_network_link_callback_t link_up;	// callback function
	brcm_network_link_callback_t link_down;	// callback function
	network_scan_result_handler_t scan_result_handler;	// callback function
	char remote_bssid[18];		// char string e.g. xx:xx:xx:xx:xx:xx
	brcm_ap_config_t ap_config;	// contains security pointer!
	uint8_t ssid[BRCM_SSID_LEN + 1];	// 802.11 spec defined unspecified or uint8
	int8_t ssid_len;			// length of ssid - # of valid octets
	uint8_t bssid[18];			// char string e.g. xx:xx:xx:xx:xx:xx
	brcm_security_config_t *security;
} brcm_recovery_data_t;

static brcm_recovery_data_t g_recovery_data;
static pthread_t g_recovery_thread;
static int g_recovery_running = 0;
static sem_t g_sem_recover;
#endif							// CONFIG_SCSC_WLAN_AUTO_RECOVERY

static int g_recovering = 0;
// END Recovery

static brcm_api_states_t g_state = BRCM_WIFIAPI_STATE_NOT_STARTED;
static struct wpa_ctrl *g_ctrl_conn;
static int g_wpa_attached = 0;
static int g_running = 0;
static int g_scanning = 0;
static pthread_t g_monitoring_thread;
static sem_t g_sem_terminate;
static sem_t g_sem_ap_mode;
static sem_t g_sem_disconnect;

static sem_t g_sem_api_block;
static pthread_mutex_t mutex_state;
static bool g_mutex_initialized = FALSE;
static network_scan_result_handler_t g_scan_result_handler;
static brcm_network_link_callback_t g_link_up;
static brcm_network_link_callback_t g_link_down;
static brcm_ap_config_t *g_ap_config;
static pid_t g_task = 0;
static char *g_network_id = NULL;
static uint8_t g_num_sta_connected = 0;
	#ifndef REMOVE_NVRAM
static brcm_wifi_nv_data_t *g_brcm_wifi_nv_data;
static char g_country_code[3] = { 0 };
	#endif
static void *nvram;

#ifdef CONFIG_BRCM_WIFI_FILESYSTEM_SUPPORT
static const char g_supplicant_conf[] = "ctrl_interface=udp";

#endif

#ifdef CONFIG_DEBUG_WLAN_SUPPLICANT_ERROR
static char *sup_argv[5];		/* We need this to be a static array, as the supplicant needs it
								 * after we exit from brcm_init()*/
#elif defined(CONFIG_DEBUG)
static char *sup_argv[5];
#else
static char *sup_argv[6];
#endif

#define LOCKUNLOCK_CRITICAL { \
	EPRINT("LOCKUNLOCK_CRITICAL - %s\n", __func__); \
	if (BRCM_API_VERBOSE) fflush(stdout); \
	pthread_mutex_trylock(&mutex_state); \
	pthread_mutex_unlock(&mutex_state); \
}

#define ENTER_CRITICAL { \
	VPRINT("ENTER_CRITICAL in API - %s\n", __func__); \
	if (!g_mutex_initialized) { \
		VPRINT("Initializing the mutex\n"); \
		if (pthread_mutex_init(&mutex_state, NULL) != OK) { \
			EPRINT("Could not initialize mutex\n"); \
		} else { \
			VPRINT("Mutex ready\n"); \
			g_mutex_initialized = TRUE; \
		} \
	} \
	if (BRCM_API_VERBOSE) fflush(stdout); \
	int err = pthread_mutex_lock(&mutex_state); \
	if (err != OK) { \
		EPRINT("mutex lock failed with error %d", err); \
	} \
	if (g_state == BRCM_WIFIAPI_STATE_RECOVERING || g_recovering) { \
		VPRINT("sem_wait recovering - %s\n", __func__); \
		if (BRCM_API_VERBOSE) fflush(stdout); \
		sem_wait(&g_sem_api_block); \
	} \
}

#define LEAVE_CRITICAL { \
	VPRINT("LEAVE_CRITICAL in API - %s\n", __func__); \
	if (BRCM_API_VERBOSE) fflush(stdout); \
	pthread_mutex_unlock(&mutex_state); \
}

/*
 * Prototype functions
 */
static int8_t brcm_api_start(WiFi_InterFace_ID_t interface_id, const brcm_ap_config_t *ap_config);
static int8_t brcm_join_network(uint8_t *ssid, int ssid_len, uint8_t *bssid, const brcm_security_config_t *sec_config);
static void brcm_wpa_reopen(void);
static int8_t brcm_terminate_supplicant(void);
static int8_t brcm_check_status(uint8_t *ssid, int8_t *ssid_len, char *bssid);
static int8_t brcm_get_network(uint8_t *ssid, uint8_t ssid_len, char **network_id);
static int8_t brcm_recv_pending(char **result);
static void brcm_remove_network(char *network_id);
	#ifndef REMOVE_NVRAM
static void brcm_init_nvram(void);
	#endif
static uint8_t brcm_stop_supplicant(void);
static void brcm_deinit(void);
static int8_t brcm_wpa_close(bool terminate);
static int8_t brcm_start_scan(void);
static int8_t brcm_get_country_code(char *country_code);
static WiFi_InterFace_ID_t brcm_get_op_mode(void);
static void brcm_set_scan_interval(uint8_t interval);
static int8_t brcm_disable_all_networks(void);
#ifdef CONFIG_SCSC_WLAN_AUTO_RECOVERY
static brcm_ap_config_t *brcm_get_ap_config(void);
#endif
/*
 * Private functions
 */
void brcm_msg_cb(char *msg, size_t len)
{
	DPRINT("BRCM_API msg_cb - %s\n", msg);
}

/**
 * Used to send request to WPA supplicant
 * Param:
 *  cmd         Command string
 *  success     Used if command returns no buffer string to give result
 *              otherwise use NULL
 * Return:
 *  buf         Character array[WPA_BUFFER_SIZE] with result string
  */
static char *brcm_send_request(char *cmd, int8_t *success)
{
	int retry = 0;

	if (g_ctrl_conn == NULL) {
		EPRINT("Ctrl iface not available\n");
		return NULL;
	}

	char *buf = (char *)zalloc(WPA_BUFFER_SIZE);
	if (buf == NULL) {
		return buf;
	}

	size_t len = WPA_BUFFER_SIZE - 1;
	DPRINT("BRCM_API command %s \n", cmd);
	for (retry = 0; retry < REQUEST_RETRY_COUNT; retry++) {
		int ret = wpa_ctrl_request(g_ctrl_conn, cmd, strlen(cmd), buf, &len, NULL);
		if (ret == -2) {
			EPRINT("%s command timed out. \n", cmd);
			if (success) {
				*success = BRCM_STATUS_ERROR;
			}
		} else if (ret < 0) {
			EPRINT("... command failed. \n");
			if (success) {
				*success = BRCM_STATUS_ERROR;
			}
		} else {
			buf[len] = '\0';
			VPRINT("Result length=%d\n%s", (int)len, buf);
			if (len > 0 && buf[len - 1] != '\n') {
				DPRINT("\n");
			}
			if (success) {
				if (strncmp(buf, "OK", strlen("OK")) == 0) {
					*success = BRCM_STATUS_SUCCESS;
				} else if (strncmp(buf, "FAIL", strlen("FAIL")) == 0) {
					DPRINT("BRCM_API command %s returned FAIL from supplicant \n", cmd);
					*success = BRCM_STATUS_COMMAND_FAILED;
				} else if (strncmp(buf, "UNKNOWN COMMAND", strlen("UNKNOWN COMMAND")) == 0) {
					*success = BRCM_STATUS_COMMAND_UNKNOWN;	//e.g. "UNKNOWN COMMAND"
				} else {
					*success = BRCM_STATUS_ERROR;
				}
			}
			break;
		}
		memset(buf, 0, WPA_BUFFER_SIZE);
	}
	return buf;
}

static void brcm_send_command_str_digit(char *string, int digit)
{
	char *pbuf = NULL;
	char command[WPA_COMMAND_MAX_SIZE] = { 0 };
	snprintf(command, WPA_COMMAND_MAX_SIZE, "%s%d", string, digit);
	pbuf = brcm_send_request(command, NULL);
	if (pbuf) {
		free(pbuf);
		pbuf = NULL;
	}
}

static void brcm_send_command_str_upto_4(char *one, char *two, char *three, char *four, int8_t *result)
{
	char *pbuf = NULL;
	char command[WPA_COMMAND_MAX_SIZE] = { 0 };
	if (four && three && two && one) {
		snprintf(command, WPA_COMMAND_MAX_SIZE, "%s%s %s%s", one, two, three, four);
	} else if (three && two && one) {
		snprintf(command, WPA_COMMAND_MAX_SIZE, "%s%s %s", one, two, three);
	} else if (two && one) {
		snprintf(command, WPA_COMMAND_MAX_SIZE, "%s%s", one, two);
	} else if (one) {
		snprintf(command, WPA_COMMAND_MAX_SIZE, "%s", one);
	}
	if (command[0] != '\0') {
		pbuf = brcm_send_request(command, result);
	}
	if (pbuf) {
		free(pbuf);
		pbuf = NULL;
	}
}

static int8_t brcm_save_config(void)
{
	int8_t result = BRCM_STATUS_ERROR;
	char *pbuf = NULL;
	pbuf = brcm_send_request(WPA_COMMAND_SAVE_CONFIG, &result);
	if (pbuf) {
		free(pbuf);
		pbuf = NULL;
	}
	return result;
}

static int8_t brcm_init_filesystem(void)
{
	uint8_t ret = BRCM_STATUS_ERROR;
#ifdef CONFIG_BRCM_WIFI_FILESYSTEM_SUPPORT
	FILE *fp = NULL;
	char *conffile = CONFIG_BRCM_WIFI_DIR "/" CONFIG_BRCM_WIFI_CONFIG_FILE_NAME;
	char *logfile = CONFIG_BRCM_WIFI_DIR "/" CONFIG_BRCM_WIFI_LOG_FILE_NAME;
	/* Create the Wi-Fi directory */
	ret = mkdir(CONFIG_BRCM_WIFI_DIR, 0666);
	if (ret == OK || get_errno() == EEXIST) {
		/* Clear supplicant log file */
		fp = fopen(logfile, "w+");
		DPRINT("Creating logfile %s\n", logfile);
		if (fp != NULL) {
			fclose(fp);
		}
		struct stat buffer;
		if (stat(conffile, &buffer)) {
			DPRINT("Creating config file %s\n", conffile);
			fp = fopen(conffile, "w+");
			if (fp == NULL) {
				EPRINT("write_file: ERROR failed to open %s for writing, errno=%d\n", conffile, errno);
			} else {
				ret = fwrite(g_supplicant_conf, sizeof(g_supplicant_conf), 1, fp);

				if (ret != 1) {
					EPRINT("write_file: ERROR failed to write to %s, errno=%d, ret=%d\n", conffile, errno, ret);
					fclose(fp);
					ret = BRCM_STATUS_ERROR;
				}
				fclose(fp);
				ret = BRCM_STATUS_SUCCESS;
			}
		} else {
			ret = BRCM_STATUS_SUCCESS;
		}
	}
#endif

	return ret;
}

static bool brcm_get_security_from_flags(const char *flag_str, brcm_security_config_t **sec, uint8_t *sec_count)
{
	bool ret = FALSE;			//NO security
	char *pos;
	char *end;
	char buffer[WPA_COMMAND_MAX_SIZE];
	memset(buffer, 0, WPA_COMMAND_MAX_SIZE);
	//identify how many mode security structs are needed
	//simply count [ in the flag string
	int x = 0;
	uint8_t mode_count = 0;
	if (flag_str != NULL) {
		while (flag_str[x] != '\0') {
			if (flag_str[x] == '[') {
				/* we should not count [P2P] as we do not support it,
				 * and [ESS] as this it not a security type */
				if ((flag_str[x + 1] != 'E') && (flag_str[x + 1] != 'P')) {
					mode_count++;
				} else {
					// Assumes that EAP can be in combination with supported security
					VPRINT("not counted: %s\n", &flag_str[x]);
				}
			}
			x++;
		}
		if (!mode_count) {
			// if flags is only ESS then accept as this is an open AP
			if (strncmp(flag_str, "[ESS]", strlen(flag_str)) == 0) {
				ret = TRUE;
			}
			goto out;
		} else {
			ret = TRUE;
			*sec_count = mode_count;
			*sec = zalloc(sizeof(brcm_security_config_t) * mode_count);
			if (*sec == NULL) {
				EPRINT("could not allocate memory for security mode block\n");
				ret = FALSE;
				goto out;
			}
		}

		brcm_security_config_t *tmpsec = *sec;	//point to first security struct in list
		uint8_t strlen = 0;
		pos = (char *)flag_str;
		for (x = 0; x < mode_count; x++) {
			bool wpa = FALSE;
			bool wpa2 = FALSE;

			pos = strchr(pos, '[');
			if (pos == NULL) {
				break;
			}
			pos++;
			end = strchr(pos, ']');
			strlen = end - pos;	//includes room for null termination
			memcpy(&buffer, pos, strlen);
			buffer[strlen] = '\0';
			// Extract flags - e.g. flags=[WPA-PSK-CCMP+TKIP][WPA2-PSK-CCMP+TKIP][ESS]

			if (strstr(buffer, "WPA2-PSK")) {
				wpa2 = TRUE;
			} else if (strstr(buffer, "WPA-PSK")) {
				wpa = TRUE;
			} else if (strstr(buffer, "WEP")) {
				// It is not possible to distinguish WEP open and shared so both set
				tmpsec->secmode = BRCM_SEC_MODE_WEP | BRCM_SEC_MODE_WEP_SHARED;
			} else if (strstr(buffer, "WPA2-EAP")) {
				tmpsec->secmode = BRCM_SEC_MODE_EAP;
			} else if (strstr(buffer, "WPA-EAP")) {
				tmpsec->secmode = BRCM_SEC_MODE_EAP;
			} else if (strstr(buffer, "NONE")) {
				// deprecated
			} else if (strstr(buffer, "IBSS")) {
				// not necessary any security
			} else if (strstr(buffer, "ESS")) {
				tmpsec->secmode = BRCM_SEC_MODE_OPEN;
			} else if (strstr(buffer, "P2P")) {
				// P2P is not supported yet
			}

			if (wpa) {
				// find cipher related settings
				if (strstr(buffer, "CCMP+TKIP")) {
					tmpsec->secmode = BRCM_SEC_MODE_WPA_MIXED;
				} else if (strstr(buffer, "CCMP")) {
					tmpsec->secmode = BRCM_SEC_MODE_WPA_CCMP;
				} else if (strstr(buffer, "TKIP")) {
					tmpsec->secmode = BRCM_SEC_MODE_WPA_TKIP;
				}
				tmpsec++;		//go to next struct
			} else if (wpa2) {
				// find cipher related settings
				if (strstr(buffer, "CCMP+TKIP")) {
					tmpsec->secmode = BRCM_SEC_MODE_WPA2_MIXED;
				} else if (strstr(buffer, "CCMP")) {
					tmpsec->secmode = BRCM_SEC_MODE_WPA2_CCMP;
				} else if (strstr(buffer, "TKIP")) {
					tmpsec->secmode = BRCM_SEC_MODE_WPA2_TKIP;
				}
				tmpsec++;		//go to next struct
			}
		}
	}
out:
	return ret;
}

static uint8_t *brcm_hexstr_2_bytearray(char *str, size_t str_size, size_t *array_size)
{
	char *string = str;
	uint8_t *byte_array = NULL;
	size_t str_len;
	size_t array_len;
	size_t count = 0;
	size_t i;

	if (str == NULL) {
		VPRINT("str NULL \n");
		return NULL;
	}

	str_len = str_size;
	if (str_len % 2 != 0) {	// must be even
		VPRINT("str not even length\n");
		return NULL;
	}
	array_len = str_len / 2;
	byte_array = (uint8_t *)zalloc(array_len + 1);
	if (byte_array) {
		char a, b;
		VPRINT("length=%d, bytes= ", array_len);
		for (i = 0; i < str_len; i += 2) {
			a = string[i + 0];
			b = string[i + 1];
			byte_array[i / 2] = ((a <= '9' ? a - '0' : a - 'a' + 10) << 4) + (b <= '9' ? b - '0' : b - 'a' + 10);
			// printf used due to not wanting function name pre-appended
			if (BRCM_API_VERBOSE) {
				printf("%02x, ", byte_array[i / 2]);
			}

			count++;
		}
		// printf used due to not wanting function name pre-appended
		if (BRCM_API_VERBOSE) {
			printf("\n");
		}
	}
	*array_size = count;
	return byte_array;
}

static uint8_t *brcm_bytearray_2_hexstr(const uint8_t *bytes, size_t size, size_t *stringlen)
{

	uint8_t *hexstr = zalloc((size * 2) + 1);	//need room for null termination
	if (hexstr != NULL) {
		uint8_t *ptr = hexstr;
		size_t i = 0;
		for (i = 0; i < size; i++) {
			ptr += snprintf((char *)ptr, 3 /*xx + \0 */ , "%02X", bytes[i]);
		}
		*stringlen = strlen((char *)hexstr);
	} else {
		*stringlen = 0;
	}
	return hexstr;
}

static const uint8_t *brcm_get_ie(const uint8_t *byte_array, const size_t size, uint8_t ie_type)
{
	const uint8_t *end;
	const uint8_t *pos;
	VPRINT("Find IE type %02x \n", ie_type);
	// Find IE type
	pos = byte_array;
	end = pos + size;
	while (pos + 1 < end) {
		if (pos + 2 + pos[1] > end) {
			VPRINT(" not found \n");
			break;
		}
		if (pos[0] == ie_type) {
			VPRINT(" found\n");
			return pos;
		}
		pos += 2 + pos[1];
	}

	return NULL;
}

/**
 * Get BSS details from wpa_supplicant.
 * Returns true at success.
 */
static bool brcm_get_bss_info(const char *bssid, brcm_scan_info_t *info)
{
	char *pbuf = NULL;
	char *pos;
	char *end;
	char command[WPA_COMMAND_MAX_SIZE] = { 0 };

	/*
	   id=11
	   bssid=20:4e:7f:a5:44:0a
	   freq=2472
	   beacon_int=100
	   capabilities=0x0411
	   qual=-64
	   noise=0
	   level=0
	   tsf=0011221546694051
	   age=1
	   ie=0019636f65782d6d616e2d746573742d6e672d333430302d326734010882840b162430486c03010d2a01002f010030140100000fac040100000fac040100000fac020c0032040c121860dd7e0050f204104a00011010440001021041000100103b0001031047001030138638384c1a56f8ffe0746aedf8c51021000d4e4554474541522c20496e632e10230008574e44523334303010240008574e4452333430301042000230311054000800060050f204000110110008574e445233343030100800020084103c000103dd090010180200f0050000dd180050f2020101800003a4000027a4000042435e0062322f00
	   flags=[WPA2-PSK-CCMP][ESS]
	   ssid=coex-man-test-ng-3400-2g4
	   snr=0
	   est_throughput=1000
	 */

	snprintf(command, WPA_COMMAND_MAX_SIZE, "%s%s", WPA_COMMAND_BSS, bssid);
	pbuf = brcm_send_request(command, NULL);
	if (pbuf) {
		// Extract bssid
		pos = strstr(pbuf, WPA_VALUE_BSSID);
		if (pos == NULL) {
			goto errout;
		}
		pos += 6;
		end = strchr(pos, '\n');
		if (end == NULL) {
			goto errout;
		}
		*end = '\0';
		memcpy(&(info->bssid), pos, 18);
		pos = end + 1;
		// Extract channel
		pos = strstr(pos, WPA_VALUE_FREQ);
		if (pos == NULL) {
			goto errout;
		}
		pos += 5;
		end = strchr(pos, '\n');
		if (end == NULL) {
			goto errout;
		}
		*end = '\0';
		ieee80211_freq_to_chan(strtol(pos, &end, 10), &(info->channel));
		pos = end + 1;
		// Extract beacon interval
		pos = strstr(pos, WPA_VALUE_BEACON_INT);
		if (pos == NULL) {
			goto errout;
		}
		pos += 11;
		end = strchr(pos, '\n');
		if (end == NULL) {
			goto errout;
		}
		*end = '\0';
		info->beacon_period = strtol(pos, &end, 10);
		pos = end + 1;
		// Extract bss_type
		pos = strstr(pos, WPA_VALUE_CAPABILITIES);
		if (pos == NULL) {
			goto errout;
		}
		pos += 13;
		end = strchr(pos, '\n');
		if (end == NULL) {
			goto errout;
		}
		*end = '\0';
		int caps = strtol(pos, &end, 16);
		pos = end + 1;
		if (caps & IEEE80211_CAP_IBSS) {
			info->bss_type = 1;
		}
		// Extract qual as RSSI (set in t20_ops.c)
		pos = strstr(pos, WPA_VALUE_QUAL);
		if (pos == NULL) {
			goto errout;
		}
		pos += 5;
		end = strchr(pos, '\n');
		if (end == NULL) {
			goto errout;
		}
		*end = '\0';
		info->rssi = strtol(pos, &end, 10);
		pos = end + 1;
		// Extract ei data
		pos = strstr(pos, WPA_VALUE_IE);
		if (pos == NULL) {
			goto errout;
		}
		pos += 3;
		end = strchr(pos, '\n');
		if (end == NULL) {
			goto errout;
		}
		*end = '\0';
		{
			size_t length = end - pos;
			const uint8_t *ht_capab = NULL, *vendor_specific = NULL;
			size_t size = 0;
			uint8_t *bytes = (uint8_t *)brcm_hexstr_2_bytearray(pos, length, &size);
			if (bytes) {
				ht_capab = brcm_get_ie(bytes, size, WLAN_EID_HT_CAP);
				if (ht_capab) {
					// Extract phy_mode
					info->phy_mode = 1;

					if (ht_capab[1] == 26) {
						struct ieee80211_ht_capabilities *cap;
						cap = (struct ieee80211_ht_capabilities *)(ht_capab + 2);
						uint16_t ht_capab_info = le_to_host16(cap->ht_capabilities_info);
						VPRINT("ht_capab_info %04x found \n", ht_capab_info);
						// Extract ht_modes - green field
						info->ht_mode.ht_capab_info = ht_capab_info;
						if (BRCM_API_VERBOSE) {
							if ((ht_capab_info & HT_CAP_INFO_GREEN_FIELD) == HT_CAP_INFO_GREEN_FIELD) {
								VPRINT("ht_capab HT_CAP_INFO_GREEN_FIELD found\n");
							}
							// Extract ht_modes - short guard interval
							if ((ht_capab_info & HT_CAP_INFO_SHORT_GI20MHZ) == HT_CAP_INFO_SHORT_GI20MHZ) {
								VPRINT("ht_capab HT_CAP_INFO_SHORT_GI20MHZ found\n");
							}
						}
						// Extract ht_modes - ht_mcs
						size_t i;
						for (i = 0; i < IEEE80211_HT_MCS_MASK_LEN; i++) {
							VPRINT("ht_capab ht_mcs %02x found\n", cap->supported_mcs_set[i]);
							info->ht_mode.mcs_index[i] = (uint8_t)cap->supported_mcs_set[i];
						}
					}
				}

				uint8_t *tmpBytes = bytes;
				uint8_t *tmpEnd = bytes + length;
				// Extract wps_support
				while (tmpBytes + 1 < tmpEnd) {
					vendor_specific = brcm_get_ie(tmpBytes, size, WLAN_EID_VENDOR_SPECIFIC);
					if (vendor_specific) {
						if (WPS_IE_VENDOR_TYPE == WPA_GET_BE32(&vendor_specific[2])) {
							VPRINT("IE data - WLAN_EID_VENDOR_SPECIFIC + WPS_IE_VENDOR_TYPE length=%d found\n", vendor_specific[1]);
							info->wps_support = 1;
							break;
						}
						tmpBytes += (2 + vendor_specific[1]);
					} else {
						break;
					}
				}
				if (bytes) {
					free(bytes);
					bytes = NULL;
				}
			}
		}
		pos = end + 1;
		// Extract flags containing security mode
		pos = strstr(pos, WPA_VALUE_FLAGS);
		if (pos == NULL) {
			goto errout;
		}
		pos += 6;
		end = strchr(pos, '\n');
		if (end == NULL) {
			goto errout;
		}
		*end = '\0';
		char *tmpflags = zalloc(end - pos + 1);
		if (tmpflags == NULL) {
			goto errout;
		}
		strlcpy(tmpflags, pos, end - pos + 1);
		bool device_supported = brcm_get_security_from_flags(tmpflags, &info->sec_modes, &info->num_sec_modes);
		if (tmpflags) {
			free(tmpflags);
			tmpflags = NULL;
		}
		if (!device_supported) {
			goto errout;
		}
		pos = end + 1;
		// Extract ssid
		pos = strstr(pos, WPA_VALUE_SSID);
		if (pos == NULL) {
			goto errout;
		}
		pos += 5;
		end = strchr(pos, '\n');
		if (end == NULL) {
			goto errout;
		}
		*end = '\0';
		info->ssid_len = printf_decode(info->ssid, BRCM_SSID_LEN + 1, pos);
		pos = end + 1;

	}
	if (pbuf) {
		free(pbuf);
		pbuf = NULL;
		return TRUE;
	}
errout:
	if (pbuf) {
		free(pbuf);
		pbuf = NULL;
	}
	return FALSE;
}

static int str_starts(const char *src, const char *match)
{
	return strncmp(src, match, strlen(match)) == 0;
}

static void brcm_scan_event_handler(const char *str, const char *event)
{
	VPRINT("BRCM_API scan_event_handler - looking for: %s\n", event);

	if (str_starts(str, event)) {
		DPRINT("\n wanted event received %s\n", str);
		if (str_starts(str, WPA_EVENT_SCAN_RESULTS)) {
			g_scanning = 0;
			if (g_scan_result_handler) {
				/* Save handler for last callback */
				network_scan_result_handler_t tmpHander = g_scan_result_handler;
				brcm_reason_t reason;
				memset(&reason, 0, sizeof(brcm_reason_t));
				/* Since we are finished with scanning scan callback needs to be cleared */
				g_scan_result_handler = NULL;
#ifdef CONFIG_SCSC_WLAN_AUTO_RECOVERY
				g_recovery_data.scan_result_handler = NULL;
#endif
				tmpHander(&reason);
			}
		}
	}
}

bool brcm_event_recieved(const char *str, const char *event)
{
	bool recieved = FALSE;
	if (str_starts(str, event)) {
		VPRINT("Event found: %s\n", event);
		recieved = TRUE;
	}
	return recieved;
}

void brcm_sta_disconnect_event_handler(const char *str, brcm_reason_t *reason)
{
	const char *s;
	VPRINT("BRCM_API - got %s\n", str);
	memset(reason, 0, sizeof(brcm_reason_t));

	s = strstr(str, WPA_VALUE_BSSID);
	if (s != NULL) {
		s += 6;
		memcpy(&(reason->bssid), s, 17);
		s += 17;
	} else {
		memset(reason->bssid, 0, sizeof(reason->bssid));
		s = str;
	}
	s = strstr(s, WPA_VALUE_REASON);
	if (s != NULL) {
		reason->reason_code = (uint32_t)strtol(s + 7, NULL, 10);
	} else {
		s = str;
	}
	s = strstr(s, WPA_VALUE_LOACLLY_GENERATED);
	if (s == NULL) {
		// Reason not found or locally generated
		reason->locally_generated = 0;
	} else {
		reason->locally_generated = (uint8_t)strtol(s + 18, NULL, 10);
	}
	VPRINT("BRCM_API reason_code: %d locally_generated: %d\n", reason->reason_code, reason->locally_generated);

	VPRINT("BRCM_API send link_down\n");
}

static void brcm_ap_disconnect_event_handler(const char *result)
{
	brcm_reason_t reason;
	memset(&reason, 0, sizeof(brcm_reason_t));
	result += sizeof(AP_STA_DISCONNECTED) - 1;	// Exclude null-termination
	if (strlen(result) >= 17) {	// bssid is a 17 character string
		memcpy(&(reason.bssid), result, 17);
	}
	if (g_link_down) {
		VPRINT("BRCM_API brcm_handle_disconnect send link_down\n");
		g_link_down(&reason);
	}
}

#ifdef CONFIG_SCSC_WLAN_AUTO_RECOVERY
static void brcm_clean_recover(void)
{
	// Terminate recovery thread
	g_recovery_running = 0;
	sem_post(&g_sem_recover);
	// Free join_config security
	if (g_recovery_data.security) {
		free(g_recovery_data.security);
		g_recovery_data.security = NULL;
	}
	// Free ap_config security
	brcm_ap_config_t *t_ap_config = &g_recovery_data.ap_config;
	if (t_ap_config->security) {
		free(t_ap_config->security);
		t_ap_config->security = NULL;
	}
	// Free ap_config vsie
	if (t_ap_config->vsie) {
		if (t_ap_config->vsie->content) {
			free(t_ap_config->vsie->content);
			t_ap_config->vsie->content = NULL;
		}
		free(t_ap_config->vsie);
		t_ap_config->vsie = NULL;
	}
	// Clean g_recovery_data
	memset(&g_recovery_data, 0, sizeof(brcm_recovery_data_t));
}

static void brcm_verify_back(void)
{
	g_link_up = g_recovery_data.link_up;
	g_link_down = g_recovery_data.link_down;
	if (g_recovery_data.scan_result_handler) {
		if (brcm_start_scan() != BRCM_STATUS_SUCCESS) {
			EPRINT("Initiate scan failed!\n");
		}
	}
	sem_post(&g_sem_api_block);
	g_recovering = 0;
	EPRINT("Back to original state \n");
}

static void brcm_verify_recovered(void)
{
	if (g_recovering) {
		if (g_recovery_data.recovering_state == g_state) {
			// Back to normal
			brcm_verify_back();

			if (g_recovery_data.recovering_state != g_recovery_data.old_state) {
				brcm_reason_t reason;
				memset(&reason, 0, sizeof(brcm_reason_t));
				switch (g_recovery_data.old_state) {
				case BRCM_WIFIAPI_STATE_STA_DISCONNECTING:
					VPRINT("BRCM_WIFIAPI_STATE_STA_DISCONNECTING \n");
					if (g_link_down) {
						VPRINT("sta was disconnecting - send link_down\n");
						g_link_down(&reason);
					}
					break;
				case BRCM_WIFIAPI_STATE_STA_CONNECTING:
					VPRINT("BRCM_WIFIAPI_STATE_STA_CONNECTING \n");
					if (g_link_up) {
						VPRINT("sta was connecting - send link up\n");
						g_link_up(&reason);
					}
					break;
				case BRCM_WIFIAPI_STATE_AP_CONNECTED:
					if (g_link_down) {
						VPRINT("ap was connected - send link_down\n");
						g_link_down(&reason);
					}
					break;
				case BRCM_WIFIAPI_STATE_AP_ENABLING:
					VPRINT("ap was enabling \n");
					break;
				case BRCM_WIFIAPI_STATE_AP_DISABLING:
					VPRINT("ap was disabling \n");
					break;
				default:
					DPRINT("Recover current state: %s - recovering from: %s\n", brcm_state_strings[g_state], brcm_state_strings[g_recovery_data.old_state]);
					break;
				}
			}

		} else {
			DPRINT("Recover current state: %s - recovering from: %s\n", brcm_state_strings[g_state], brcm_state_strings[g_recovery_data.old_state]);
		}
	}
}

static void brcm_reinitiate_state(void)
{
	// Re-initiate state
	int8_t res_api = BRCM_STATUS_ERROR;
	DPRINT("Re-initiate - old_state: %s\n", brcm_state_strings[g_recovery_data.old_state]);
	switch (g_recovery_data.old_state) {
	case BRCM_WIFIAPI_STATE_NOT_STARTED:
		/* Should not happen - nothing to do! */
		brcm_verify_back();
		break;
	case BRCM_WIFIAPI_STATE_STA_DISCONNECTING:
		g_recovery_data.recovering_state = BRCM_WIFIAPI_STATE_SUPPLICANT_RUNNING;
		res_api = brcm_api_start(BRCM_WIFI_STATION_IF, NULL);
		DPRINT("WiFiStart 0 returned result=%d \n", res_api);
		if (res_api != BRCM_STATUS_SUCCESS) {
			EPRINT("Not able to start Wi-Fi 0! \n");
		}
		brcm_verify_recovered();
		break;
	case BRCM_WIFIAPI_STATE_SUPPLICANT_RUNNING:
		/* Setup mode as station */
		res_api = brcm_api_start(BRCM_WIFI_STATION_IF, NULL);
		DPRINT("WiFiStart 1 returned result=%d \n", res_api);
		if (res_api != BRCM_STATUS_SUCCESS) {
			EPRINT("Not able to start Wi-Fi 1! \n");
		}
		brcm_verify_back();
		break;
	case BRCM_WIFIAPI_STATE_AP_CONNECTED:
	/* fall through */
	case BRCM_WIFIAPI_STATE_AP_ENABLING:
		g_recovery_data.recovering_state = BRCM_WIFIAPI_STATE_AP_ENABLED;
	/* fall through */
	case BRCM_WIFIAPI_STATE_AP_ENABLED: {
		/* Setup mode */
		brcm_ap_config_t *pl_ap_config = brcm_get_ap_config();
		res_api = brcm_api_start(g_recovery_data.old_interface_type, pl_ap_config);
		free(pl_ap_config);
		DPRINT("WiFiStart 2 returned result=%d \n", res_api);
		if (res_api != BRCM_STATUS_SUCCESS) {
			EPRINT("Not able to start Wi-Fi 2! \n");
		}
		break;
	}
	case BRCM_WIFIAPI_STATE_STA_CONNECTING:
		g_recovery_data.recovering_state = BRCM_WIFIAPI_STATE_STA_CONNECTED;
	/* fall through */
	case BRCM_WIFIAPI_STATE_STA_CONNECTED:
		res_api = brcm_api_start(g_recovery_data.old_interface_type, NULL);
		DPRINT("WiFiStart returned result=%d \n", res_api);
		if (res_api == BRCM_STATUS_SUCCESS) {
			DPRINT("Restarted Wi-Fi - initiate Join! \n");
			VPRINT("Saved ssid: %s\n", g_recovery_data.ssid);
			VPRINT("Saved bssid: %s\n", g_recovery_data.bssid);

			if (g_recovery_data.security) {
				VPRINT("Saved security settings: security mode %d\n", g_recovery_data.security->secmode);
			}
			res_api = brcm_join_network(g_recovery_data.ssid, g_recovery_data.ssid_len, g_recovery_data.bssid, g_recovery_data.security);
//                    WiFiNetworkJoin(g_recovery_data.ssid, g_recovery_data.ssid_len,
//                    g_recovery_data.bssid, g_recovery_data.security);
			if (res_api != BRCM_STATUS_SUCCESS) {
				EPRINT("Not able to join network! \n");
			}
		} else {
			EPRINT("Not able to start Wi-Fi 4! \n");
		}
		break;
	case BRCM_WIFIAPI_STATE_AP_DISABLING:
	/* fall through */
	case BRCM_WIFIAPI_STATE_TERMINATING:
		/* Assuming we are shutting down */
		g_state = BRCM_WIFIAPI_STATE_NOT_STARTED;
		brcm_verify_back();
		break;
	default:
		break;
	}
}

void brcm_recovery_thread_handler(void *param)
{
	g_recovery_running = 1;
	sem_init(&g_sem_recover, 0, 0);

	while (g_recovery_running) {
		sem_wait(&g_sem_recover);
		DPRINT("Recover semaphore released\n");
		// Re-initiate state
		if (g_recovering) {
			brcm_reinitiate_state();
		}
	}
	sem_destroy(&g_sem_recover);

	VPRINT("BRCM_API pthread_exit %d \n", g_recovery_thread);
	pthread_detach(g_recovery_thread);
	pthread_exit(&g_recovery_thread);
	g_recovery_thread = 0;
}

static bool brcm_recovery_handler(const char *str)
{
	bool handled = FALSE;

	if (brcm_event_recieved(str, WPA_EVENT_HANGED) && g_state != BRCM_WIFIAPI_STATE_TERMINATING) {
		EPRINT("BRCM-RECOVERY-EVENT received\n");
		// Initial entering here - start recovering handling
		LOCKUNLOCK_CRITICAL;
		VPRINT("Current state: %s\n", brcm_state_strings[g_state]);
		g_recovery_data.old_state = g_state;
		g_recovery_data.recovering_state = g_state;
		handled = TRUE;
		g_state = BRCM_WIFIAPI_STATE_RECOVERING;
		if (brcm_terminate_supplicant() != BRCM_STATUS_SUCCESS) {
			EPRINT("Not able to terminate supplicant! \n");
		}
	}
	return handled;
}
#endif							// CONFIG_SCSC_WLAN_AUTO_RECOVERY

void brcm_monitor_thread_handler(void *param)
{
	g_running = 1;
	char *result = NULL;
	uint8_t join_count = 0;

	while (g_running) {
		if (brcm_recv_pending(&result) && (result != NULL)) {
			char *tmp = result;
			brcm_reason_t reason;
			memset(&reason, 0, sizeof(brcm_reason_t));
			DPRINT("BRCM_API brcm_monitor_thread_handler event - %s\n", result);
			result = strchr(result, '>');
			if (result == NULL) {
				if (tmp) {
					free(tmp);
					tmp = NULL;
				}
				continue;
			}
			result++;
#ifdef CONFIG_SCSC_WLAN_AUTO_RECOVERY
			if (brcm_recovery_handler(result) == FALSE)
#endif
			{
				// Checking events received
				if (g_scanning) {
					brcm_scan_event_handler(result, WPA_EVENT_SCAN_RESULTS);
				}

				VPRINT("Switch - current state: %s\n", brcm_state_strings[g_state]);
				switch (g_state) {
				case BRCM_WIFIAPI_STATE_SUPPLICANT_RUNNING:
					// Handling reconnect after remote disconnect
					if (brcm_event_recieved(result, WPA_EVENT_CONNECTED)) {
						brcm_check_status(reason.ssid, &reason.ssid_len, reason.bssid);
						brcm_get_network(reason.ssid, reason.ssid_len, &g_network_id);
						g_state = BRCM_WIFIAPI_STATE_STA_CONNECTED;
						brcm_set_scan_interval(BRCM_SCAN_INTERVAL); // connected so lets set scan interval back and save power
						if (g_link_up) {
							VPRINT("BRCM_API send link_up\n");
							g_link_up(&reason);
						}
					}
					// To ensure join counter is reset in switch mode
					join_count = 0;
					break;
				case BRCM_WIFIAPI_STATE_AP_ENABLING:
					// AP mode setup and select network
					if (brcm_event_recieved(result, AP_EVENT_ENABLED)) {
						g_state = BRCM_WIFIAPI_STATE_AP_ENABLED;
						sem_post(&g_sem_ap_mode);
					} else if (brcm_event_recieved(result, AP_EVENT_DISABLED)) {
						// The network setup failed
						g_state = BRCM_WIFIAPI_STATE_SUPPLICANT_RUNNING;
						sem_post(&g_sem_ap_mode);
					} else {
						VPRINT("Info: Event not handled %s in current state %s\n", result, brcm_state_strings[g_state]);
					}
					break;
				case BRCM_WIFIAPI_STATE_AP_ENABLED:
					if (brcm_event_recieved(result, AP_STA_CONNECTED)) {
						result += sizeof(AP_STA_CONNECTED) - 1;
						if (strlen(result) >= 17) {	// bssid is a 17 character string
							memcpy(&(reason.bssid), result, 17);	// Exclude null-termination
						}
						g_num_sta_connected++;
						if (g_num_sta_connected == 1) {
							g_state = BRCM_WIFIAPI_STATE_AP_CONNECTED;

						}
						if (g_link_up) {
							VPRINT("BRCM_API brcm_link_event_handler send link_up\n");
							g_link_up(&reason);
						}
					} else {
						VPRINT("Info: Event not handled %s in current state %s\n", result, brcm_state_strings[g_state]);
					}
					break;
				case BRCM_WIFIAPI_STATE_AP_CONNECTED:
					if (brcm_event_recieved(result, AP_STA_DISCONNECTED)) {
						g_num_sta_connected--;
						if (g_num_sta_connected == 0) {
							g_state = BRCM_WIFIAPI_STATE_AP_ENABLED;
						}
						/* Always call brcm_ap_disconnect_event_handler() at the end of function
						 * as link_down callback handler is called */
						brcm_ap_disconnect_event_handler(result);
					} else {
						VPRINT("Info: Event not handled %s in current state %s\n", result, brcm_state_strings[g_state]);
					}
					break;
				case BRCM_WIFIAPI_STATE_AP_DISABLING:
					// AP stop sent
					if (brcm_event_recieved(result, AP_EVENT_DISABLED)) {
						g_state = BRCM_WIFIAPI_STATE_SUPPLICANT_RUNNING;
						sem_post(&g_sem_ap_mode);
					} else if (brcm_event_recieved(result, AP_EVENT_ENABLED)) {
						g_state = BRCM_WIFIAPI_STATE_AP_ENABLED;
						sem_post(&g_sem_ap_mode);
					} else {
						VPRINT("Info: Event not handled %s in current state %s\n", result, brcm_state_strings[g_state]);
					}
					break;
				case BRCM_WIFIAPI_STATE_STA_CONNECTING: {
					bool event_handled = FALSE;
					if (brcm_event_recieved(result, WPA_EVENT_CONNECTED)) {
						brcm_check_status(reason.ssid, &reason.ssid_len, reason.bssid);
						g_state = BRCM_WIFIAPI_STATE_STA_CONNECTED;
						// connected so lets set scan interval back to limit power consumption
						brcm_set_scan_interval(BRCM_SCAN_INTERVAL);
						event_handled = TRUE;
					} else if (brcm_event_recieved(result, WPA_EVENT_NETWORK_NOT_FOUND)) {
						/* Assumed to be because network with specification setup is not
						 * found in scan results - handle as error - disable network */
						if (join_count == BRCM_STA_JOIN_SCAN_ATTEMPT) {
							reason.reason_code = BRCM_REASON_NETWORK_CONFIGURATION_NOT_FOUND;
							event_handled = TRUE;
						} else {
							join_count++;
						}
					} else if (brcm_event_recieved(result, WPA_EVENT_TEMP_DISABLED)) {
						reason.reason_code = BRCM_REASON_NETWORK_AUTHENTICATION_FAILED;
						event_handled = TRUE;
					} else if (brcm_event_recieved(result, BRCM_EVENT_ASSOCIATION_REQ_FAILED)) {
						reason.reason_code = BRCM_REASON_ASSOCIATION_REQ_FAILED;
						event_handled = TRUE;
					} else {
						VPRINT("Info: Event not handled %s in current state %s\n", result, brcm_state_strings[g_state]);
					}
					if (event_handled) {
						join_count = 0;
						if (reason.reason_code) {
							VPRINT("reason.reason_code=%d\n", reason.reason_code);
							// Connection failed change back state
							g_state = BRCM_WIFIAPI_STATE_SUPPLICANT_RUNNING;
							if (g_network_id) {
								brcm_remove_network(g_network_id);
								free(g_network_id);
								g_network_id = NULL;
							}
						}
						if (g_link_up) {
							VPRINT("BRCM_API brcm_link_event_handler send link_up\n");
							g_link_up(&reason);
						}
					}
					break;
				}
				case BRCM_WIFIAPI_STATE_STA_CONNECTED:
					if (brcm_event_recieved(result, WPA_EVENT_DISCONNECTED)) {
						brcm_sta_disconnect_event_handler(result, &reason);
						g_state = BRCM_WIFIAPI_STATE_SUPPLICANT_RUNNING;
						if (g_link_down) {
							VPRINT("BRCM_API brcm_link_event_handler send link_down\n");
							g_link_down(&reason);
						}
						// TODO: clean join info for recovery
					} else {
						VPRINT("Info: Event not handled %s in current state %s\n", result, brcm_state_strings[g_state]);
					}
					break;
				case BRCM_WIFIAPI_STATE_STA_DISCONNECTING:
					if (brcm_event_recieved(result, WPA_EVENT_DISCONNECTED)) {
						brcm_sta_disconnect_event_handler(result, &reason);
						g_state = BRCM_WIFIAPI_STATE_SUPPLICANT_RUNNING;
						/* start by disabling all previous networks to make sure
						 * they are not running simultaneous */
						uint8_t tmp_result = brcm_disable_all_networks();
						if (tmp_result != BRCM_STATUS_SUCCESS) {
							EPRINT("disable networks - failed\n");
						}
						if (g_network_id) {
							brcm_remove_network(g_network_id);
							free(g_network_id);
							g_network_id = NULL;
						}
						if (g_link_down) {
							VPRINT("BRCM_API brcm_link_event_handler send link_down\n");
							g_link_down(&reason);
						}
						// Release sem_wait after finished removing the network
						sem_post(&g_sem_disconnect);
						// TODO: clean join info for recovery
					} else {
						VPRINT("Info: Event not handled %s in current state %s\n", result, brcm_state_strings[g_state]);
					}
					break;
				case BRCM_WIFIAPI_STATE_TERMINATING:
					if (brcm_event_recieved(result, WPA_EVENT_TERMINATING)) {
						VPRINT("WPA_EVENT_TERMINATING Received\n");
						g_running = 0;
						sem_post(&g_sem_terminate);
					} else {
						VPRINT("Info: Event not handled %s in current state %s\n", result, brcm_state_strings[g_state]);
					}
					break;
				case BRCM_WIFIAPI_STATE_RECOVERING:
					if (brcm_event_recieved(result, WPA_EVENT_TERMINATING) == TRUE) {
						// Secondary entry here - continue shut down and re-initiate
						VPRINT("WPA_EVENT_TERMINATING Received in recover handler \n");
						pid_t r_task_id = -1;
						int status = -1;
						UNUSED(status);
						r_task_id = waitpid(g_task, &status, NULL);
						DPRINT("Wait for supplicant task to terminate\n");
						if (r_task_id != g_task) {
							DPRINT("    result: %d, status: %d\n", r_task_id, status);
							VPRINT("Error waiting for task termination - sleep and continue \n");
							usleep(100000);
						}
						// if monitor still running - stop running
						if (g_running) {
							VPRINT(" stop running\n");
							// Will end while loop in monitoring thread and exit thread
							g_running = 0;
						}
						// Disconnect wpa_ctrl connection
						if (brcm_wpa_close(TRUE) != BRCM_STATUS_SUCCESS) {
							EPRINT("Error: still running after disconnect. \n");
						}
						// CLean up local
						brcm_deinit();
#ifdef CONFIG_SCSC_WLAN_AUTO_RECOVERY
						g_recovering = 1;
						sem_post(&g_sem_recover);
#endif
					} else {
						VPRINT("Info: Event not handled %s in current state %s\n", result, brcm_state_strings[g_state]);
					}
					break;

				default:
					EPRINT("Should not happen as the state is not valid\n");
					break;
				}
			}
			if (tmp) {
				free(tmp);
				tmp = NULL;
			}
#ifdef CONFIG_SCSC_WLAN_AUTO_RECOVERY
			brcm_verify_recovered();
#endif
		}
	}
	VPRINT("BRCM_API pthread_exit %d \n", g_monitoring_thread);
	g_monitoring_thread = 0;
}

/* Receive monitor message */
static int8_t brcm_recv_pending(char **result)
{
	if (g_ctrl_conn == NULL) {
		if (g_running) {
			brcm_wpa_reopen();
		}
		return 0;
	}
	while (wpa_ctrl_pending(g_ctrl_conn) > 0) {
		size_t len = WPA_BUFFER_SIZE - 1;
		char *buf = zalloc(len + 1);
		if (buf == NULL) {
			return 0;
		} else {
			if (wpa_ctrl_recv_monitor(g_ctrl_conn, buf, &len) == 0) {
				buf[len] = '\0';
				*result = buf;
				DPRINT("brcm_recv_pending received: %d bytes\n", len);
				return 1;
			} else {
				EPRINT("BRCM_API Could not read pending message.\n");
				UNUSED(buf);	//prevent lint warning
				if (buf) {
					free(buf);
					// prevent lint warning so no buf = NULL;
				}
			}
		}
	}
	if (wpa_ctrl_pending(g_ctrl_conn) < 0) {
		if (g_running) {
			brcm_wpa_reopen();
		}
		return 0;
	}
	return 0;
}

static int8_t brcm_wpa_open(char *network_path)
{
	int8_t result = BRCM_STATUS_ERROR;
	/* The control connection */
	VPRINT("BRCM_API wpa_ctrl_connect before open\n");
	if (BRCM_WIFI_API_DEBUG_SLEEP) {
		brcm_demo_app_sleep(1, NULL);
	}
	g_ctrl_conn = wpa_ctrl_open(network_path);
	if (g_ctrl_conn == NULL) {
		EPRINT("BRCM_API Failed to connect to interface: %s \n", CTRL_IFNAME);
		return result;
	} else {
		VPRINT("BRCM_API wpa_ctrl_connect success\n");
	}
	if (BRCM_WIFI_API_DEBUG_SLEEP) {
		brcm_demo_app_sleep(1, NULL);
	}
	/* The monitor connection and thread setup */
	if (wpa_ctrl_attach(g_ctrl_conn) == 0) {
		g_wpa_attached = 1;
		VPRINT("BRCM_API wpa_ctrl_attach success\n");
		if (BRCM_WIFI_API_DEBUG_SLEEP) {
			brcm_demo_app_sleep(1, NULL);
		}
	} else {
		EPRINT("BRCM_API Failed to open monitor connection " "through control interface\n");
		return result;
	}
	if (pthread_create(&g_monitoring_thread, NULL, (void *)brcm_monitor_thread_handler, NULL)) {
		EPRINT("BRCM_API Failed to create thread\n");
	} else {
		result = BRCM_STATUS_SUCCESS;
		pthread_setname_np(g_monitoring_thread, "Wi-Fi API monitor");
		pthread_detach(g_monitoring_thread);
		VPRINT("BRCM_API Monitoring thread created successfully\n");
	}
	if (BRCM_WIFI_API_DEBUG_SLEEP) {
		brcm_demo_app_sleep(1, NULL);
	}

	return result;
}

static int8_t brcm_wpa_close(bool terminate)
{
	int8_t result = BRCM_STATUS_ERROR;

	if (g_ctrl_conn != NULL) {
		if (g_wpa_attached && !terminate) {
			// If we are terminating the supplicant it is not possible to send detach
			if (wpa_ctrl_detach(g_ctrl_conn) == 0) {
				VPRINT("BRCM_API wpa_ctrl_detach success\n");
			}
			g_wpa_attached = 0;
			VPRINT("BRCM_API g_mon_conn closed\n");
		}
		if (BRCM_WIFI_API_DEBUG_SLEEP) {
			brcm_demo_app_sleep(1, "detach and close wpa_ctrl");
		}
		wpa_ctrl_close(g_ctrl_conn);
		g_ctrl_conn = NULL;
		VPRINT("BRCM_API g_ctrl_conn closed\n");
		result = BRCM_STATUS_SUCCESS;
	} else {
		VPRINT("BRCM_API wpa_ctrl iface already closed!\n");
	}

	if (BRCM_WIFI_API_DEBUG_SLEEP) {
		brcm_demo_app_sleep(1, "end of disconnect wpa_ctrl");
	}
	return result;
}

static void brcm_wpa_reopen(void)
{
	VPRINT("BRCM_API Trying to reconnect...\n");
	(void)brcm_wpa_close(FALSE);
	(void)brcm_wpa_open(CTRL_IFNAME);
}

static int8_t brcm_get_network(uint8_t *ssid, uint8_t ssid_len, char **network_id)
{
	int8_t result = BRCM_STATUS_ERROR;
	char *pbuf = NULL;
	char command[WPA_COMMAND_MAX_SIZE] = { 0 };
	char ssid_formated[WPA_MAX_SSID_LEN];
	printf_encode(ssid_formated, WPA_MAX_SSID_LEN - 1, ssid, ssid_len);
	// List networks to see if already available
	snprintf(command, WPA_COMMAND_MAX_SIZE, "%s", WPA_COMMAND_LIST_NETWORKS);
	VPRINT("BRCM_API get_network list: %s\n", command);
	pbuf = brcm_send_request(command, NULL);
	if (pbuf != NULL) {
		VPRINT("%s", pbuf);
	}
	if (pbuf) {
		char *pos, *tab, *name;
		pos = strchr(pbuf, '\n');
		if (pos == NULL) {
			goto errout;
		}
		pos = pos + 1;
		// Find network id if available
		name = strstr(pos, ssid_formated);
		if (name) {
			VPRINT("BRCM_API existing network found\n");
			tab = name - 1;
			*tab = '\0';		// Replace tab with END-OF-STRING
			pos = name - 2;		// NOTE: this only work if the ID is 2 digits or less.
			if (strncmp((pos - 1), "\n", 1) == 0) {
				// One digit ID
				*network_id = strdup(pos);
			} else if (strncmp((pos - 2), "\n", 1) == 0) {
				// Two digit ID
				*network_id = strdup(pos - 1);
			} else {
				EPRINT("BRCM_API - no newline - should not happen\n");
				goto errout;
			}
		} else {
			goto errout;
		}
		result = BRCM_STATUS_SUCCESS;
		if (pbuf) {
			free(pbuf);
			pbuf = NULL;
		}
	}
	return result;
errout:
	if (pbuf) {
		free(pbuf);
		pbuf = NULL;
	}
	return result;
}

static int8_t brcm_check_status(uint8_t *ssid, int8_t *ssid_len, char *bssid)
{
	int8_t result = BRCM_STATUS_ERROR;
	char *pbuf = NULL;
	char command[WPA_COMMAND_MAX_SIZE] = { 0 };
	char *pos, *end;
	*ssid_len = 0;

	// Check state of network possibly already connected
	snprintf(command, WPA_COMMAND_MAX_SIZE, "%s", WPA_COMMAND_STATUS);
	VPRINT("BRCM_API check_status select: %s\n", command);
	pbuf = brcm_send_request(command, NULL);
	if (pbuf) {
		VPRINT("  Response: %s", pbuf);
		pos = strstr(pbuf, WPA_VALUE_WPA_STATE);
		if (pos == NULL) {
			goto errout;
		}
		pos += 10;
		if (str_starts(pos, WPA_STATE_DISCONNECTED)) {
			//g_state = BRCM_WIFIAPI_STATE_SUPPLICANT_RUNNING;
		} else if (str_starts(pos, WPA_STATE_COMPLETED)) {
			pos = strstr(pbuf, WPA_VALUE_BSSID);
			if (pos != NULL) {
				pos += 6;
				end = strchr(pos, '\n');
				if (end == NULL) {
					goto errout;
				}
				*end = '\0';
				if (bssid != NULL) {
					memcpy(bssid, pos, 18);    // Including '\0'
				}
				pos = end + 1;
				/* NOTE as ssid is a substring of bssid, we need to skip past the bssid entry */
				pos = strstr(pos, WPA_VALUE_SSID);
			}
			if (pos != NULL) {
				pos += 5;
				end = strchr(pos, '\n');
				if (end == NULL) {
					goto errout;
				}
				*end = '\0';
				if (ssid != NULL) {
					*ssid_len = printf_decode(ssid, BRCM_SSID_LEN + 1, pos);
				}
			}
		} else {				//if(str_starts(pos,WPA_STATE_SCANNING)) {

		}
		result = BRCM_STATUS_SUCCESS;
		if (pbuf) {
			free(pbuf);
			pbuf = NULL;
		}
	}
	return result;

errout:
	if (pbuf) {
		free(pbuf);
		pbuf = NULL;
	}
	return result;
}

static int8_t brcm_set_security(const brcm_security_config_t *sec_config, const char *network_id)
{
	int8_t result = BRCM_STATUS_SECURITY_FAILED;
	// Set security for both AP mode and STA mode
	char *keymgmt = WPA_PARAM_KEY_MGMT_NONE;
	char *authalg = WPA_VALUE_AUTH_ALG_OPEN;

	if (sec_config == NULL) {
		brcm_send_command_str_upto_4(WPA_COMMAND_SET_NETWORK, (char *)network_id, WPA_PARAM_KEY_MGMT_NONE, NULL, &result);
		if (result != BRCM_STATUS_SUCCESS) {
			goto errout;
		}
	} else {
		VPRINT("BRCM_API setup network with security settings: " "security mode %d\n", sec_config->secmode);

		if (sec_config->secmode == BRCM_SEC_MODE_OPEN) {
			keymgmt = WPA_PARAM_KEY_MGMT_NONE;
		} else if (sec_config->secmode == BRCM_SEC_MODE_WEP || sec_config->secmode == BRCM_SEC_MODE_WEP_SHARED) {
			if (brcm_get_op_mode() == BRCM_WIFI_SOFT_AP_IF) {
				DPRINT("BRCM_API set_security - WEP in AP mode - wrong parameter\n");
				result = BRCM_STATUS_PARAM_FAILED;
				goto errout;
			}
			keymgmt = WPA_PARAM_KEY_MGMT_WEP;
		} else if (sec_config->secmode == BRCM_SEC_MODE_EAP) {
			DPRINT("BRCM_API set_security - EAP - wrong parameter\n");
			result = BRCM_STATUS_PARAM_FAILED;
			goto errout;
		} else if (sec_config->secmode == (BRCM_SEC_MODE_WEP | BRCM_SEC_MODE_WEP_SHARED)) {
			DPRINT("BRCM_API set_security - " "WEP OPEN/SHARED cannot be set together - wrong parameter\n");
			result = BRCM_STATUS_PARAM_FAILED;
			goto errout;
		} else if (sec_config->secmode & (BRCM_SEC_MODE_WPA_MIXED | BRCM_SEC_MODE_WPA2_MIXED)) {
			// WPA or WPA2
			char *proto = NULL;
			char *pairwise = NULL;

			keymgmt = WPA_PARAM_KEY_MGMT_WPA_PSK;

			// setting proto
			if ((sec_config->secmode & BRCM_SEC_MODE_WPA_MIXED) && (sec_config->secmode & BRCM_SEC_MODE_WPA2_MIXED)) {
				proto = WPA_VALUE_PROTO_MIXED;
			} else if (sec_config->secmode & BRCM_SEC_MODE_WPA2_MIXED) {
				proto = WPA_VALUE_PROTO_RSN;
			} else {
				proto = WPA_VALUE_PROTO_WPA;
			}

			brcm_send_command_str_upto_4(WPA_COMMAND_SET_NETWORK, (char *)network_id, WPA_PARAM_PROTO, proto, &result);
			if (result != BRCM_STATUS_SUCCESS) {
				goto errout;
			}
			// setting cipher
			if (sec_config->secmode == BRCM_SEC_MODE_WPA_MIXED || sec_config->secmode == BRCM_SEC_MODE_WPA2_MIXED || sec_config->secmode == (BRCM_SEC_MODE_WPA_MIXED | BRCM_SEC_MODE_WPA2_MIXED)) {
				pairwise = WPA_VALUE_CIPHER_MIXED;
			} else if (sec_config->secmode == BRCM_SEC_MODE_WPA_CCMP || sec_config->secmode == BRCM_SEC_MODE_WPA2_CCMP) {
				pairwise = WPA_VALUE_CIPHER_CCMP;
			} else {
				pairwise = WPA_VALUE_CIPHER_TKIP;
			}
			brcm_send_command_str_upto_4(WPA_COMMAND_SET_NETWORK, (char *)network_id, WPA_PARAM_PAIRWISE, pairwise, &result);
			if (result != BRCM_STATUS_SUCCESS) {
				goto errout;
			}

			if (brcm_get_op_mode() == BRCM_WIFI_SOFT_AP_IF) {
				brcm_send_command_str_upto_4(WPA_COMMAND_SET_NETWORK, (char *)network_id, WPA_PARAM_GROUP, pairwise, &result);
				if (result != BRCM_STATUS_SUCCESS) {
					goto errout;
				}
			}
		} else {
			DPRINT("BRCM_API set_security - wrong parameter\n");
			result = BRCM_STATUS_PARAM_FAILED;
			goto errout;
		}

		brcm_send_command_str_upto_4(WPA_COMMAND_SET_NETWORK, (char *)network_id, keymgmt, NULL, &result);
		if (result != BRCM_STATUS_SUCCESS) {
			goto errout;
		}
		// We always set auth_alg
		if (sec_config->secmode == BRCM_SEC_MODE_WEP_SHARED) {
			authalg = WPA_VALUE_AUTH_ALG_SHARED;
		}
		//We always set auth_alg
		brcm_send_command_str_upto_4(WPA_COMMAND_SET_NETWORK, (char *)network_id, WPA_PARAM_AUTH_ALG, authalg, &result);
		if (result != BRCM_STATUS_SUCCESS) {
			goto errout;
		}

		if (sec_config->secmode != BRCM_SEC_MODE_OPEN && sec_config->passphrase[0] != '\0') {
			char command[WPA_COMMAND_MAX_SIZE] = { 0 };
			size_t len = strlen(sec_config->passphrase);
			if (sec_config->secmode == BRCM_SEC_MODE_WEP || sec_config->secmode == BRCM_SEC_MODE_WEP_SHARED) {
				// wep_key
				VPRINT("BRCM_API set_security WEP key: %s\n", sec_config->passphrase);
				// Validate length ascii (5 or 13) or hex (10 or 26)
				if (sec_config->passphrase[0] == '"') {
					// ascii specified
					if (len >= BRCM_WIFI_WEP_ASCII_KEY_MIN + 2 && len <= BRCM_WIFI_WEP_ASCII_KEY_MAX + 2) {
						snprintf(command, WPA_COMMAND_MAX_SIZE, "%s%s %s%s", WPA_COMMAND_SET_NETWORK, network_id, WPA_PARAM_WEPKEY, sec_config->passphrase);
					} else {
						DPRINT("BRCM_API set_security WEP - wrong ASCII key length\n");
						result = BRCM_STATUS_PARAM_FAILED;
						goto errout;
					}
				} else {
					// hex specified
					if (len >= BRCM_WIFI_WEP_HEX_KEY_MIN && len <= BRCM_WIFI_WEP_HEX_KEY_MAX) {
						snprintf(command, WPA_COMMAND_MAX_SIZE, "%s%s %s%s", WPA_COMMAND_SET_NETWORK, network_id, WPA_PARAM_WEPKEY, sec_config->passphrase);
					} else {
						DPRINT("BRCM_API set_security WEP - wrong HEX key length\n");
						result = BRCM_STATUS_PARAM_FAILED;
						goto errout;
					}
				}
			} else {
				// wpa_key
				VPRINT("BRCM_API set_security WPA key: %s\n", sec_config->passphrase);
				// Validate length ascii (8 or 63)
				if (len >= BRCM_WIFI_WPA_ASCII_KEY_MIN && len <= BRCM_WIFI_WPA_ASCII_KEY_MAX) {
					snprintf(command, WPA_COMMAND_MAX_SIZE, "%s%s %s\"%s\"", WPA_COMMAND_SET_NETWORK, network_id, WPA_PARAM_PSK, sec_config->passphrase);
				} else {
					DPRINT("BRCM_API set_security WPA - wrong key length\n");
					result = BRCM_STATUS_PARAM_FAILED;
					goto errout;
				}
			}
			char *pbuf = brcm_send_request(command, &result);
			if (pbuf) {
				free(pbuf);
				pbuf = NULL;
				if (result != BRCM_STATUS_SUCCESS) {
					goto errout;
				}
			}
		} else {
			DPRINT("BRCM_API set_security key missing (or set but not needed)\n");
			goto errout;
		}
	}
errout:
	return result;
}

#ifdef CONFIG_SCSC_WLAN_AUTO_RECOVERY
static brcm_ap_config_t *brcm_get_ap_config(void)
{

	// Clean up structure
	brcm_ap_config_t *p_ap_config = zalloc(sizeof(brcm_ap_config_t));

	if (p_ap_config == NULL) {
		EPRINT("Memory allocation failed \n");
	} else {
		brcm_ap_config_t *tmp_ap_config = &g_recovery_data.ap_config;
		memcpy(p_ap_config, tmp_ap_config, sizeof(brcm_ap_config_t));

		if (tmp_ap_config->security) {
			p_ap_config->security = zalloc(sizeof(brcm_security_config_t));
			if (p_ap_config->security == NULL) {
				EPRINT("Memory allocation failed \n");
			} else {
				memcpy(p_ap_config->security, tmp_ap_config->security, sizeof(brcm_security_config_t));
			}
		}
		if (tmp_ap_config->vsie) {
			p_ap_config->vsie = zalloc(sizeof(brcm_vendor_ie_t));
			if (p_ap_config->vsie == NULL) {
				EPRINT("Memory allocation failed \n");
			} else {
				memcpy(p_ap_config->vsie, tmp_ap_config->vsie, sizeof(brcm_vendor_ie_t));
			}
			p_ap_config->vsie->content = zalloc(tmp_ap_config->vsie->content_length);
			if (p_ap_config->vsie->content == NULL) {
				EPRINT("Memory allocation failed \n");
			} else {
				memcpy(p_ap_config->vsie->content, tmp_ap_config->vsie->content, tmp_ap_config->vsie->content_length);
			}
		}
	}
	return p_ap_config;
}

static void brcm_save_ap_config(brcm_ap_config_t *ap_config)
{

	// Clean up structure
	brcm_ap_config_t *t_ap_config = &g_recovery_data.ap_config;
	if (t_ap_config->security) {
		free(t_ap_config->security);
		t_ap_config->security = NULL;
	}
	if (t_ap_config->vsie) {
		if (t_ap_config->vsie->content) {
			free(t_ap_config->vsie->content);
			t_ap_config->vsie->content = NULL;
		}
		free(t_ap_config->vsie);
		t_ap_config->vsie = NULL;
	}
	memset(&g_recovery_data.ap_config, 0, sizeof(brcm_ap_config_t));

	// Save new config
	memcpy(&g_recovery_data.ap_config, ap_config, sizeof(brcm_ap_config_t));
	if (ap_config->security) {
		t_ap_config->security = zalloc(sizeof(brcm_security_config_t));
		if (t_ap_config->security == NULL) {
			EPRINT("Memory allocation failed \n");
		} else {
			memcpy(t_ap_config->security, ap_config->security, sizeof(brcm_security_config_t));
		}
	}
	if (ap_config->vsie) {
		t_ap_config->vsie = zalloc(sizeof(brcm_vendor_ie_t));
		if (t_ap_config->vsie == NULL) {
			EPRINT("Memory allocation failed \n");
		} else {
			memcpy(t_ap_config->vsie, ap_config->vsie, sizeof(brcm_vendor_ie_t));
		}

		if (ap_config->vsie->content) {
			t_ap_config->vsie->content = zalloc(ap_config->vsie->content_length);
			if (t_ap_config->vsie->content == NULL) {
				EPRINT("Memory allocation failed \n");
			} else {
				memcpy(t_ap_config->vsie->content, ap_config->vsie->content, ap_config->vsie->content_length);
			}
		}
	}
}

static void brcm_save_join(uint8_t *ssid, int ssid_len, uint8_t *bssid, const brcm_security_config_t *sec_config)
{
	// Clean up structure
	if (g_recovery_data.security) {
		free(g_recovery_data.security);
		g_recovery_data.security = NULL;
	}
	memset(&(g_recovery_data.ssid), 0, sizeof(g_recovery_data.ssid));
	memset(&(g_recovery_data.bssid), 0, sizeof(g_recovery_data.bssid));

	// Save new join
	if (ssid) {
		memcpy(&(g_recovery_data.ssid), ssid, sizeof(g_recovery_data.ssid));
		VPRINT("Saved ssid: %s\n", g_recovery_data.ssid);
	}
	g_recovery_data.ssid_len = ssid_len;
	if (bssid) {
		memcpy(&(g_recovery_data.bssid), bssid, sizeof(g_recovery_data.bssid));
		VPRINT("Saved bssid: %s\n", g_recovery_data.bssid);
	}

	if (sec_config) {
		// TODO: security not saved!!!
		g_recovery_data.security = zalloc(sizeof(brcm_security_config_t));
		if (g_recovery_data.security == NULL) {
			EPRINT("Memory allocation failed \n");
		} else {
			memcpy(g_recovery_data.security, sec_config, sizeof(brcm_security_config_t));
			VPRINT("Saved security settings: security mode %d\n", g_recovery_data.security->secmode);
		}
	}
}
#endif

static int8_t brcm_join_network(uint8_t *ssid, int ssid_len, uint8_t *bssid, const brcm_security_config_t *sec_config)
{
	int8_t result = BRCM_STATUS_ERROR;
	char *pbuf = NULL;
	char *network_id = NULL;

	VPRINT("BRCM_API join_network setup network with ssid %s\n", ssid);
	if (sec_config) {
		VPRINT("BRCM_API join_network setup network with security settings: " "security mode %d\n", sec_config->secmode);
	}
	// Find network or add new network
	result = brcm_get_network(ssid, ssid_len, &network_id);
	if ((result != BRCM_STATUS_SUCCESS) && (network_id == NULL) /* Attempt to make SVACE happy */) {
		// Add new network
		char ssid_formatted[WPA_MAX_SSID_LEN];
		char command[WPA_COMMAND_MAX_SIZE] = { 0 };
		snprintf(command, WPA_COMMAND_MAX_SIZE, "%s", WPA_COMMAND_ADD_NETWORK);
		pbuf = brcm_send_request(command, NULL);
		if (pbuf) {
			pbuf[strcspn(pbuf, "\r\n")] = '\0';
			network_id = strdup(pbuf);
			free(pbuf);
			pbuf = NULL;
		} else {
			goto errout;
		}

		// Set network ssid
		printf_encode(ssid_formatted, WPA_MAX_SSID_LEN, ssid, ssid_len);
		memset(command, 0, WPA_COMMAND_MAX_SIZE);
		snprintf(command, WPA_COMMAND_MAX_SIZE, "%s%s %sP\"%s\"", WPA_COMMAND_SET_NETWORK, network_id, WPA_PARAM_SSID, ssid_formatted);
		pbuf = brcm_send_request(command, &result);
		if (pbuf) {
			free(pbuf);
			// removed to prevent lint warning pbuf = NULL;
			if (result != BRCM_STATUS_SUCCESS) {
				goto errout;
			}
		}
		// Set scan_ssid which scans for APs using hidden SSIDs.
		memset(command, 0, WPA_COMMAND_MAX_SIZE);
		snprintf(command, WPA_COMMAND_MAX_SIZE, "%s%s %s%d", WPA_COMMAND_SET_NETWORK, network_id, WPA_PARAM_SCAN_SSID, 1);
		pbuf = brcm_send_request(command, &result);
		if (pbuf) {
			free(pbuf);
			// removed to prevent lint warning pbuf = NULL;
			if (result != BRCM_STATUS_SUCCESS) {
				goto errout;
			}
		}
		// Set network bssid if available
		if (bssid != NULL && bssid[0] != '\0') {
			brcm_send_command_str_upto_4(WPA_COMMAND_SET_NETWORK, network_id, WPA_PARAM_BSSID, (char *)bssid, &result);
			if (result != BRCM_STATUS_SUCCESS) {
				goto errout;
			}
		}
	}
	// Set security
	result = brcm_set_security(sec_config, network_id);
	if (result != BRCM_STATUS_SUCCESS) {
		// remove network
		brcm_send_command_str_upto_4(WPA_COMMAND_REMOVE_NETWORK, network_id, NULL, NULL, NULL);
	} else {
		// Select network (and disable other networks)
		g_state = BRCM_WIFIAPI_STATE_STA_CONNECTING;
		brcm_set_scan_interval(BRCM_SCAN_INTERVAL_CONNECT); //set more agressive scan interval for connections
		brcm_send_command_str_upto_4(WPA_COMMAND_SELECT_NETWORK, network_id, NULL, NULL, &result);
		if (result != BRCM_STATUS_SUCCESS) {
			g_state = BRCM_WIFIAPI_STATE_SUPPLICANT_RUNNING;
			goto errout;
		}
		if (g_network_id) {
			free(g_network_id);
			g_network_id = NULL;
		}
		g_network_id = strdup(network_id);
	}

errout:
	if (network_id) {
		free(network_id);
		network_id = NULL;
	}

	return result;
}

static void brcm_set_bss_expiration(void)
{
	brcm_send_command_str_digit(WPA_COMMAND_BSS_EXPIRE_AGE, BRCM_BSS_EXPIRE_AGE);
}

static void brcm_set_scan_interval(uint8_t interval)
{
	if (interval < BRCM_SCAN_INTERVAL_MIN) {
		interval = BRCM_SCAN_INTERVAL_MIN;
	} else if (interval > BRCM_SCAN_INTERVAL_MAX) {
		interval = BRCM_SCAN_INTERVAL_MAX;
	}
	brcm_send_command_str_digit(WPA_COMMAND_SCAN_INTERVAL, interval);
}

static void brcm_set_autoconnect(uint8_t onoff)
{
	brcm_send_command_str_digit(WPA_COMMAND_AUCOTONNECT, onoff);
}

static void brcm_set_updateconfig(void)
{
	brcm_send_command_str_digit(WPA_COMMAND_UPDATE_CONFIG, BRCM_SAVE_CONFIG);
}

static int8_t brcm_disable_all_networks(void)
{
	int8_t result = BRCM_STATUS_SUCCESS;
	brcm_send_command_str_upto_4(WPA_COMMAND_DISABLE_NETWORK, NULL, NULL, NULL, NULL);
	if (BRCM_WIFI_API_DEBUG_SLEEP) {
		brcm_demo_app_sleep(1, "DISABLE ALL NETWORKS");
	}
	return result;

}

static int8_t brcm_stop_ap(void)
{
	int8_t result = BRCM_STATUS_ERROR;
	VPRINT("requesting to stop ap mode\n");
	if (BRCM_WIFI_API_DEBUG_SLEEP) {
		brcm_demo_app_sleep(1, NULL);
	}
	brcm_send_command_str_upto_4(WPA_COMMAND_STOP_AP, NULL, NULL, NULL, &result);
	if (result == BRCM_STATUS_SUCCESS) {
		VPRINT("stop ap mode awaiting AP_DISABLED event\n");
		int res = sem_wait(&g_sem_ap_mode);
		if (res) {
			EPRINT("BRCM_API sem_wait error %d\n", errno);
		}
		if (g_state != BRCM_WIFIAPI_STATE_SUPPLICANT_RUNNING) {
			EPRINT("stop ap mode - awaiting AP_DISABLED event - failed\n");
			result = BRCM_STATUS_ERROR;
		}
	}
	if (BRCM_WIFI_API_DEBUG_SLEEP) {
		brcm_demo_app_sleep(1, "STOP AP");
	}
	return result;
}

static void brcm_remove_network(char *network_id)
{
	// remove network
	brcm_send_command_str_upto_4(WPA_COMMAND_REMOVE_NETWORK, network_id, NULL, NULL, NULL);
}

// create a new network block for AP mode (mode=2)
static int8_t brcm_set_ap_network(brcm_ap_config_t *ap_config)
{
	int8_t result = BRCM_STATUS_ERROR;
	char *pbuf = NULL;
	char command[WPA_COMMAND_MAX_SIZE] = { 0 };
	char *network_id = NULL;
	char ssid_formated[WPA_MAX_SSID_LEN];
	printf_encode(ssid_formated, WPA_MAX_SSID_LEN - 1, ap_config->ssid, ap_config->ssid_len);
	g_state = BRCM_WIFIAPI_STATE_AP_ENABLING;

	VPRINT("BRCM_API setup network with ssid %s\n", ssid_formated);

	if ((brcm_get_network(ap_config->ssid, ap_config->ssid_len, &network_id) != BRCM_STATUS_SUCCESS)
		&& network_id == NULL) {
		//no network block exists with that SSID - create one!
		memset(command, 0, WPA_COMMAND_MAX_SIZE);
		snprintf(command, WPA_COMMAND_MAX_SIZE, "%s", WPA_COMMAND_ADD_NETWORK);
		pbuf = brcm_send_request(command, NULL);
		if (pbuf) {
			pbuf[strcspn(pbuf, "\r\n")] = '\0';
			network_id = strdup(pbuf);
			if (pbuf) {
				free(pbuf);
				//removed to prevent lint warning pbuf = NULL;
			}
		}
		if (BRCM_WIFI_API_DEBUG_SLEEP) {
			brcm_demo_app_sleep(1, WPA_COMMAND_ADD_NETWORK);
		}
	}

	if (network_id) {
		// Set mode (needs to be mode=2 for AP mode)
		brcm_send_command_str_upto_4(WPA_COMMAND_SET_NETWORK, network_id, WPA_PARAM_MODE_AP, NULL, &result);
		if (result != BRCM_STATUS_SUCCESS) {
			goto errout;
		}

		if (BRCM_WIFI_API_DEBUG_SLEEP) {
			brcm_demo_app_sleep(1, WPA_PARAM_MODE_AP);
		}
		// Set network ssid
		if (ap_config->ssid_len != 0) {
			memset(command, 0, WPA_COMMAND_MAX_SIZE);
			snprintf(command, WPA_COMMAND_MAX_SIZE, "%s%s %sP\"%s\"", WPA_COMMAND_SET_NETWORK, network_id, WPA_PARAM_SSID, ssid_formated);
			pbuf = brcm_send_request(command, &result);
			if (pbuf) {
				free(pbuf);
				//removed to prevent lint warning pbuf = NULL;
				if (result != BRCM_STATUS_SUCCESS) {
					goto errout;
				}
			}
		}
		// Set network disabled=0
		if (BRCM_WIFI_API_DEBUG_SLEEP) {
			brcm_demo_app_sleep(2, "After set mode=2 - before disable = 0 ");
		}
		brcm_send_command_str_upto_4(WPA_COMMAND_SET_NETWORK, network_id, WPA_PARAM_NETWORK_DISABLED, NULL, &result);
		if (result != BRCM_STATUS_SUCCESS) {
			goto errout;
		}
		if (BRCM_WIFI_API_DEBUG_SLEEP) {
			brcm_demo_app_sleep(2, "After  disable = 0 - before set security");
		}

		// Set security
		result = brcm_set_security(ap_config->security, network_id);
		if (BRCM_WIFI_API_DEBUG_SLEEP) {
			brcm_demo_app_sleep(1, "security set");
		}

		if (result == BRCM_STATUS_SUCCESS) {
			// Set freqency/channel
			if (ap_config->channel != 0) {
				int localFreq = 0;
				char ccode[3];
				memset(ccode, 0, 3);
				if (brcm_get_country_code(ccode) == BRCM_STATUS_SUCCESS) {
					int endchannel = 13;
					if (strncmp(ccode, "US", 2) == 0 || strncmp(ccode, "CA", 2) == 0) {
						endchannel = 11;
					}
					if (ap_config->channel >= 1 || ap_config->channel <= endchannel) {
						localFreq = 2407 + 5 * ap_config->channel;
					}
					if (strncmp(ccode, "JP", 2) == 0 || ap_config->channel == 14) {
						localFreq = 2484;
					}
				}

				memset(command, 0, WPA_COMMAND_MAX_SIZE);
				if (localFreq) {
					// Frequency found the specify otherwise failed
					snprintf(command, WPA_COMMAND_MAX_SIZE, "%s%s %s%d", WPA_COMMAND_SET_NETWORK, network_id, WPA_PARAM_FREQUENCY, localFreq);
					pbuf = brcm_send_request(command, &result);
					if (pbuf) {
						free(pbuf);
						// removed to prevent lint error pbuf = NULL;
						if (result != BRCM_STATUS_SUCCESS) {
							goto errout;
						}
					}
				} else {
					DPRINT("BRCM_API ERROR: Failed to set channel - wrong channel \n");
					result = BRCM_STATUS_PARAM_FAILED;
					goto errout;
				}
			}
			if (BRCM_WIFI_API_DEBUG_SLEEP) {
				brcm_demo_app_sleep(2, "After set freq - before select ");
			}

			// Set ap config beacon interval different from default 100
			if (ap_config->beacon_period != 100) {
				memset(command, 0, WPA_COMMAND_MAX_SIZE);
				snprintf(command, WPA_COMMAND_MAX_SIZE, "%s%s %s%d", WPA_COMMAND_SET_NETWORK, network_id, WPA_PARAM_BEACON_INT, ap_config->beacon_period);
				pbuf = brcm_send_request(command, &result);
				if (pbuf) {
					free(pbuf);
					// removed to prevent lint warning pbuf = NULL;
					if (result != BRCM_STATUS_SUCCESS) {
						goto errout;
					}
				}
			}
			// Set ap config DTIM interval different from default 2
			if (ap_config->DTIM != 2) {
				memset(command, 0, WPA_COMMAND_MAX_SIZE);
				snprintf(command, WPA_COMMAND_MAX_SIZE, "%s%s %s%d", WPA_COMMAND_SET_NETWORK, network_id, WPA_PARAM_DTIM_PERIOD, ap_config->DTIM);
				pbuf = brcm_send_request(command, &result);
				if (pbuf) {
					free(pbuf);
					// removed to prevent lint warning pbuf = NULL;
					if (result != BRCM_STATUS_SUCCESS) {
						goto errout;
					}
				}
			}
			// Set ap config Vendor specific IE
			if (ap_config->vsie != NULL) {
				DPRINT("OUI: %02X%02X%02X\n", ap_config->vsie->oui[0], ap_config->vsie->oui[1], ap_config->vsie->oui[2]);
				uint32_t cmdsize = ap_config->vsie->content_length * 3 + 30;
				char *tcommand = zalloc(cmdsize);
				size_t slen = 0;

				if (tcommand == NULL) {
					goto errout;
				}
				if (ap_config->vsie->content_length > 253) {
					free(tcommand);
					result = BRCM_STATUS_PARAM_FAILED;
					goto errout;
				}
				uint8_t *iehex = brcm_bytearray_2_hexstr(ap_config->vsie->content,
								 ap_config->vsie->content_length,
								 &slen);
				VPRINT("Content length: %d\nContent: %s\n", ap_config->vsie->content_length, iehex);
				snprintf(tcommand, cmdsize, "%s%s%02X%02X%02X %s", WPA_COMMAND_SET, WPA_PARAM_VSIE, ap_config->vsie->oui[0], ap_config->vsie->oui[1], ap_config->vsie->oui[2], iehex);
				pbuf = brcm_send_request(tcommand, &result);
				if (iehex != NULL) {
					free(iehex);
				}
				if (tcommand) {
					free(tcommand);
				}
				if (pbuf) {
					free(pbuf);
					// removed to prevent lint warning pbuf = NULL;
					if (result != BRCM_STATUS_SUCCESS) {
						goto errout;
					}
				}
			}
			// Set phy_mode if disabled, default is enabled
			if (ap_config->phy_mode == 0) {
#ifdef CONFIG_HT_OVERRIDES
				brcm_send_command_str_upto_4(WPA_COMMAND_SET_NETWORK, network_id, WPA_PARAM_DISABLE_HT, NULL, &result);
				if (result != BRCM_STATUS_SUCCESS) {
					goto errout;
				}
#else
				result = BRCM_STATUS_COMMAND_UNKNOWN;
				goto errout;
#endif
			} else {
				if ((ap_config->ht_mode.ht_capab_info & HT_CAP_INFO_GREEN_FIELD)
					&& (ap_config->ht_mode.ht_capab_info & HT_CAP_INFO_SHORT_GI20MHZ)) {
					// Cannot both be set
					DPRINT("Cannot set both Green field and Short Guard Interval \n");
					result = BRCM_STATUS_PARAM_FAILED;
					goto errout;
				}
				// Set green field
				// green_field and guard_interval are mutually exclusive
				if (ap_config->ht_mode.ht_capab_info & HT_CAP_INFO_GREEN_FIELD) {

#ifdef CONFIG_HT_OVERRIDES
					brcm_send_command_str_upto_4(WPA_COMMAND_SET_NETWORK, network_id, WPA_PARAM_HT_CAPAB, HT_CAP_INFO_GREEN_FIELD, &result);
					if (result != BRCM_STATUS_SUCCESS) {
						goto errout;
					}
#else
					result = BRCM_STATUS_COMMAND_UNKNOWN;
					goto errout;
#endif
				}
				// Set guard interval off
				// green_field and guard_interval are mutually exclusive
#ifdef CONFIG_HT_OVERRIDES
				if ((ap_config->ht_mode.ht_capab_info & HT_CAP_INFO_SHORT_GI20MHZ) == 0) {
					brcm_send_command_str_upto_4(WPA_COMMAND_SET_NETWORK, network_id, WPA_PARAM_DISABLE_SHORT_GI, NULL, &result);
					if (result != BRCM_STATUS_SUCCESS) {
						goto errout;
					}
				}
#endif
				// Set MCS index
				if (ap_config->ht_mode.mcs_index[0] != 0 || ap_config->ht_mode.mcs_index[1] != 0) {
#ifdef CONFIG_HT_OVERRIDES
					memset(command, 0, WPA_COMMAND_MAX_SIZE);
					snprintf(command, WPA_COMMAND_MAX_SIZE, "%s%s %s%x", WPA_COMMAND_SET_NETWORK, network_id, WPA_PARAM_HT_MCS, ap_config->ht_mode.mcs_index);
					pbuf = brcm_send_request(command, &result);
					if (pbuf) {
						free(pbuf);
						pbuf = NULL;
						if (result != BRCM_STATUS_SUCCESS) {
							goto errout;
						}
					}
#else
					result = BRCM_STATUS_COMMAND_UNKNOWN;
					goto errout;
#endif
				}

			}

			// Select network
			brcm_send_command_str_upto_4(WPA_COMMAND_SELECT_NETWORK, network_id, NULL, NULL, &result);
			if (result != BRCM_STATUS_SUCCESS) {
				goto errout;
			} else {
				VPRINT("setup ap mode awaiting AP_ENABLED event\n");
				int res = sem_wait(&g_sem_ap_mode);
				if (res) {
					EPRINT("BRCM_API sem_wait error %d\n", errno);
				}
				if (g_state != BRCM_WIFIAPI_STATE_AP_ENABLED) {
					EPRINT("stop ap mode - awaiting AP_DISABLED event - failed\n");
					result = BRCM_STATUS_ERROR;
					goto errout;
				}
				if (g_network_id) {
					free(g_network_id);
					g_network_id = NULL;
				}
				g_network_id = strdup(network_id);
			}
			if (BRCM_WIFI_API_DEBUG_SLEEP) {
				brcm_demo_app_sleep(2, "After select");
			}
		}

errout:
		if (result != BRCM_STATUS_SUCCESS) {
			// remove network
			brcm_remove_network(network_id);
			g_state = BRCM_WIFIAPI_STATE_SUPPLICANT_RUNNING;
		}
		if (BRCM_WIFI_API_DEBUG_SLEEP) {
			brcm_demo_app_sleep(1, "AP setup");
		}
		if (network_id) {
			free(network_id);
			network_id = NULL;
		}
	}

	return result;
}

static int8_t brcm_start_scan(void)
{
	int8_t result = BRCM_STATUS_ERROR;
	brcm_send_command_str_upto_4(WPA_COMMAND_SCAN, NULL, NULL, NULL, &result);
	if (result == BRCM_STATUS_SUCCESS) {
		g_scanning = 1;
	}
	return result;
}

brcm_scan_info_t *brcm_parse_scan_results(char *sr)
{
	brcm_scan_info_t *results = NULL;
	char *pos, *end;
	pos = strchr(sr, '\n');

	if (pos == NULL) {
		return results;
	}

	brcm_scan_info_t *local_scan_result = (brcm_scan_info_t *)zalloc((size_t)sizeof(brcm_scan_info_t));
	if (local_scan_result == NULL) {
		return results;
	}

	results = local_scan_result;
	/*
	 * Example of result to parse
	 *  bssid
	 *  c4:04:15:0d:5d:04
	 *  20:4e:7f:5a:d4:a0
	 *  00:11:21:76:d4:00
	 */
	*pos = '\0';
	pos = pos + 1;
	bool discarted = FALSE;
	brcm_scan_info_t *local_prev = NULL;

	while (*pos) {
		if (local_scan_result == NULL) {
			break;
		}
		memset(local_scan_result, 0, sizeof(brcm_scan_info_t));
		end = strchr(pos, '\n');
		if (end == NULL) {
			break;
		}

		*end = '\0';			// Replace '\n' with '\0' to make it a valid string
		memcpy(&(local_scan_result->bssid), pos, 18);
		/* The BSSID have been extracted, fill in the remaining information */

		if (brcm_get_bss_info(local_scan_result->bssid, local_scan_result)) {
			discarted = FALSE;
			pos = end + 1;
			if (*pos != '\0') {
				local_prev = local_scan_result;
				local_scan_result->next = zalloc(sizeof(brcm_scan_info_t));
				local_scan_result = local_scan_result->next;
			} else {
				local_scan_result->next = NULL;
			}
		} else {
			pos = end + 1;
			discarted = TRUE;
		}
	}

	if (discarted) {
		if (local_scan_result) {
			VPRINT("Allocated one to many - set previous next to NULL\n");
			free(local_scan_result);
			local_scan_result = NULL;
			if (local_prev) {
				local_prev->next = NULL;
			}
		}
	}

	return results;
}

static int8_t brcm_get_api_scan_results(brcm_scan_info_t **result_handler)
{
	int8_t result = BRCM_STATUS_ERROR;
	char *pbuf = NULL;
	char command[WPA_COMMAND_MAX_SIZE] = { 0 };
	snprintf(command, WPA_COMMAND_MAX_SIZE, "%s", WPA_COMMAND_SCAN_RESULTS);
	pbuf = brcm_send_request(command, NULL);
	if (pbuf) {
		*result_handler = brcm_parse_scan_results(pbuf);
		free(pbuf);
		pbuf = NULL;
		result = BRCM_STATUS_SUCCESS;
	}
	return result;
}

static int8_t brcm_leave_network(void)
{
	int8_t result = BRCM_STATUS_ERROR;
	brcm_send_command_str_upto_4(WPA_COMMAND_DISCONNECT, NULL, NULL, NULL, &result);
	return result;
}

static int8_t brcm_get_bssid(char **bssid)
{
	int8_t result = BRCM_STATUS_ERROR;
	char *pbuf = NULL;
	char command[WPA_COMMAND_MAX_SIZE] = { 0 };
	char *pos, *end;

	snprintf(command, WPA_COMMAND_MAX_SIZE, "%s", WPA_COMMAND_STATUS);
	pbuf = brcm_send_request(command, NULL);
	if (pbuf) {
		pos = strstr(pbuf, WPA_VALUE_BSSID);
		if (pos == NULL) {
			VPRINT("BRCM_API get bssid FAILED");
			free(pbuf);
			pbuf = NULL;
			goto errout;
		}
		pos += 6;
		end = strchr(pos, '\n');
		if (end == NULL) {
			VPRINT("BRCM_API get bssid FAILED");
			free(pbuf);
			pbuf = NULL;
			goto errout;
		}
		*end = '\0';
		VPRINT("BRCM_API get_bssid bssid %s\n", pos);
		memcpy(*bssid, pos, 18);
		free(pbuf);
		pbuf = NULL;
		result = BRCM_STATUS_SUCCESS;
	}
errout:
	return result;
}

static int8_t brcm_get_country_code(char *country_code)
{
	int8_t result = BRCM_STATUS_ERROR;
#ifndef REMOVE_NVRAM
	brcm_init_nvram();
	if (up_wlan_read_config((void *)nvram, BRCM_WIFI_NV_DATA_SIZE)) {
		g_brcm_wifi_nv_data = (brcm_wifi_nv_data_t *)nvram;
		memcpy(country_code, g_brcm_wifi_nv_data->country_code, 3);
		country_code[2] = '\0';
		DPRINT("Country code found: %s\n", country_code);
		result = BRCM_STATUS_SUCCESS;
	}
	if (nvram) {
		free(nvram);
		nvram = NULL;
	}
#else
	up_wlan_get_country(country_code);
	result = BRCM_STATUS_SUCCESS;
#endif
	return result;
}

int8_t brcm_sent_force_panic(void)
{
	int8_t result = BRCM_STATUS_ERROR;
	brcm_send_command_str_upto_4(WPA_COMMAND_DRIVER, WPA_PARAM_FORCE_PANIC, NULL, NULL, &result);
	return result;
}

static int8_t brcm_set_country_code(const char *country_code, bool write_to_nvram, bool write_to_driver)
{
	int8_t result = BRCM_STATUS_ERROR;
#ifndef REMOVE_NVRAM
//    char *pbuf = NULL;
	brcm_init_nvram();

	if (write_to_driver) {
		brcm_send_command_str_upto_4(WPA_COMMAND_DRIVER, WPA_PARAM_COUNTRY, (char *)country_code, NULL, &result);
		VPRINT("BRCM_API country code: %s\n", country_code);
	}
	if (write_to_nvram) {
		if (nvram == NULL) {
			nvram = malloc(4 * 1024);	// flash block size of 4k
		}
		// set the country code in NV ram for next time
		//start by reading the existing values form NVRAM and Erase to be ready for writing
		if (up_wlan_read_config((void *)nvram, BRCM_WIFI_NV_DATA_SIZE)) {
			g_brcm_wifi_nv_data = (brcm_wifi_nv_data_t *)nvram;
			VPRINT("Read from NVRAM: %s\n", g_brcm_wifi_nv_data->country_code);
			up_wlan_erase_config();
			memcpy(g_brcm_wifi_nv_data->country_code, country_code, 3);
			g_brcm_wifi_nv_data->country_code[2] = '\0';
			//store the country code for later
			g_country_code[0] = country_code[0];
			g_country_code[1] = country_code[1];
			// write it back to nvram
			DPRINT("Writing countrycode %s to NVRAM\n", g_brcm_wifi_nv_data->country_code);
			if (!up_wlan_write_config((void *)nvram, BRCM_WIFI_NV_DATA_SIZE)) {
				EPRINT("NVRAM write failed\n");
			} else {
				result = BRCM_STATUS_SUCCESS;
			}
		} else {
			EPRINT("NVRAM read failed\n");
		}
	}
	if (g_state == BRCM_WIFIAPI_STATE_NOT_STARTED) {
		// we need to cleanup nvram as it will only get cleaned on wifistop
		if (nvram != NULL) {
			free(nvram);
			nvram = NULL;
		}
	}
#else
	up_wlan_set_country(country_code);
#endif
	return result;
}

static int8_t brcm_get_tx_power(uint8_t *dbm)
{
#ifndef REMOVE_NVRAM
	int8_t result = BRCM_STATUS_ERROR;
	char *pbuf = NULL;
	char *pos, *end;
	char command[WPA_COMMAND_MAX_SIZE] = { 0 };
	brcm_init_nvram();

	snprintf(command, WPA_COMMAND_MAX_SIZE, "%s", WPA_COMMAND_GET_TX_POWER);
	pbuf = brcm_send_request(command, NULL);

	if (pbuf) {
		*dbm = strtol(pbuf, &end, 10);
		VPRINT("dbm = %d\n", *dbm);
		pos = end + 1;
		free(pbuf);
		pbuf = NULL;
		result = BRCM_STATUS_SUCCESS;
	} else {
		DPRINT("Could not get tx_power from supplicant, trying from NVRAM\n");
		if (nvram == NULL) {
			nvram = malloc(4 * 1024);	// flash block size of 4k
		}
		if (up_wlan_read_config((void *)nvram, BRCM_WIFI_NV_DATA_SIZE)) {
			g_brcm_wifi_nv_data = (brcm_wifi_nv_data_t *)nvram;
			*dbm = g_brcm_wifi_nv_data->tx_power;
			DPRINT("Tx Power found in NVRAM: %d dbm\n", dbm);
			result = BRCM_STATUS_SUCCESS;
		}
	}
	if (nvram) {
		free(nvram);
		nvram = NULL;
	}
#else
	int result = up_wlan_get_txpower();
	if(result <= 0)
		return BRCM_STATUS_ERROR;
	*dbm = result;
#endif
	return BRCM_STATUS_SUCCESS;
}

static int8_t brcm_set_tx_power(uint8_t *dbm, bool write_to_nvram, bool write_to_sup)
{
#ifndef REMOVE_NVRAM
	int8_t result = BRCM_STATUS_ERROR;
	if (*dbm > 30 || *dbm < 12) {
		return BRCM_STATUS_PARAM_FAILED;
	}
	if (write_to_sup) {
		brcm_send_command_str_digit(WPA_COMMAND_SET_TX_POWER, *dbm);
	}
	if (write_to_nvram) {
		brcm_init_nvram();
		// set the tx_power in NV ram for next time
		//start by reading the existing values form NVRAM and Erase to be ready for writing
		if (up_wlan_read_config((void *)nvram, BRCM_WIFI_NV_DATA_SIZE)) {
			g_brcm_wifi_nv_data = (brcm_wifi_nv_data_t *)nvram;
			VPRINT("Tx Power read from NVRAM: %d\n", g_brcm_wifi_nv_data->tx_power);
			up_wlan_erase_config();
			g_brcm_wifi_nv_data->tx_power = *dbm;
			// write it back to nvram
			DPRINT("Writing tx_power (dbm) %d to NVRAM\n", g_brcm_wifi_nv_data->tx_power);
			if (!up_wlan_write_config((void *)nvram, BRCM_WIFI_NV_DATA_SIZE)) {
				EPRINT("NVRAM write failed\n");
			} else {
				result = BRCM_STATUS_SUCCESS;
			}
		} else {
			EPRINT("NVRAM read failed\n");
		}
	}
	if (g_state == BRCM_WIFIAPI_STATE_NOT_STARTED) {
		// we need to cleanup nvram as it will only get cleaned on wifistop
		if (nvram != NULL) {
			free(nvram);
			nvram = NULL;
		}
	}
	return result;
#else
	if(up_wlan_set_txpower(*dbm) != 0)
		return BRCM_STATUS_ERROR;
	return BRCM_STATUS_SUCCESS;
#endif
}

static int8_t brcm_get_rssi(int8_t *rssi_value)
{
	int8_t result = BRCM_STATUS_ERROR;

	char *pbuf = NULL;
	char *pos, *end;

	char command[WPA_COMMAND_MAX_SIZE] = { 0 };

	snprintf(command, WPA_COMMAND_MAX_SIZE, "%s", WPA_COMMAND_SIGNAL_POLL);

	pbuf = brcm_send_request(command, NULL);
	if (pbuf) {
		VPRINT("BRCM_API brcm_get_rssi buffer: %s\n", pbuf);
		// Extract rssi
		pos = strstr(pbuf, WPA_VALUE_RSSI);
		if (pos == NULL) {
			VPRINT("BRCM_API get_rssi cannot find: %s\n", WPA_VALUE_RSSI);
			free(pbuf);
			pbuf = NULL;
			goto errout;
		}
		pos += 5;
		end = strchr(pos, '\n');
		if (end == NULL) {
			VPRINT("BRCM_API get_rssi failed \n");
			free(pbuf);
			pbuf = NULL;
			goto errout;
		}
		*end = '\0';
		*rssi_value = strtol(pos, &end, 10);
		VPRINT("BRCM_API rssi: %d\n", *rssi_value);
		result = BRCM_STATUS_SUCCESS;
	}
errout:
	return result;
}

static int8_t brcm_get_mac(uint8_t *mac)
{
	int8_t result = BRCM_STATUS_ERROR;
	char *pbuf = NULL;
	char command[WPA_COMMAND_MAX_SIZE] = { 0 };
	char *pos, *end;

	// Check status to get own address
	snprintf(command, WPA_COMMAND_MAX_SIZE, "%s", WPA_COMMAND_STATUS);
	pbuf = brcm_send_request(command, NULL);
	if (pbuf) {
		uint8_t i;
		pos = strstr(pbuf, WPA_VALUE_IP_ADDRESS);
		if (pos != NULL) {
			pos += 11;
			pos = strstr(pos, WPA_VALUE_ADDRESS);
		} else {
			pos = strstr(pbuf, WPA_VALUE_ADDRESS);
		}
		if (pos == NULL) {
			VPRINT("BRCM_API get_mac parameter missing WPA_VALUE_ADDRESS\n");
			free(pbuf);
			pbuf = NULL;
			goto errout;
		}
		pos += 8;
		end = strchr(pos, '\n');
		if (end == NULL) {
			VPRINT("BRCM_API get_mac command 2 FAILED\n");
			free(pbuf);
			pbuf = NULL;
			goto errout;
		}
		*end = '\0';
		for (i = 0; i < 6; i++) {
			mac[i] = strtol(pos, &end, 16);
			VPRINT("BRCM_API get_mac select: %02x\n", mac[i]);
			pos += 3;
		}
		free(pbuf);
		pbuf = NULL;
		result = BRCM_STATUS_SUCCESS;
	}
errout:
	return result;
}

static int8_t brcm_get_channel(int8_t *channel)
{
	int8_t result = BRCM_STATUS_ERROR;
	char *pbuf = NULL;
	char command[WPA_COMMAND_MAX_SIZE] = { 0 };
	char *pos, *end;

	snprintf(command, WPA_COMMAND_MAX_SIZE, "%s", WPA_COMMAND_STATUS);
	pbuf = brcm_send_request(command, NULL);
	if (pbuf) {
		// Extract frequency
		pos = strstr(pbuf, WPA_VALUE_FREQ);
		if (pos == NULL) {
			VPRINT("BRCM_API get: %s FAILED \n", WPA_VALUE_FREQ);
			free(pbuf);
			pbuf = NULL;
			goto errout;
		}
		pos += 5;
		end = strchr(pos, '\n');
		if (end == NULL) {
			free(pbuf);
			pbuf = NULL;
			goto errout;
		}
		*end = '\0';
		ieee80211_freq_to_chan(strtol(pos, &end, 10), (uint8_t *)channel);
		VPRINT("BRCM_API channel: %d\n", *channel);
		result = BRCM_STATUS_SUCCESS;
		pos = end + 1;
		free(pbuf);
		pbuf = NULL;
	}
errout:
	return result;
}

static int8_t brcm_terminate_supplicant(void)
{
	int8_t result = BRCM_STATUS_ERROR;
	VPRINT("brcm_terminate_supplicant \n");
	brcm_send_command_str_upto_4(WPA_COMMAND_TERMINATE, NULL, NULL, NULL, &result);
	return result;
}

static uint8_t brcm_stop_supplicant(void)
{
	uint8_t result;
	int status = -1;
	UNUSED(status);
	pid_t r_task_id = -1;
	g_state = BRCM_WIFIAPI_STATE_TERMINATING;
	VPRINT("BRCM_API terminate requested\n");
	if (BRCM_WIFI_API_DEBUG_SLEEP) {
		brcm_demo_app_sleep(1, "terminate requested - stop app and terminate");
	}
	result = brcm_terminate_supplicant();
	if (result == BRCM_STATUS_SUCCESS) {
		int res = sem_wait(&g_sem_terminate);
		if (res) {
			EPRINT("BRCM_API sem_wait error %d\n", errno);
		}
	} else {
		EPRINT("Termination of supplicant failed - but will continue disconnecting wpa_ctrl \n");
	}
	if (BRCM_WIFI_API_DEBUG_SLEEP) {
		brcm_demo_app_sleep(1, "before waitpid");
	}
	r_task_id = waitpid(g_task, &status, NULL);
	DPRINT("Wait for supplicant task to terminate\n");
	if (r_task_id != g_task) {
		EPRINT("Error waiting for task termination but nothing to do about it\n");
	}
	DPRINT("    result: %d, status: %d\n", r_task_id, status);
	g_task = 0;
	// if monitor still running at least running state to not running
	if (g_running) {
		VPRINT("BRCM_API stop running\n");
		// Will end while loop in monitoring thread and exit thread
		g_running = 0;
	}
	// Disconnect wpa_ctrl connection
	result = brcm_wpa_close(TRUE);
	if (result != BRCM_STATUS_SUCCESS) {
		EPRINT("Error: still running after disconnect. \n");
	}
	return result;
}

static uint8_t brcm_start_supplicant(void)
{
	uint8_t result;
#ifndef CONFIG_DEBUG
	char *logarg = NULL;
#endif
	char *confarg = NULL;
#ifdef CONFIG_BRCM_WIFI_FILESYSTEM_SUPPORT
	if (brcm_init_filesystem() == BRCM_STATUS_SUCCESS) {
		confarg = "-c" CONFIG_BRCM_WIFI_DIR "/" CONFIG_BRCM_WIFI_CONFIG_FILE_NAME;
#ifndef CONFIG_DEBUG
		logarg = "-f" CONFIG_BRCM_WIFI_DIR "/" CONFIG_BRCM_WIFI_LOG_FILE_NAME;
#endif							// CONFIG_DEBUG
	}
#endif							// CONFIG_BRCM_WIFI_FILESYSTEM_SUPPORT
	VPRINT("BRCM_API start up supplicant ifname %s\n", CTRL_IFNAME);
	// Start by creating task for wpa_supplicant
#ifdef CONFIG_DEBUG_WLAN_SUPPLICANT_ERROR
	sup_argv[0] = "-dd";		// ERROR + WARN + INFO
#ifdef CONFIG_DEBUG_WLAN_SUPPLICANT_DEBUG
	sup_argv[0] = "-ddd";		// + DEBUG
#endif
#ifdef CONFIG_DEBUG_WLAN_SUPPLICANT_MORE
	sup_argv[0] = "-dddd";		// + DUMP
#endif
#ifdef CONFIG_DEBUG_WLAN_SUPPLICANT_VERBOSE
	sup_argv[0] = "-ddddd";		// + EXCESSIVE
#endif
	sup_argv[1] = "-t";
	sup_argv[2] = "-i" CTRL_IFNAME;
	if (confarg != NULL) {
		sup_argv[3] = confarg;
	} else {
		sup_argv[3] = "-Cudp";
	}
	sup_argv[4] = NULL;
#elif defined(CONFIG_DEBUG)		// user manually disabled logging from supplicant
	sup_argv[0] = "-qqq";
	sup_argv[1] = "-t";
	sup_argv[2] = "-i" CTRL_IFNAME;
	if (confarg != NULL) {
		sup_argv[3] = confarg;
	} else {
		sup_argv[3] = "-Cudp";
	}
	sup_argv[4] = NULL;
#else
	sup_argv[0] = "-B";
	sup_argv[1] = "-t";
	sup_argv[2] = "-i" CTRL_IFNAME;
	if (confarg != NULL) {
		sup_argv[3] = confarg;
	} else {
		sup_argv[3] = "-Cudp";
	}

	if (logarg != NULL) {
		sup_argv[4] = logarg;
		sup_argv[5] = NULL;
	} else {
		sup_argv[4] = NULL;
	}
#endif
	VPRINT("BRCM_API call task_create\n");
	if (BRCM_WIFI_API_DEBUG_SLEEP) {
		brcm_demo_app_sleep(1, NULL);
	}
	g_task = task_create("WPA Supplicant", 100, 4096, wpa_supplicant_main, sup_argv);
	sleep(1);

	VPRINT("BRCM_API task_create called \n");
	if (g_task < 0) {
		EPRINT("BRCM_API ERROR: Failed to start supplicant \n");
		result = BRCM_STATUS_SUPPLICANT_START_FAILED;
	} else {
		DPRINT("BRCM_API task_create supplicant task pid: %d\n", g_task);
		/* initialize globals */
		g_state = BRCM_WIFIAPI_STATE_SUPPLICANT_RUNNING;
		g_running = 0;
		g_wpa_attached = 0;
		if (BRCM_WIFI_API_DEBUG_SLEEP) {
			brcm_demo_app_sleep(1, NULL);
		}
		result = brcm_wpa_open(CTRL_IFNAME);
		if (BRCM_WIFI_API_DEBUG_SLEEP) {
			brcm_demo_app_sleep(2, NULL);
		}
	}
	return result;
}

static bool brcm_is_connected(void)
{
	bool ret = FALSE;
	if ((g_state == BRCM_WIFIAPI_STATE_STA_CONNECTED) || (g_state == BRCM_WIFIAPI_STATE_AP_CONNECTED)) {
		ret = TRUE;
	}
	return ret;
}

#ifndef REMOVE_NVRAM
static void brcm_init_nvram(void)
{

	if (nvram == NULL) {
		nvram = zalloc(4 * 1024);	// flash block size of 4k
		if (!nvram) {
			return;
		}
	}
	if (up_wlan_read_config((void *)nvram, BRCM_WIFI_NV_DATA_SIZE)) {
		g_brcm_wifi_nv_data = (brcm_wifi_nv_data_t *)nvram;
		//check if it this is not the first boot - it is already initialized
		if (memcmp("BRCM", g_brcm_wifi_nv_data->initialized, 4) != 0) {
			char *default_cc = BRCM_WIFI_NV_DEFAULT_COUNTRY_CODE;
			uint8_t tx_power = BRCM_WIFI_NV_DEFAULT_TX_POWER;

			//erase the NVRAM area so we are ready to write to it
			up_wlan_erase_config();
			// write it back to nvram
			strncpy(&g_brcm_wifi_nv_data->initialized[0], "BRCM", 4);
			g_brcm_wifi_nv_data->initialized[4] = '\0';	//needed for svace
			/* the read value is not one we can understand - lets store a sane one */
			g_brcm_wifi_nv_data->country_code[0] = default_cc[0];
			g_brcm_wifi_nv_data->country_code[1] = default_cc[1];
			g_brcm_wifi_nv_data->country_code[2] = '\0';
			if (tx_power < 12 || tx_power > 30) {
				EPRINT("Wrong value (%d) for TX power found in config. Correcting it to 30dbm\n", tx_power);
				tx_power = 30;
			}
			g_brcm_wifi_nv_data->tx_power = tx_power;
			if (!up_wlan_write_config((void *)nvram, BRCM_WIFI_NV_DATA_SIZE)) {
				EPRINT("NVRAM write failed\n");
			} else {
				VPRINT("NVRAM now contains:\n\tcountry-code: %s\n\ttx_power: %d\n", g_brcm_wifi_nv_data->country_code, g_brcm_wifi_nv_data->tx_power);
			}
		} else {
			VPRINT("NVRAM already initialized\n");
			VPRINT("NVRAM contains:\n\tcountry-code: %s\n\ttx_power: %d\n", g_brcm_wifi_nv_data->country_code, g_brcm_wifi_nv_data->tx_power);
		}

	} else {
		EPRINT("Failed to read NVRAM\n");
	}
}
#endif

static int8_t brcm_init(WiFi_InterFace_ID_t interface_id, const brcm_ap_config_t *ap_config)
{
	int8_t result = BRCM_STATUS_ERROR;
	g_scan_result_handler = NULL;
	g_scanning = 0;
	sem_init(&g_sem_disconnect, 0, 0);
	sem_init(&g_sem_ap_mode, 0, 0);
	sem_init(&g_sem_api_block, 0, 0);
	sem_init(&g_sem_terminate, 0, 0);

	if (ap_config) {
		VPRINT("start ap_config available\n");
		if (g_ap_config != NULL) {
			if (g_ap_config->security) {
				free(g_ap_config->security);	// We might have a sec. config to free
				g_ap_config->security = NULL;
			}
			if (g_ap_config->vsie) {
				if (g_ap_config->vsie->content) {
					free(g_ap_config->vsie->content);
					g_ap_config->vsie->content = NULL;
				}
				free(g_ap_config->vsie);	// We might have a vsie config to free
				g_ap_config->vsie = NULL;
			}
			free(g_ap_config);	// Free app_config of needed
			g_ap_config = NULL;
		}
		g_ap_config = zalloc(sizeof(brcm_ap_config_t));
		if (g_ap_config == NULL) {
			goto errout;
		}

		*g_ap_config = *ap_config;
		if (ap_config->security) {
			g_ap_config->security = zalloc(sizeof(brcm_security_config_t));
			if (g_ap_config->security == NULL) {
				goto errout;
			}
			*(g_ap_config->security) = *(ap_config->security);
		} else {
			g_ap_config->security = NULL;
		}
		if (ap_config->vsie) {
			g_ap_config->vsie = zalloc(sizeof(brcm_vendor_ie_t));
			if (g_ap_config->vsie == NULL) {
				goto errout;
			}
			*(g_ap_config->vsie) = *(ap_config->vsie);
			g_ap_config->vsie->content = zalloc(ap_config->vsie->content_length);
			if (g_ap_config->vsie->content == NULL) {
				goto errout;
			}
			memcpy(g_ap_config->vsie->content, ap_config->vsie->content, ap_config->vsie->content_length);
		} else {
			g_ap_config->vsie = NULL;
		}
	}

	if (g_task == 0) {
		result = brcm_start_supplicant();
		if (result == BRCM_STATUS_SUCCESS) {
#ifdef CONFIG_SCSC_WLAN_AUTO_RECOVERY
			// setup recovery thread
			if (!g_recovery_running) {
				if (pthread_create(&g_recovery_thread, NULL, (void *)brcm_recovery_thread_handler, NULL)) {
					EPRINT("Failed to recovery create thread\n");
				} else {
					result = BRCM_STATUS_SUCCESS;
					pthread_setname_np(g_monitoring_thread, "Wi-Fi API recovery");
					VPRINT("Recovery thread created successfully\n");
				}
			}
#endif
			// configure supplicant
			brcm_set_updateconfig();
			brcm_set_scan_interval(BRCM_SCAN_INTERVAL);
			brcm_set_bss_expiration();
			if (interface_id == BRCM_WIFI_SOFT_AP_IF) {
				brcm_set_autoconnect(0);
			} else {
				brcm_set_autoconnect(1);
			}

		} else {
			goto errout;
		}
	} else {
		if (g_task && g_running && g_wpa_attached) {
			VPRINT("Supplicant already started ifname %s in task %d \n", CTRL_IFNAME, g_task);
			result = BRCM_STATUS_SUCCESS;
		} else {
			EPRINT("Supplicant started but socket connections need restart \n");
		}
	}
	// initialize nvram if first time we run. use default values from menuconfig
#ifndef REMOVE_NVRAM
	brcm_init_nvram();
	brcm_set_country_code(g_brcm_wifi_nv_data->country_code, FALSE, TRUE);
	brcm_set_tx_power(&g_brcm_wifi_nv_data->tx_power, FALSE, TRUE);
#endif

errout:
	return result;
}

static void brcm_deinit(void)
{
	g_scan_result_handler = NULL;
	g_link_up = NULL;
	g_link_down = NULL;
	g_wpa_attached = 0;
	g_running = 0;
	g_scanning = 0;
	g_state = BRCM_WIFIAPI_STATE_NOT_STARTED;
	g_task = 0;

	sem_destroy(&g_sem_terminate);
	sem_destroy(&g_sem_ap_mode);
	sem_destroy(&g_sem_disconnect);
	sem_destroy(&g_sem_api_block);

	if (nvram != NULL) {
		free(nvram);
		nvram = NULL;
	}
	if (g_ap_config != NULL) {
		if (g_ap_config->security) {
			free(g_ap_config->security);	// We might have a sec. config to free
			g_ap_config->security = NULL;
		}
		if (g_ap_config->vsie) {
			if (g_ap_config->vsie->content) {
				free(g_ap_config->vsie->content);	// We might have a vsie config to free
				g_ap_config->vsie->content = NULL;
			}
			free(g_ap_config->vsie);	// We might have a vsie config to free
			g_ap_config->vsie = NULL;
		}
		// Free app_config of needed
		free(g_ap_config);
		g_ap_config = NULL;
	}
	if (g_network_id) {
		free(g_network_id);
		g_network_id = NULL;
	}
}

static int8_t brcm_api_start(WiFi_InterFace_ID_t interface_id, const brcm_ap_config_t *ap_config)
{
	int8_t result = BRCM_STATUS_ERROR;
	DPRINT("Start\n");

	if (BRCM_WIFI_API_DEBUG_SLEEP) {
		brcm_demo_app_sleep(1, NULL);
	}
	result = brcm_init(interface_id, ap_config);

	if (result == BRCM_STATUS_SUCCESS) {
		if (interface_id == BRCM_WIFI_STATION_IF) {
			VPRINT("Start STA\n");
			/* Check old interface type - if AP then stop first */
			if (brcm_get_op_mode() == BRCM_WIFI_SOFT_AP_IF) {
				VPRINT("STA enabled - stop AP mode\n");
				g_state = BRCM_WIFIAPI_STATE_AP_DISABLING;
				result = brcm_stop_ap();
				if (result != BRCM_STATUS_SUCCESS) {
					EPRINT("stop ap mode - failed - still attempt to disable networks\n");
					uint8_t tmp_result = brcm_disable_all_networks();
					if (tmp_result != BRCM_STATUS_SUCCESS) {
						EPRINT("disable networks - failed\n");
					}
				} else {
					VPRINT("supplicant already running so disabling all existing networks\n");
					result = brcm_disable_all_networks();
				}
				// remove network
				if (g_network_id) {
					brcm_remove_network(g_network_id);
					free(g_network_id);
					g_network_id = NULL;
				}
			}
		} else if (interface_id == BRCM_WIFI_SOFT_AP_IF) {
			VPRINT("Start AP\n");
			/* Disconnect if in STA mode and connected */
			if (g_state == BRCM_WIFIAPI_STATE_STA_CONNECTED) {
				g_state = BRCM_WIFIAPI_STATE_STA_DISCONNECTING;
				VPRINT("supplicant already running - disconnect connected STA networks\n");
				result = brcm_leave_network();
				if (result == BRCM_STATUS_SUCCESS) {
					int res = sem_wait(&g_sem_disconnect);
					if (res) {
						EPRINT("g_sem_disconnect sem_wait error %d\n", errno);
					}
				} else {
					EPRINT("%s Error: disconnect. \n", __func__);
				}
			}

			result = brcm_set_ap_network(g_ap_config);
#ifdef CONFIG_SCSC_WLAN_AUTO_RECOVERY
			if (!g_recovering) {
				/* Save start information needed in case of recovery handling */
				if (result == BRCM_STATUS_SUCCESS) {
					// TODO: can rely on g_ap_config as this is now our own???
					brcm_save_ap_config(g_ap_config);
				}
			}
#endif
		}

		if (BRCM_WIFI_API_DEBUG_SLEEP) {
			brcm_demo_app_sleep(1, NULL);
		}
		/* Do not assign global until after used to check for running interfaces
		 * If i.e. start of AP mode fails then supplicant is still running and we
		 * need to make sure that the interface type is set in that situation */
#ifdef CONFIG_SCSC_WLAN_AUTO_RECOVERY
		if ((result == BRCM_STATUS_SUCCESS || (g_task != 0 && g_state == BRCM_WIFIAPI_STATE_SUPPLICANT_RUNNING)) && !g_recovering) {
			g_recovery_data.old_interface_type = interface_id;
		}
#endif
		if (result != BRCM_STATUS_SUCCESS && (brcm_get_op_mode() == BRCM_WIFI_SOFT_AP_IF)) {
			if (g_network_id) {
				free(g_network_id);
				g_network_id = NULL;
			}
		}
	} else {
		brcm_deinit();
	}
	return result;
}

static WiFi_InterFace_ID_t brcm_get_op_mode(void)
{
	if (g_state >= BRCM_WIFIAPI_STATE_SUPPLICANT_RUNNING && g_state <= BRCM_WIFIAPI_STATE_STA_DISCONNECTING) {
		return BRCM_WIFI_STATION_IF;
	} else if (g_state >= BRCM_WIFIAPI_STATE_AP_ENABLING && g_state <= BRCM_WIFIAPI_STATE_AP_DISABLING) {
		return BRCM_WIFI_SOFT_AP_IF;
	}

	return BRCM_WIFI_NONE;
}

/**
 * Public interface
 */
int8_t WiFiStart(WiFi_InterFace_ID_t interface_id, const brcm_ap_config_t *ap_config)
{
	int8_t result = BRCM_STATUS_ERROR;

	if (interface_id == BRCM_WIFI_NONE || ((interface_id == BRCM_WIFI_SOFT_AP_IF) && (ap_config == NULL))) {
		/* Not a valid parameters */
		DPRINT("WiFiStart parameters not set\n");
		result = BRCM_STATUS_PARAM_FAILED;
		goto errout;
	}

	if (interface_id == brcm_get_op_mode()) {
		DPRINT("WiFiStart interface type already enabled\n");
		result = BRCM_STATUS_ALREADY_STARTED;
		goto errout;
	}
	ENTER_CRITICAL;

	result = brcm_api_start(interface_id, ap_config);

	LEAVE_CRITICAL;
errout:
	return result;
}

int8_t WiFiStop(void)
{
	int8_t result = BRCM_STATUS_ERROR;
	if (g_state == BRCM_WIFIAPI_STATE_NOT_STARTED) {
		DPRINT("BRCM_WiFiStop - no interface started\n");
		return BRCM_STATUS_SUCCESS;
	}
	DPRINT("BRCM_WiFiStop\n");
	ENTER_CRITICAL;

	// Disconnect if in STA mode and connected
	if (g_state == BRCM_WIFIAPI_STATE_STA_CONNECTED) {
		g_state = BRCM_WIFIAPI_STATE_STA_DISCONNECTING;

		result = brcm_leave_network();
		if (result == BRCM_STATUS_SUCCESS) {
			int res = sem_wait(&g_sem_disconnect);
			if (res) {
				EPRINT("BRCM_API sem_wait error %d\n", errno);
			}
		} else {
			EPRINT("%s Error: disconnect. \n", __func__);
		}
		// Still progress shutting down
	}
	// Stop AP mode if in AP mode
	if (brcm_get_op_mode() == BRCM_WIFI_SOFT_AP_IF) {
		VPRINT("Stop AP mode\n");
		g_state = BRCM_WIFIAPI_STATE_AP_DISABLING;
		result = brcm_stop_ap();
		// remove network
		if (g_network_id) {
			brcm_remove_network(g_network_id);
			free(g_network_id);
			g_network_id = NULL;
		}
	} else {
		result = BRCM_STATUS_SUCCESS;
	}

	// Stop supplicant
	if (result == BRCM_STATUS_SUCCESS) {
#ifdef CONFIG_SCSC_WLAN_AUTO_RECOVERY
		brcm_clean_recover();
#endif
		result = brcm_stop_supplicant();
		// CLean up local
		brcm_deinit();
	}

	LEAVE_CRITICAL;
	pthread_mutex_destroy(&mutex_state);
	g_mutex_initialized = FALSE;

	return result;
}

int8_t WiFiScanNetwork(void)
{
	ENTER_CRITICAL;
	int8_t result = BRCM_STATUS_NOT_STARTED;
	if (g_state != BRCM_WIFIAPI_STATE_NOT_STARTED) {
		result = brcm_start_scan();
	}
	LEAVE_CRITICAL;
	return result;
}

int8_t WiFiGetScanResults(brcm_scan_info_t **scan_results)
{
	ENTER_CRITICAL;
	int8_t result = BRCM_STATUS_NOT_STARTED;
	if (g_state != BRCM_WIFIAPI_STATE_NOT_STARTED) {
		result = brcm_get_api_scan_results(scan_results);
	}
	LEAVE_CRITICAL;
	return result;
}

int8_t WiFiFreeScanResults(brcm_scan_info_t **scan_results)
{
	ENTER_CRITICAL;
	brcm_scan_info_t *tmp, *tmp_bss_list = *scan_results;
	// This will handle passing a NULL pointer
	while (tmp_bss_list != NULL) {
		tmp = tmp_bss_list;
		tmp_bss_list = tmp_bss_list->next;
		free(tmp->sec_modes);
		tmp->sec_modes = NULL;
		free(tmp);
	}
	tmp = NULL;
	*scan_results = NULL;
	LEAVE_CRITICAL;
	return BRCM_STATUS_SUCCESS;
}

int8_t WiFiRegisterLinkCallback(brcm_network_link_callback_t link_up, brcm_network_link_callback_t link_down)
{
	int8_t result = BRCM_STATUS_ERROR;
	brcm_reason_t reason;
	memset(&reason, 0, sizeof(brcm_reason_t));
	//
	ENTER_CRITICAL;
	g_link_up = link_up;
	g_link_down = link_down;
#ifdef CONFIG_SCSC_WLAN_AUTO_RECOVERY
	g_recovery_data.link_up = link_up;
	g_recovery_data.link_down = link_down;
#endif
	LEAVE_CRITICAL;
	if (g_running) {
		result = brcm_check_status(reason.ssid, &reason.ssid_len, reason.bssid);
		if (result == BRCM_STATUS_SUCCESS) {
			if (brcm_is_connected()) {	// Mind SSID can be empty for hidden networks
				if (g_link_up) {
					g_link_up(&reason);
				}
			}
		}
	} else {
		result = BRCM_STATUS_SUCCESS;
	}
	return result;
}

int8_t WiFiRegisterScanCallback(network_scan_result_handler_t scan_result_handler)
{
	ENTER_CRITICAL;

	g_scan_result_handler = scan_result_handler;
#ifdef CONFIG_SCSC_WLAN_AUTO_RECOVERY
	g_recovery_data.scan_result_handler = scan_result_handler;
#endif
	LEAVE_CRITICAL;
	return BRCM_STATUS_SUCCESS;
}

int8_t WiFiNetworkJoin(uint8_t *ssid, uint8_t ssid_len, uint8_t *bssid, const brcm_security_config_t *security_config)
{
	ENTER_CRITICAL;
	int8_t result = BRCM_STATUS_NOT_STARTED;
	if (g_state == BRCM_WIFIAPI_STATE_STA_CONNECTED) {
		result = BRCM_STATUS_ALREADY_CONNECTED;
	} else if (g_state == BRCM_WIFIAPI_STATE_SUPPLICANT_RUNNING) {
		if ((security_config == NULL) || ((security_config != NULL) && (security_config->secmode != 0) && (security_config->passphrase[0] != '\0'))) {
			// Save network information is case of recovery handling
#ifdef CONFIG_SCSC_WLAN_AUTO_RECOVERY
			if (!g_recovering) {
				brcm_save_join(ssid, ssid_len, bssid, security_config);
			}
#endif
			//Call to join network
			result = brcm_join_network(ssid, ssid_len, bssid, security_config);
		} else {
			EPRINT("BRCM_API NetworkJoin - error on security config\n");
		}
	} else {
		EPRINT("BRCM_API Invalid Join request - current state: %s\n", brcm_state_strings[g_state]);
	}

	LEAVE_CRITICAL;
	return result;
}

int8_t WiFiNetworkLeave(void)
{
	ENTER_CRITICAL;
	int8_t result = BRCM_STATUS_NOT_STARTED;
	if (g_state != BRCM_WIFIAPI_STATE_NOT_STARTED) {
		if (brcm_is_connected()) {
			g_state = BRCM_WIFIAPI_STATE_STA_DISCONNECTING;
			result = brcm_leave_network();
		} else {
			result = BRCM_STATUS_NOT_CONNECTED;
			DPRINT("BRCM_API NetworkLeave - not connected\n");
		}
	}
	LEAVE_CRITICAL;
	return result;
}

int8_t WiFiGetTxPower(uint8_t *dbm)
{
	ENTER_CRITICAL;
	int8_t result = BRCM_STATUS_NOT_STARTED;
	if (g_state != BRCM_WIFIAPI_STATE_NOT_STARTED) {
		if (dbm != NULL) {
			result = brcm_get_tx_power(dbm);
		} else {
			if (brcm_is_connected()) {
				result = BRCM_STATUS_PARAM_FAILED;
			} else {
				result = BRCM_STATUS_NOT_CONNECTED;
			}
		}
	}
	LEAVE_CRITICAL;
	return result;
}

int8_t WiFiSetTxPower(uint8_t *dbm)
{
	ENTER_CRITICAL;
	int8_t result = BRCM_STATUS_NOT_STARTED;
	if (dbm != NULL) {
		if (g_state != BRCM_WIFIAPI_STATE_NOT_STARTED) {
			if (brcm_is_connected()) {
				result = brcm_set_tx_power(dbm, TRUE, TRUE);
			} else {
				DPRINT("Supplicant not connected - setting in NVRAM only\n");
				result = brcm_set_tx_power(dbm, TRUE, FALSE);
			}
		} else {
			DPRINT("Supplicant not running - setting in NVRAM only\n");
			result = brcm_set_tx_power(dbm, TRUE, FALSE);
		}
	} else {
		result = BRCM_STATUS_PARAM_FAILED;
		DPRINT("No value provided for tx power\n");
	}
	LEAVE_CRITICAL;
	return result;
}

int8_t WiFiGetMac(uint8_t *mac)
{
	ENTER_CRITICAL;
	int8_t result = BRCM_STATUS_NOT_STARTED;
	if (mac != NULL) {
		if (g_state != BRCM_WIFIAPI_STATE_NOT_STARTED) {
			result = brcm_get_mac(mac);
		}
	} else {
		result = BRCM_STATUS_PARAM_FAILED;
	}
	LEAVE_CRITICAL;
	return result;
}

int8_t WiFiGetRssi(int8_t *rssi)
{
	ENTER_CRITICAL;
	int8_t result = BRCM_STATUS_NOT_STARTED;
	if (rssi != NULL) {
		if (g_state != BRCM_WIFIAPI_STATE_NOT_STARTED) {
			if (brcm_is_connected()) {
				result = brcm_get_rssi(rssi);
			} else {
				result = BRCM_STATUS_NOT_CONNECTED;
			}
		}
	} else {
		result = BRCM_STATUS_PARAM_FAILED;
	}
	LEAVE_CRITICAL;
	return result;
}

int8_t WiFiGetChannel(int8_t *channel)
{
	ENTER_CRITICAL;
	int8_t result = BRCM_STATUS_NOT_STARTED;
	if (channel != NULL) {
		if (g_state != BRCM_WIFIAPI_STATE_NOT_STARTED) {
			if (brcm_is_connected()) {
				result = brcm_get_channel(channel);
			} else {
				result = BRCM_STATUS_NOT_CONNECTED;
			}
		}
	} else {
		result = BRCM_STATUS_PARAM_FAILED;
	}
	LEAVE_CRITICAL;
	return result;
}

int8_t WiFiIsConnected(uint8_t *ret, brcm_reason_t *details)
{
	ENTER_CRITICAL;
	int8_t result = BRCM_STATUS_NOT_CONNECTED;
	uint8_t count = 0;
	if (details != NULL) {
		memset(details, 0, sizeof(brcm_reason_t));
	}

	if (g_state == BRCM_WIFIAPI_STATE_STA_CONNECTED) {
		count = 1;
		result = BRCM_STATUS_SUCCESS;
		if (details != NULL) {
			result = brcm_check_status(details->ssid, &details->ssid_len, details->bssid);
		}
	} else if (g_state == BRCM_WIFIAPI_STATE_AP_CONNECTED) {
		count = g_num_sta_connected;
		result = BRCM_STATUS_SUCCESS;
	} else if (g_state == BRCM_WIFIAPI_STATE_NOT_STARTED) {
		result = BRCM_STATUS_NOT_STARTED;
	}

	*ret = count;
	LEAVE_CRITICAL;
	return result;
}

int8_t WiFiGetOpMode(WiFi_InterFace_ID_t *ret)
{
	*ret = brcm_get_op_mode();
	return BRCM_STATUS_SUCCESS;
}

int8_t WiFiSetCountryCode(const char *country_code)
{
	ENTER_CRITICAL;
	int8_t result = BRCM_STATUS_ERROR;

	bool write_to_nvram = FALSE;
	bool write_to_driver = FALSE;

	if (country_code != NULL) {
		if (brcm_get_op_mode() == BRCM_WIFI_STATION_IF) {
			// we can send to supplicant in station mode at any point.
			// it will however not be used until next scan an will not impact
			// existing connections.
			write_to_nvram = TRUE;
			write_to_driver = TRUE;
		} else {
			write_to_nvram = TRUE;
			write_to_driver = FALSE;
		}
		result = brcm_set_country_code(country_code, write_to_nvram, write_to_driver);
	} else {
		DPRINT("SetCountryCode - Countrycode not provided\n");
		result = BRCM_STATUS_PARAM_FAILED;
	}
	LEAVE_CRITICAL;
	return result;
}

int8_t WiFiGetCountryCode(char *country_code)
{
	ENTER_CRITICAL;
	int8_t result = BRCM_STATUS_NOT_STARTED;
	if (g_state != BRCM_WIFIAPI_STATE_NOT_STARTED) {
		result = brcm_get_country_code(country_code);
	} else {
		DPRINT("GetCountryCode - Interface not stared \n");
	}
	LEAVE_CRITICAL;
	return result;
}

int8_t WiFiSaveConfig(void)
{
	int8_t result = BRCM_STATUS_NOT_SUPPORTED;
#ifdef CONFIG_BRCM_WIFI_FILESYSTEM_SUPPORT
	result = BRCM_STATUS_NOT_STARTED;

	ENTER_CRITICAL;
	if (brcm_get_op_mode() == BRCM_WIFI_STATION_IF) {
		result = brcm_save_config();
	} else if (g_state == BRCM_WIFIAPI_STATE_NOT_STARTED) {
		DPRINT("WiFiSaveConfig - system not started\n");
	} else {
		DPRINT("WiFiSaveConfig - not allowed during AP mode\n");
		result = BRCM_STATUS_NOT_ALLOWED;
	}
	LEAVE_CRITICAL;
#endif
	return result;
}

void WiFiinitConfig(void)
{
	brcm_wpa_reopen();
	g_state = BRCM_WIFIAPI_STATE_STA_CONNECTED;
}
