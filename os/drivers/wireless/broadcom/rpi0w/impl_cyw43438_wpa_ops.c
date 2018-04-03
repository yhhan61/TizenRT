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

//#include <common/defs.h>
#include <tinyara/kmalloc.h>
#include <net/if.h>
#include <net/lwip/netif.h>
#include "wpa_driver_copy.h"
#include "wwd_constants.h"
#include "wwd_structures.h"
#include "wwd_management.h"
#include "wwd_network.h"
#include "wwd_internal.h"
#include "wwd_thread_internal.h"
#include "wwd_wifi.h"
#include "RTOS/wwd_rtos_interface.h"

#ifndef WPA_SUPPLICANT_SUPPORT
#define WPA_SUPPLICANT_SUPPORT
#endif
#define MAX_SCAN_RESULT_BUFF     (200) /* Sure good enough? */
#define DEBUG_WPA_OPS

#define CYW_MAC_FORMAT "%02x:%02x:%02x:%02x:%02x:%02x"
#define CYW_MAC_STR(x) (x)[0], (x)[1], (x)[2], (x)[3], (x)[4], (x)[5]

enum wpa_alg {
	WPA_ALG_NONE,
	WPA_ALG_WEP,
	WPA_ALG_TKIP,
	WPA_ALG_CCMP,
	WPA_ALG_IGTK,
	WPA_ALG_PMK,
	WPA_ALG_GCMP,
	WPA_ALG_SMS4,
	WPA_ALG_KRK,
	WPA_ALG_GCMP_256,
	WPA_ALG_CCMP_256,
	WPA_ALG_BIP_GMAC_128,
	WPA_ALG_BIP_GMAC_256,
	WPA_ALG_BIP_CMAC_256
};

typedef struct {
	void *global;
	void *ctx;
	char ifname[IFNAMSIZ + 1];
	struct wpa_driver_capa capa;
} CYW43438_DRV_T;

typedef struct
{
	uint32_t current_index;
	struct {
		wl_bss_info_t info;
		uint8_t ie_data[1024];/* Sure good enough? */
	} bss[MAX_SCAN_RESULT_BUFF];
} __bss_list_t;

static const char *CYW_IFNAME = "wl1";
static __bss_list_t	*__bss_list = NULL;
static wiced_scan_result_t *__ap_scan_result_buff;
static wiced_ssid_t __current_ssid;
static bool __scan_completed_ok;
static host_semaphore_type_t __scan_completed_semaphore;
static pthread_t __associate_ok_thread_t;
static pthread_mutex_t __wpa_interface_mutex;

void *cyw43438_init2(void *ctx, const char *ifname, void *global_priv);
void cyw43438_deinit(void *priv);
const u8 *cyw43438_get_mac_addr(void *priv);
int cyw43438_get_capa(void *priv, struct wpa_driver_capa *capa);
int cyw43438_scan2(void *priv, struct wpa_driver_scan_params *params);
struct wpa_scan_results *cyw43438_get_scan_results2(void *priv);
int cyw43438_associate(void *priv, struct wpa_driver_associate_params *params);
int cyw43438_deauthenticate(void *priv, const u8 *addr, int reason_code);
int cyw43438_set_ap(void *priv, struct wpa_driver_ap_params *settings);
int cyw43438_stop_ap(void *priv);
int cyw43438_del_station(void *priv, const u8 *addr, int reason);
int cyw43438_set_country(void *priv, const char *country_code);
int cyw43438_get_country(void *priv, char *country_code);
int cyw43438_set_rts(void *priv, int rts);
int cyw43438_signal_poll(void *priv, struct wpa_signal_info *si);
int cyw43438_set_frag(void *priv, int frag_threshold);
int cyw43438_get_bssid(void *priv, u8 *bssid);
struct hostapd_hw_modes *cyw43438_get_hw_feature_data(void *priv, u16 *num_modes, u16 *flags);
int cyw43438_hapd_send_eapol(void *priv, const u8 *addr, const u8 *data, size_t data_len, int encrypt, const u8 *own_addr, u32 flags);
int cyw43438_seqnum(const char *ifname, void *priv, const u8 *addr, int idx, u8 *seq);
int cyw43438_set_tx_power(void *priv, int dbm);
int cyw43438_get_tx_power(void *priv);
int cyw43438_set_panic(void *priv);
static int32_t __get_last_scan_result_index(void);
static void __set_last_scan_result_index_increase(void);
static void __set_last_scan_result_index_zero(void);
static uint32_t __get_num_scan_result(void);
static wiced_scan_result_t *get_one_scan_result(unsigned int index);
static int32_t __get_ies(const uint8_t *bssid, void* ies, uint32_t *ies_len);
static wl_bss_info_t *__get_wl_bss_info(const uint8_t *bssid);
static void __reset_current_ssid(void);
static wiced_bool_t __is_same_data(const void* d1, const void *d2, uint32_t size);
static void __send_associate_ok_event_to_wpa_supplicant(void* ctx);
/*
struct wireless_dev *cyw43438_add_virtual_intf(struct wiphy *wiphy, const char *name, enum nl80211_iftype type, u32 *flags, struct vif_params *params);
*/
int cyw43438_set_key(const char *ifname, void *priv, enum wpa_alg alg, const u8 *mac_addr, int key_idx, int set_tx, const u8 *seq, size_t seq_len, const u8 *key, size_t key_len);
int cyw43438_get_ssid(void *priv, u8 *ssid);
int cyw43438_sta_remove(void *priv, const u8 *addr);
int cyw43438_sta_deauth(void *priv, const u8 *own_addr, const u8 *addr, int reason);
#if 0 /* Test code */
extern void print_scan_result(wiced_scan_result_t* record);
#endif
extern int eth_send_eapol(const u8 *src_mac_addr, const u8 *dst_mac_addr,	const u8 *buf, u16 len, u16 proto);
extern wiced_country_code_t wwd_get_country( void );
extern struct netif* get_cyw43438_wlan_netif(void);

#ifdef	SUPPORT_DUMP_BYTES
static void wwd_dump_bytes(void *data, uint32_t bytes);
#endif

static void wpa_wiced_scan_result_handler(wiced_scan_result_t** result_ptr, void* user_data, wiced_scan_status_t status)
{
	wl_bss_info_t *bss_info;
	uint32_t bss_index;

	if (result_ptr == NULL) {
		/* finished a scan*/
		host_rtos_set_semaphore(&__scan_completed_semaphore, WICED_FALSE);

		return;
	}

#if 0
	do {
	/* wpa_supplicant removes and sorts duplicates.
	 * So no need for duplicate testing here.
	 */
		int32_t scaned_ap_count;
		wiced_mac_t *tmp_mac;

		scaned_ap_count = __get_num_scan_result();
		tmp_mac = &(*result_ptr)->BSSID;

		do  {
			if (__is_same_data(tmp_mac->octet, bss_list->bss[--scaned_ap_count].info.BSSID.octet, 6) == WICED_TRUE)
					return;
		} while (scaned_ap_count);
	} while (0);
#endif

	bss_index = __get_last_scan_result_index();
	bss_info = (*result_ptr)->bi;
	memcpy(&__bss_list->bss[bss_index].info,	bss_info, sizeof(wl_bss_info_t));
	memcpy(__bss_list->bss[bss_index].ie_data, ((uint8_t *)bss_info) + bss_info->ie_offset, bss_info->ie_length);

	__set_last_scan_result_index_increase();
	*result_ptr = __ap_scan_result_buff + __get_last_scan_result_index();
}

wwd_result_t wpa_wiced_wifi_find_ap(const wiced_ssid_t* ssid, wiced_scan_result_t* ap_info, const uint16_t* optional_channel_list)
{
    wwd_result_t result;

	result = (wwd_result_t) wwd_wifi_scan(
			WICED_SCAN_TYPE_ACTIVE,
			WICED_BSS_TYPE_ANY,
			ssid,
			NULL,
			optional_channel_list,
			NULL,
			wpa_wiced_scan_result_handler,
			(wiced_scan_result_t **)&ap_info,
			NULL,
			WWD_STA_INTERFACE);

	if(result != WWD_SUCCESS)
		return WWD_BADARG;

    if (host_rtos_get_semaphore(&__scan_completed_semaphore, 1000000, WICED_FALSE) == WWD_SUCCESS)
		return WWD_SUCCESS;

	return WWD_TIMEOUT;
}

static uint32_t __get_num_scan_result(void)
{
	return __get_last_scan_result_index();
}

wiced_scan_result_t *get_one_scan_result(unsigned int index)
{
	if(index >= MAX_SCAN_RESULT_BUFF)
		return NULL;

	return __ap_scan_result_buff + index;
}

void __set_default_cyw43438_drv(CYW43438_DRV_T *drv)
{
	drv->capa.flags = WPA_DRIVER_FLAGS_AP;
	drv->capa.flags |= WPA_DRIVER_FLAGS_PROBE_RESP_OFFLOAD;
	drv->capa.flags |= WPA_DRIVER_FLAGS_AP_UAPSD;
	drv->capa.flags |= WPA_DRIVER_FLAGS_INACTIVITY_TIMER;
	drv->capa.flags |= WPA_DRIVER_FLAGS_SANE_ERROR_CODES;
	drv->capa.flags |= WPA_DRIVER_FLAGS_SET_KEYS_AFTER_ASSOC_DONE;
	drv->capa.flags |= WPA_DRIVER_FLAGS_EAPOL_TX_STATUS;
	drv->capa.flags |= WPA_DRIVER_FLAGS_AP_TEARDOWN_SUPPORT;
	drv->capa.flags |= WPA_DRIVER_FLAGS_4WAY_HANDSHAKE;
	drv->capa.key_mgmt = WPA_DRIVER_CAPA_KEY_MGMT_WPA_NONE | WPA_DRIVER_CAPA_KEY_MGMT_WPA | WPA_DRIVER_CAPA_KEY_MGMT_WPA_PSK | WPA_DRIVER_CAPA_KEY_MGMT_WPA2 | WPA_DRIVER_CAPA_KEY_MGMT_WPA2_PSK;
	drv->capa.enc = WPA_DRIVER_CAPA_ENC_WEP40 | WPA_DRIVER_CAPA_ENC_WEP104 | WPA_DRIVER_CAPA_ENC_TKIP | WPA_DRIVER_CAPA_ENC_CCMP | WPA_DRIVER_CAPA_ENC_WEP128;
	drv->capa.auth = WPA_DRIVER_AUTH_OPEN | WPA_DRIVER_AUTH_SHARED;
	drv->capa.max_remain_on_chan = 5000;
	drv->capa.wmm_ac_supported = 1;
	drv->capa.max_scan_ssids = 100;
	drv->capa.max_sched_scan_ssids = 100;
	drv->capa.sched_scan_supported = 0;
	drv->capa.max_match_sets = 16;
	drv->capa.max_remain_on_chan = 5000;
	drv->capa.max_stations = 1;
	drv->capa.probe_resp_offloads = WPA_DRIVER_PROBE_RESP_OFFLOAD_WPS2 | WPA_DRIVER_PROBE_RESP_OFFLOAD_P2P;
	drv->capa.max_acl_mac_addrs = 10;
}

/**
 * init2 - Initialize driver interface (with global data)
 * @ctx: context to be used when calling wpa_supplicant functions,
 * e.g., wpa_supplicant_event()
 * @ifname: interface name, e.g., wlan0
 * @global_priv: private driver global data from global_init()
 * Returns: Pointer to private data, %NULL on failure
 *
 * This function can be used instead of init() if the driver wrapper
 * uses global data.
 */
void *cyw43438_init2(void *ctx, const char *ifname, void *global_priv)
{
	struct netif *dev;
	CYW43438_DRV_T *cyw43438_drv;

	UNUSED_PARAMETER(ctx);
	UNUSED_PARAMETER(ifname);
	UNUSED_PARAMETER(global_priv);

	__scan_completed_ok = false;
	__reset_current_ssid();

	 if (pthread_mutex_init(&__wpa_interface_mutex, NULL))
		 return NULL;

	__bss_list = (__bss_list_t *)kmm_zalloc(sizeof(__bss_list_t));
	if (__bss_list == NULL)
		return NULL;

	__ap_scan_result_buff = (wiced_scan_result_t *)kmm_zalloc(sizeof(wiced_scan_result_t) * MAX_SCAN_RESULT_BUFF);
	if (__ap_scan_result_buff == NULL)
		return NULL;

    if (host_rtos_init_semaphore(&__scan_completed_semaphore) != WWD_SUCCESS )
        return NULL;

    cyw43438_drv = (CYW43438_DRV_T *)kmm_zalloc(sizeof(CYW43438_DRV_T));
	if (!cyw43438_drv)
		return NULL;

	__set_default_cyw43438_drv(cyw43438_drv);
	cyw43438_drv->ctx = ctx;

	dev = get_cyw43438_wlan_netif();
	if (!dev)
		return NULL;

	netdev_ifup(dev);

	/* Free drv context if driver start was unsuccessful (interface is not up) */
	if ((dev->d_flags & IFF_UP) == 0)
		return NULL;

	return cyw43438_drv;
}

/**
 * deinit - Deinitialize driver interface
 * @priv: private driver interface data from init()
 *
 * Shut down driver interface and processing of driver events. Free
 * private data buffer if one was allocated in init() handler.
 */
void cyw43438_deinit(void *priv)
{

	pthread_mutex_destroy(&__wpa_interface_mutex);

	if (__bss_list != NULL)
		kmm_free(__bss_list);

	if (__ap_scan_result_buff != NULL)
		kmm_free(__ap_scan_result_buff);

	if (priv)
		kmm_free(priv);

	host_rtos_deinit_semaphore(&__scan_completed_semaphore);
}

/**
 * get_mac_addr - Get own MAC address
 * @priv: private driver interface data
 *
 * Returns: Pointer to own MAC address or %NULL on failure
 *
 * This optional function can be used to get the own MAC address of the
 * device from the driver interface code. This is only needed if the
 * l2_packet implementation for the OS does not provide easy access to
 * a MAC address. */
const u8 *cyw43438_get_mac_addr(void *priv)
{
	struct netif *dev;

	UNUSED_PARAMETER(priv);

	dev = get_cyw43438_wlan_netif();
	if (dev)
		return (const u8 *)(dev->d_mac.ether_addr_octet);

	return NULL;
}


/**
 * get_capa - Get driver capabilities
 * @priv: private driver interface data
 *
 * Returns: 0 on success, -1 on failure
 *
 * Get driver/firmware/hardware capabilities.
 */
int cyw43438_get_capa(void *priv, struct wpa_driver_capa *capa)
{
	CYW43438_DRV_T *cyw43438_drv;

	cyw43438_drv = (CYW43438_DRV_T *)priv;

	memcpy(capa, &(cyw43438_drv->capa), sizeof(struct wpa_driver_capa));

	return 0;
}

/**
 * scan2 - Request the driver to initiate scan
 * @priv: private driver interface data
 * @params: Scan parameters
 *
 * Returns: 0 on success, -1 on failure
 *
 * Once the scan results are ready, the driver should report scan
 * results event for wpa_supplicant which will eventually request the
 * results with wpa_driver_get_scan_results2().
 */
static int __cyw43438_scan2(void *priv, struct wpa_driver_scan_params *request)
{
	uint32_t try_count;
	CYW43438_DRV_T *drv;

	UNUSED_PARAMETER(request);

	__scan_completed_ok = false;

	__set_last_scan_result_index_zero();

	drv = (CYW43438_DRV_T*)priv;

	wpa_supplicant_event_send(drv->ctx, EVENT_SCAN_STARTED, NULL);

	try_count = 1;
	do {
		if (wpa_wiced_wifi_find_ap(NULL, __ap_scan_result_buff + 0, NULL) != WWD_SUCCESS)
			return -1;
	} while(--try_count);

	__scan_completed_ok = true;

	wpa_supplicant_event_send(drv->ctx, EVENT_SCAN_RESULTS, NULL);
//	wpa_supplicant_event(drv->ctx, EVENT_SCAN_RESULTS, NULL);

	return 0;
}

int cyw43438_scan2(void *priv, struct wpa_driver_scan_params *request)
{
	int ret;

	pthread_mutex_lock(&__wpa_interface_mutex);
	ret = __cyw43438_scan2(priv, request);
	pthread_mutex_unlock(&__wpa_interface_mutex);

	return ret;
}

static int cyw43438_channel_to_frequency(int chan, int band)
{
	if (chan <= 0)
		return 0;

	switch (band) {
	case 0: /* BAND_2GHZ: */
		if (chan == 14)
			return 2484;
		else if (chan < 14)
			return 2407 + chan * 5;
		break;
	case 1: /* BAND_5GHZ */
		if (chan >= 182 && chan <= 196)
			return 4000 + chan * 5;
		else
			return 5000 + chan * 5;
		break;
	default:
		break;
	}
	return 0;
}

static void cyw43438_add_wpa_scan_entry(struct wpa_scan_results *results, wl_bss_info_t *bi, void *ie_data)
{
	struct wpa_scan_res *scan_res;
	wl_chanspec_t band;
	uint16_t channel;
	uint32_t ie_len;
	uint8_t *scan_res_ies;

	ie_len = bi->ie_length;
	scan_res = kmm_zalloc(sizeof(*scan_res) + ie_len);
	if (!scan_res) {
#ifdef DEBUG_WPA_OPS
		printf("Failed to allocate memory for scan entry (wpa_scan_res)\n");
#endif
		return;
	}

	channel = bi->ctl_ch;
	band = wwd_channel_to_wl_band(channel) == WL_CHANSPEC_BAND_5G ? 1 : 0;
	memcpy((char*)&scan_res->bssid, (char*)&bi->BSSID, ETH_ALEN);
	scan_res->freq = cyw43438_channel_to_frequency(channel, band);
	scan_res->beacon_int = bi->beacon_period;
	scan_res->caps = bi->capability;
	scan_res->qual = bi->RSSI;
	scan_res->tsf = 0;
	scan_res->ie_len = ie_len;

	if (ie_len > 0) {
		scan_res_ies = (u8 *)(scan_res + 1);
		memcpy(scan_res_ies, ie_data, ie_len);
	}

	results->res[results->num++] = scan_res;
}

/**
 * get_scan_results2 - Fetch the latest scan results
 * @priv: private driver interface data
 *
 * Returns: Allocated buffer of scan results (caller is responsible for
 * freeing the data structure) on success, NULL on failure
 */
struct wpa_scan_results *cyw43438_get_scan_results2(void *priv)
{
	struct wpa_scan_results *results;
	wl_bss_info_t *bi;
	uint32_t num_scan_count;

	UNUSED_PARAMETER(priv);

	results = NULL;
	bi = NULL;

	if(__scan_completed_ok == false)
		return NULL;

	num_scan_count = __get_num_scan_result();
	if (num_scan_count == 0)
		return NULL;

	results = kmm_zalloc(sizeof(*results));
	if (results == NULL)
		return NULL;

	results->res = kmm_zalloc(sizeof(*results->res) * num_scan_count);
	if (results->res == NULL) {
#ifdef DEBUG_WPA_OPS
		printf("Failed to allocate memory for wpa_scan_res array\n");
#endif
		kmm_free(results);

		return NULL;
	}

	do {
		--num_scan_count;
		cyw43438_add_wpa_scan_entry(results, &__bss_list->bss[num_scan_count].info, __bss_list->bss[num_scan_count].ie_data);
	} while (num_scan_count);

#if 0 /* Test code */
#ifdef DEBUG_WPA_OPS
		do 	{
			uint32_t tot_cnt;
			uint32_t i;

			tot_cnt = __get_num_scan_result();
			for (i = 0; i < tot_cnt; ++i) {
				printf("[%2d] ", i + 1);
				print_scan_result(__ap_scan_result_buff + i);
			}
		} while (0);
#endif
#endif /* Test code */

	return results;
}

/**
 * associate - Request driver to associate
 * @priv: private driver interface data
 * @params: association parameters
 *
 * Returns: 0 on success, -1 on failure
 */
static void* __associate_ok_event(void* data)
{
	union wpa_event_data event;

	memset(&event, 0, sizeof(event));
	event.assoc_info.authorized = 1;
	usleep(1000000);
	wpa_supplicant_event(data, EVENT_ASSOC, &event);

	return NULL;
}

static wiced_bool_t __is_same_data(const void* d1, const void *d2, uint32_t size)
{
	uint8_t *u1;
	uint8_t *u2;
	uint32_t i;

	u1 = (uint8_t *)d1;
	u2 = (uint8_t *)d2;

	for (i = 0; i < size - 1; ++i)
		if (u1[i] != u2[i])
			return WICED_FALSE;

	return WICED_TRUE;
}
static void __reset_current_ssid(void)
{
	memset(&__current_ssid, 0x00, sizeof(wiced_ssid_t));
}

static int32_t __get_ies(const uint8_t *bssid, void* ies, uint32_t *ies_len)
{
	int32_t count;
	int32_t index;
	wl_bss_info_t *bi;

	count = __get_num_scan_result();
	if (count <= 0)
		return -1;

	index = count - 1;
	do {
		bi = &(__bss_list->bss[index].info);
		if (__is_same_data(bssid, bi->BSSID.octet, 6) == WICED_TRUE) {
			ies = __bss_list->bss[index].ie_data;
			*ies_len = bi->ie_length;

			return 0;
		}
	} while (index--);

	return -1;
}

static wl_bss_info_t *__get_wl_bss_info(const uint8_t *bssid)
{
	int32_t count;
	int32_t index;
	wl_bss_info_t *bi;

	count = __get_num_scan_result();
	if (count <= 0)
		return NULL;

	index = count - 1;
	do {
		bi = &(__bss_list->bss[index].info);
		if (__is_same_data(bssid, bi->BSSID.octet, 6) == WICED_TRUE) {

			return bi;
		}
	} while (index--);

	return NULL;
}

static void __send_associate_ok_event_to_wpa_supplicant(void* ctx)
{
	pthread_create(&__associate_ok_thread_t, NULL, __associate_ok_event, ctx);
}

/**
 * associate - Request driver to associate
 * @priv: private driver interface data
 * @params: association parameters
 *
 * Returns: 0 on success, -1 on failure
 */
static int __cyw43438_associate(void *priv, struct wpa_driver_associate_params *request)
{
	union wpa_event_data event;
	wiced_security_t security;
	uint32_t passphrase_length;
	uint32_t status;
	const uint8_t *passphrase;
	uint32_t total_scaned_ap_count;
	uint32_t i;
	wiced_scan_result_t *scan_result;
	uint32_t found_bssid;

	UNUSED_PARAMETER(priv);

	pthread_join(__associate_ok_thread_t, (void **)&status);

	__reset_current_ssid();

	total_scaned_ap_count = __get_num_scan_result();
	security = 0;
	found_bssid = 0;
	for (i = 0; i < total_scaned_ap_count; ++i) {
		scan_result = __ap_scan_result_buff + i;
		if (__is_same_data(request->bssid, scan_result->BSSID.octet, 6) == WICED_TRUE) {
			security = scan_result->security;
			found_bssid = 1;
			break;
		}
	}
	if (found_bssid == 0)
		goto auth_reject;

	passphrase = (const uint8_t *)request->passphrase;
	if (passphrase)
		passphrase_length = strnlen((char *)passphrase, 63);
	else
		passphrase_length = 0;

	__current_ssid.length = request->ssid_len;
	memcpy(__current_ssid.value, request->ssid, __current_ssid.length);

    if (wwd_wifi_join(
    		&__current_ssid,
			security,
			passphrase,
			passphrase_length,
			NULL) !=  WWD_SUCCESS)
			goto auth_reject;

	__send_associate_ok_event_to_wpa_supplicant((void *)((CYW43438_DRV_T *)priv)->ctx);

	return 0;

auth_reject:
	__get_ies(request->bssid, (uint8_t *)event.assoc_reject.resp_ies, &event.assoc_reject.resp_ies_len);
	event.assoc_reject.status_code = 1;
	event.assoc_reject.bssid = request->bssid;
	wpa_supplicant_event_send(((CYW43438_DRV_T *)priv)->ctx, EVENT_ASSOC_REJECT, &event);

	return -1;
}
 int cyw43438_associate(void *priv, struct wpa_driver_associate_params *request)
{
	 int ret;

	 pthread_mutex_lock(&__wpa_interface_mutex);
	 ret = __cyw43438_associate(priv, request);
	 pthread_mutex_unlock(&__wpa_interface_mutex);

	 return ret;
}

/**
 * deauthenticate - Request driver to deauthenticate
 * @priv: private driver interface data
 * @addr: peer address (BSSID of the AP)
 * @reason_code: 16-bit reason code to be sent in the deauthentication
 *	frame
 *
 * Returns: 0 on success, -1 on failure
 */
static int __cyw43438_deauthenticate(void *priv, const u8 *addr, int reason_code)
{
	wl_bss_info_t *bss_info;

	UNUSED_PARAMETER(priv);
	UNUSED_PARAMETER(reason_code);

	if (reason_code == 3) /* 3:WLAN_REASON_DEAUTH_LEAVING */ {
		bss_info = __get_wl_bss_info(addr);
		if (bss_info == NULL)
			return -1;
		if (wwd_wifi_leave(WWD_STA_INTERFACE) != WWD_SUCCESS)
			return -1;

		__reset_current_ssid();
	}

	return 0;
}

int cyw43438_deauthenticate(void *priv, const u8 *addr, int reason_code)
{
	int ret;

	pthread_mutex_lock(&__wpa_interface_mutex);
	ret = __cyw43438_deauthenticate(priv, addr, reason_code);
	pthread_mutex_unlock(&__wpa_interface_mutex);

	return ret;
}

/**
 * resume - Notification on system resume/thaw event
 * @priv: Private driver interface data
 */
void cyw43438_resume(void *priv)
{
	UNUSED_PARAMETER(priv);
}

/**
 * suspend - Notification on system suspend/hibernate event
 * @priv: Private driver interface data
 */
void cyw43438_suspend(void *priv)
{
	UNUSED_PARAMETER(priv);
}

/**
 * get_ifname - Get interface name
 * @priv: private driver interface data
 *
 * Returns: Pointer to the interface name. This can differ from the
 * interface name used in init() call. Init() is called first.
 *
 * This optional function can be used to allow the driver interface to
 * replace the interface name with something else, e.g., based on an
 * interface mapping from a more descriptive name.
 */
const char *cyw43438_get_ifname(void *priv)
{
	UNUSED_PARAMETER(priv);

	return (const char *)CYW_IFNAME;
}

/**
 * send_ether - Send an ethernet packet (AP only)
 * @priv: private driver interface data
 * @dst: Destination MAC address
 * @src: Source MAC address
 * @proto: Ethertype
 * @data: EAPOL packet starting with IEEE 802.1X header
 * @data_len: Length of the EAPOL packet in octets
 * Returns: 0 on success, -1 on failure
 */
int cyw43438_send_ether(void *priv, const u8 *dst, const u8 *src, u16 proto, const u8 *data, size_t data_len)
{
	return eth_send_eapol(src, dst, data, data_len, proto);
}

/**
 * set_ap - Set Beacon and Probe Response information for AP mode
 * @priv: Private driver interface data
 * @params: Parameters to use in AP mode
 *
 * This function is used to configure Beacon template and/or extra IEs
 * to add for Beacon and Probe Response frames for the driver in
 * AP mode. The driver is responsible for building the full Beacon
 * frame by concatenating the head part with TIM IE generated by the
 * driver/firmware and finishing with the tail part. Depending on the
 * driver architectue, this can be done either by using the full
 * template or the set of additional IEs (e.g., WPS and P2P IE).
 * Similarly, Probe Response processing depends on the driver design.
 * If the driver (or firmware) takes care of replying to Probe Request
 * frames, the extra IEs provided here needs to be added to the Probe
 * Response frames.
 *
 * Returns: 0 on success, -1 on failure
 */
int cyw43438_set_ap(void *priv, struct wpa_driver_ap_params *params)
{
	UNUSED_PARAMETER(priv);
	UNUSED_PARAMETER(params);

	return -1;
}

/**
 * stop_ap - Removes beacon from AP
 * @priv: Private driver interface data
 * Returns: 0 on success, -1 on failure (or if not supported)
 *
 * This optional function can be used to disable AP mode related
 * configuration. Unlike deinit_ap, it does not change to station
 * mode.
 */
int cyw43438_stop_ap(void *priv)
{
	UNUSED_PARAMETER(priv);

	return -1;
}

/**
 * sta_remove - Remove a station entry (AP only)
 * @priv: Private driver interface data
 * @addr: MAC address of the station to be removed
 * Returns: 0 on success, -1 on failure
 */
int cyw43438_sta_remove(void *priv, const u8 *addr)
{
	return cyw43438_del_station(priv, addr, 0);
}

/**
 * sta_deauth - Deauthenticate a station (AP only)
 * @priv: Private driver interface data
 * @own_addr: Source address and BSSID for the Deauthentication frame
 * @addr: MAC address of the station to deauthenticate
 * @reason: Reason code for the Deauthentiation frame
 * Returns: 0 on success, -1 on failure
 *
 * This function requests a specific station to be deauthenticated and
 * a Deauthentication frame to be sent to it.
 */
int cyw43438_sta_deauth(void *priv, const u8 *own_addr, const u8 *addr, int reason)
{
	UNUSED_PARAMETER(own_addr);

	return cyw43438_del_station(priv, addr, reason);
}

/**
 * set_frag - Set fragmentation threshold
 * @priv: Private driver interface data
 * @frag: Fragmentation threshold in octets
 * Returns: 0 on success, -1 on failure
 */
int cyw43438_set_frag(void *priv, int frag)
{
	UNUSED_PARAMETER(priv);
	UNUSED_PARAMETER(frag);

	return -1;
}

/**
 * set_rts - Set RTS threshold
 * @priv: Private driver interface data
 * @rts: RTS threshold in octets
 * Returns: 0 on success, -1 on failure
 */
int cyw43438_set_rts(void *priv, int rts)
{
	UNUSED_PARAMETER(priv);
	UNUSED_PARAMETER(rts);

	return -1;
}

/**
 * signal_poll - Get current connection information
 * @priv: Private driver interface data
 * @signal_info: Connection info structure
 */
int cyw43438_signal_poll(void *priv, struct wpa_signal_info *signal_info)
{
	UNUSED_PARAMETER(priv);
	UNUSED_PARAMETER(signal_info);

	return -1;
}

/**
 * get_country - Get country
 * @priv: Private driver interface data
 * @alpha2: Buffer for returning country code (at least 3 octets)
 * Returns: 0 on success, -1 on failure
 */
int cyw43438_get_country(void *priv, char *alpha2)
{
	wiced_country_code_t cc;
	u8 buf[4] = {'\0', '\0', '\0', '\0'};

	UNUSED_PARAMETER(priv);

	cc = wwd_get_country();

	buf[0] = cc & 0xff;
	buf[1] = (cc >> 8) & 0xff;
	buf[2] = (cc >> 16) & 0xff;

	memcpy(alpha2, buf, 4);

	return 0;
}

/**
 * set_country - Set country
 * @priv: Private driver interface data
 * @alpha2: country to which to switch to
 * Returns: 0 on success, -1 on failure
 *
 * This function is for drivers which support some form
 * of setting a regulatory domain.
 */
int cyw43438_set_country(void *priv, const char *alpha2)
{
	uint32_t cc;

	UNUSED_PARAMETER(priv);

	cc = (wiced_country_code_t)alpha2[0];
	cc |= (wiced_country_code_t)alpha2[1] << 8;
	cc |= (wiced_country_code_t)alpha2[2] << 16;

	wwd_set_country((wiced_country_code_t)cc);

	return 0;
}

/**
 * get_bssid - Get the current BSSID
 * @priv: private driver interface data
 * @bssid: buffer for BSSID (ETH_ALEN = 6 bytes)
 *
 * Returns: 0 on success, -1 on failure
 *
 * Query kernel driver for the current BSSID and copy it to bssid.
 * Setting bssid to 00:00:00:00:00:00 is recommended if the STA is not
 * associated.
 */
int cyw43438_get_bssid(void *priv, u8 *bssid)
{
	wiced_mac_t w_bssid;

	UNUSED_PARAMETER(priv);

	if (wwd_wifi_get_bssid(&w_bssid) != WWD_SUCCESS)
		return -1;

	bssid[0] = w_bssid.octet[0];
	bssid[1] = w_bssid.octet[1];
	bssid[2] = w_bssid.octet[2];
	bssid[3] = w_bssid.octet[3];
	bssid[4] = w_bssid.octet[4];
	bssid[5] = w_bssid.octet[5];

	return 0;
}

/**
 * get_hw_feature_data - Get hardware support data (channels and rates)
 * @priv: Private driver interface data
 * @num_modes: Variable for returning the number of returned modes
 * flags: Variable for returning hardware feature flags
 * Returns: Pointer to allocated hardware data on success or %NULL on
 * failure. Caller is responsible for freeing this.
 */
struct hostapd_hw_modes *cyw43438_get_hw_feature_data(void *priv, u16 *num_modes, u16 *flags)
{
	UNUSED_PARAMETER(priv);
	UNUSED_PARAMETER(num_modes);
	UNUSED_PARAMETER(flags);

	return NULL;
}

/**
 * set_key - Configure encryption key
 * @ifname: Interface name (for multi-SSID/VLAN support)
 * @priv: private driver interface data
 * @alg: encryption algorithm (%WPA_ALG_NONE, %WPA_ALG_WEP,
 *	%WPA_ALG_TKIP, %WPA_ALG_CCMP, %WPA_ALG_IGTK, %WPA_ALG_PMK,
 *	%WPA_ALG_GCMP, %WPA_ALG_GCMP_256, %WPA_ALG_CCMP_256,
 *	%WPA_ALG_BIP_GMAC_128, %WPA_ALG_BIP_GMAC_256,
 *	%WPA_ALG_BIP_CMAC_256);
 *	%WPA_ALG_NONE clears the key.
 * @addr: Address of the peer STA (BSSID of the current AP when setting
 *	pairwise key in station mode), ff:ff:ff:ff:ff:ff for
 *	broadcast keys, %NULL for default keys that are used both for
 *	broadcast and unicast; when clearing keys, %NULL is used to
 *	indicate that both the broadcast-only and default key of the
 *	specified key index is to be cleared
 * @key_idx: key index (0..3), usually 0 for unicast keys; 0..4095 for
 *	IGTK
 * @set_tx: configure this key as the default Tx key (only used when
 *	driver does not support separate unicast/individual key
 * @seq: sequence number/packet number, seq_len octets, the next
 *	packet number to be used for in replay protection; configured
 *	for Rx keys (in most cases, this is only used with broadcast
 *	keys and set to zero for unicast keys); %NULL if not set
 * @seq_len: length of the seq, depends on the algorithm:
 *	TKIP: 6 octets, CCMP/GCMP: 6 octets, IGTK: 6 octets
 * @key: key buffer; TKIP: 16-byte temporal key, 8-byte Tx Mic key,
 *	8-byte Rx Mic Key
 * @key_len: length of the key buffer in octets (WEP: 5 or 13,
 *	TKIP: 32, CCMP/GCMP: 16, IGTK: 16)
 *
 * Returns: 0 on success, -1 on failure
 *
 * Configure the given key for the kernel driver. If the driver
 * supports separate individual keys (4 default keys + 1 individual),
 * addr can be used to determine whether the key is default or
 * individual. If only 4 keys are supported, the default key with key
 * index 0 is used as the individual key. STA must be configured to use
 * it as the default Tx key (set_tx is set) and accept Rx for all the
 * key indexes. In most cases, WPA uses only key indexes 1 and 2 for
 * broadcast keys, so key index 0 is available for this kind of
 * configuration.
 *
 * Please note that TKIP keys include separate TX and RX MIC keys and
 * some drivers may expect them in different order than wpa_supplicant
 * is using. If the TX/RX keys are swapped, all TKIP encrypted packets
 * will trigger Michael MIC errors. This can be fixed by changing the
 * order of MIC keys by swapping te bytes 16..23 and 24..31 of the key
 * in driver_*.c set_key() implementation, see driver_ndis.c for an
 * example on how this can be done.
 */
int cyw43438_set_key(const char *ifname, void *priv, enum wpa_alg alg, const u8 *addr, int key_idx, int set_tx, const u8 *seq, size_t seq_len, const u8 *key, size_t key_len)
{
	UNUSED_PARAMETER(ifname);
	UNUSED_PARAMETER(priv);
	UNUSED_PARAMETER(addr);
	UNUSED_PARAMETER(key_idx);
	UNUSED_PARAMETER(set_tx);
	UNUSED_PARAMETER(seq);
	UNUSED_PARAMETER(seq_len);

	return -1;
}

/**
 * get_ssid - Get the current SSID
 * @priv: private driver interface data
 * @ssid: buffer for SSID (at least 32 bytes)
 *
 * Returns: Length of the SSID on success, -1 on failure
 *
 * Query kernel driver for the current SSID and copy it to ssid.
 * Returning zero is recommended if the STA is not associated.
 *
 * Note: SSID is an array of octets, i.e., it is not nul terminated and
 * can, at least in theory, contain control characters (including nul)
 * and as such, should be processed as binary data, not a printable
 * string.
 */
int cyw43438_get_ssid(void *priv, u8 *ssid)
{
	UNUSED_PARAMETER(priv);

	if(__current_ssid.length < 32) {
		memcpy(ssid, __current_ssid.value, __current_ssid.length);
		ssid[__current_ssid.length] = 0;
	}

	return __current_ssid.length;
}

/**
 * hapd_send_eapol - Send an EAPOL packet (AP only)
 * @priv: private driver interface data
 * @addr: Destination MAC address
 * @data: EAPOL packet starting with IEEE 802.1X header
 * @data_len: Length of the EAPOL packet in octets
 * @encrypt: Whether the frame should be encrypted
 * @own_addr: Source MAC address
 * @flags: WPA_STA_* flags for the destination station
 *
 * Returns: 0 on success, -1 on failure
 */
int cyw43438_hapd_send_eapol(void *priv, const u8 *addr, const u8 *data, size_t data_len, int encrypt, const u8 *own_addr, u32 flags)
{
#ifndef ETH_P_PAE
#define ETH_P_PAE 0x888E
#endif

	UNUSED_PARAMETER(priv);
	UNUSED_PARAMETER(addr);
	UNUSED_PARAMETER(data);
	UNUSED_PARAMETER(data_len);
	UNUSED_PARAMETER(encrypt);
	UNUSED_PARAMETER(own_addr);
	UNUSED_PARAMETER(flags);

	eth_send_eapol(own_addr, addr, data, data_len, ETH_P_PAE);

	return -1;
}

/**
 * get_seqnum - Fetch the current TSC/packet number (AP only)
 * @ifname: The interface name (main or virtual)
 * @priv: Private driver interface data
 * @addr: MAC address of the station or %NULL for group keys
 * @idx: Key index
 * @seq: Buffer for returning the latest used TSC/packet number
 * Returns: 0 on success, -1 on failure
 *
 * This function is used to fetch the last used TSC/packet number for
 * a TKIP, CCMP, GCMP, or BIP/IGTK key. It is mainly used with group
 * keys, so there is no strict requirement on implementing support for
 * unicast keys (i.e., addr != %NULL).
 */
int cyw43438_seqnum(const char *ifname, void *priv, const u8 *addr, int idx, u8 *seq)
{
	UNUSED_PARAMETER(ifname);
	UNUSED_PARAMETER(priv);
	UNUSED_PARAMETER(addr);
	UNUSED_PARAMETER(idx);
	UNUSED_PARAMETER(seq);

	return -1;
}

/**
 * set_tx_power - Notify driver of TX power levels
 * @priv: Private driver interface data
 * @band: The selected power level
 * Returns 0 on success, -1 on failure
 */
int cyw43438_set_tx_power(void *priv, int power)
{
	wwd_result_t ret;

	UNUSED_PARAMETER(priv);

	ret = wwd_wifi_set_tx_power((uint8_t)power);
	if (ret == WWD_SUCCESS)
		return 0;

	return -1;
}

/**
 * get_tx_power - Driver TX power levels
 * @priv: Private driver interface data
 * @band: The selected power level
 * Returns 0 on success, -1 on failure
 */
int cyw43438_get_tx_power(void *priv)
{
	wwd_result_t ret;
	uint8_t dbm;

	UNUSED_PARAMETER(priv);

	ret = wwd_wifi_get_tx_power(&dbm);
	if (ret == WWD_SUCCESS)
		return 0;

	return -1;
}

/**
 * set_panic - Force panic the wlan driver.
 * @priv: Private driver interface data
 * Returns: 0 on success, -1 on failure
 *
 * This function used to force panic in WLAN driver.
 */
int cyw43438_set_panic(void *priv)
{
	UNUSED_PARAMETER(priv);

	return -1;
}

int cyw43438_del_station(void *priv, const u8 *addr, int reason)
{
	return -1;
}

#ifdef	SUPPORT_DUMP_BYTES
static void wwd_dump_bytes(void *data, uint32_t bytes)
{
	uint32_t i;
	uint8_t *buf;

	buf = (uint8_t *)data;
	for (i = 0; i < bytes; i++) {
		if ((i % 16) == 8) printf(" ");
		else if(( i% 16) == 0) printf("\n");
		printf("%02X ", buf[i]);
	}
	printf("\n");
}
#endif

static int32_t __get_last_scan_result_index(void)
{
	if (__bss_list != NULL)
		return __bss_list->current_index;

	return -1;
}

static void __set_last_scan_result_index_increase(void)
{
/* TODO
* The fact is, the ap_result_buff_write_pos value should not reach MAX_SCAN_RESULT_BUFF.
* Is the MAX_SCAN_RESULT_BUFF value large enough for practical use?
*/
	if (__bss_list != NULL)
		__bss_list->current_index++;
}

static void __set_last_scan_result_index_zero(void)
{
	if (__bss_list != NULL)
		__bss_list->current_index = 0;
}
