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
#include "driver.h"

#ifdef CONFIG_BRCM_WLAN
extern void *cyw43438_init2(void *ctx, const char *ifname, void *global_priv);
extern void cyw43438_deinit(void *priv);
extern const u8 *cyw43438_get_mac_addr(void *priv);
extern int cyw43438_get_capa(void *priv, struct wpa_driver_capa *capa);
extern int cyw43438_scan2(void *priv, struct wpa_driver_scan_params *params);
extern struct wpa_scan_results *cyw43438_get_scan_results2(void *priv);
extern int cyw43438_associate(void *priv, struct wpa_driver_associate_params *params);
extern int cyw43438_deauthenticate(void *priv, const u8 *addr, int reason_code);
extern int cyw43438_set_ap(void *priv, struct wpa_driver_ap_params *settings);
extern int cyw43438_stop_ap(void *priv);
extern int cyw43438_del_station(void *priv, const u8 *addr, int reason);
extern ssize_t cyw43438_set_country(void *priv, const char *country_code);
extern ssize_t cyw43438_get_country(void *priv, char *country_code);
extern int cyw43438_set_rts(void *priv, int rts);
extern int cyw43438_signal_poll(void *priv, struct wpa_signal_info *si);
extern int cyw43438_set_frag(void *priv, int frag_threshold);
extern int cyw43438_get_bssid(void *priv, u8 *bssid);
extern struct hostapd_hw_modes *cyw43438_get_hw_feature_data(void *priv, u16 *num_modes, u16 *flags);
extern int cyw43438_hapd_send_eapol(void *priv, const u8 *addr, const u8 *data, size_t data_len, int encrypt, const u8 *own_addr, u32 flags);
extern int cyw43438_set_key(const char *ifname, void *priv, enum wpa_alg alg, const u8 *mac_addr, int key_idx, int set_tx, const u8 *seq, size_t seq_len, const u8 *key, size_t key_len);
extern int cyw43438_seqnum(const char *ifname, void *priv, const u8 *addr, int idx, u8 *seq);
extern int cyw43438_set_tx_power(void *priv, int dbm);
extern int cyw43438_get_tx_power(void *priv);
extern int cyw43438_set_panic(void *priv);
//extern struct wireless_dev *cyw43438_add_virtual_intf(struct wiphy *wiphy, const char *name, enum nl80211_iftype type, u32 *flags, struct vif_params *params);
extern int cyw43438_get_ssid(void *priv, u8 *ssid);
extern int cyw43438_sta_remove(void *priv, const u8 *addr);
extern int cyw43438_sta_deauth(void *priv, const u8 *own_addr, const u8 *addr, int reason);
extern void cyw43438_resume(void *priv);
extern void cyw43438_suspend(void *priv);
extern const char *cyw43438_get_ifname(void *priv);
extern int cyw43438_send_ether(void *priv, const u8 *dst, const u8 *src, u16 proto, const u8 *data, size_t data_len);

const struct wpa_driver_ops wpa_driver_cyw4348_ops = {
	.name = "cyw43438",
	.desc = "Cypress CYW43438 Driver",
	.init2 = cyw43438_init2,
	.deinit = cyw43438_deinit,
	.get_mac_addr = cyw43438_get_mac_addr,
	.get_capa = cyw43438_get_capa,
	.scan2 = cyw43438_scan2,
	.get_scan_results2 = cyw43438_get_scan_results2,
	.associate = cyw43438_associate,
	.deauthenticate = cyw43438_deauthenticate,
	.set_ap = cyw43438_set_ap,
	.stop_ap = cyw43438_stop_ap,
	.sta_remove = cyw43438_sta_remove,
	.sta_deauth = cyw43438_sta_deauth,
	.set_frag = cyw43438_set_frag,
	.set_rts = cyw43438_set_rts,
	.signal_poll = cyw43438_signal_poll,
	.get_country = cyw43438_get_country,
	.set_country = cyw43438_set_country,
	.get_bssid = cyw43438_get_bssid,
	.get_hw_feature_data = cyw43438_get_hw_feature_data,
	.set_key = cyw43438_set_key,
	.get_ssid = cyw43438_get_ssid,
	.hapd_send_eapol = cyw43438_hapd_send_eapol,
	.get_seqnum = cyw43438_seqnum,
	.set_tx_power = cyw43438_set_tx_power,
	.get_tx_power = cyw43438_get_tx_power,
	.set_panic = cyw43438_set_panic,
	.resume = cyw43438_resume,
	.suspend = cyw43438_suspend,
	.get_ifname = cyw43438_get_ifname,
	.send_ether = cyw43438_send_ether,
/*  .if_add = cyw43438_add_virtual_intf */
};
#else /* CONFIG_BRCM_WLAN */
int cyw43438_sta_remove(void *priv, const u8 *addr)
{
	return 0;
}

int cyw43438_sta_deauth(void *priv, const u8 *own_addr, const u8 *addr, int reason)
{
	return 0;
}

const struct wpa_driver_ops wpa_driver_cyw4348_ops = {
	.name = "cyw4348",
	.desc = "Cypress CYW43438 Driver",
};
#endif /* CONFIG_BRCM_WLAN */
