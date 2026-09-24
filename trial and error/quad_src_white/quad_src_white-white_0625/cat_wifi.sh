#!/bin/bash
set -e
WLAN_IF="wlan0"
BOARD_IP="192.168.3.104/24"
# 强制网关为主机，所有公网流量必须经过主机转发
GATEWAY="192.168.3.100"
# DNS和主机保持一致，避免DNS劫持、解析环境不一致
DNS1="223.5.5.5"
DNS2="114.114.114.114"

# 清理旧WiFi连接
nmcli connection delete board_wifi_share 2>/dev/null || true

# 填写你当前WiFi名称、密码
WIFI_SSID="TP-LINK_AFBA"
WIFI_PWD="hitcrt88"

# 配置静态IP、唯一默认网关（强制所有外网走主机）
nmcli connection add \
type wifi con-name board_wifi_share ifname ${WLAN_IF} \
wifi.ssid "${WIFI_SSID}" \
wifi-sec.key-mgmt wpa-psk wifi-sec.psk "${WIFI_PWD}" \
ipv4.method manual \
ipv4.addresses ${BOARD_IP} \
ipv4.gateway ${GATEWAY} \
ipv4.dns "${DNS1},${DNS2}"

nmcli connection up board_wifi_share

# 彻底清理全局代理，避免API请求异常
unset HTTP_PROXY HTTPS_PROXY ALL_PROXY http_proxy https_proxy all_proxy

# 删除其他网卡默认路由，防止流量从eth1等网卡出走（关键！避免绕开主机）
sudo ip route del default dev eth1 2>/dev/null

echo "===== 板卡网络配置完成，所有流量强制经过主机出口 ====="
ip route | grep default
