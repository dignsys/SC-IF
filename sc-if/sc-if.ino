/*
 * SC-IF Main
 * Author : DIGNSYS Inc.
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Ethernet_Generic.h>
#include <Preferences.h>

#define VERSION_SCIF_FW  "20240125"

//#define OP_CHARGER_SIDE
//#define OP_SERVER_TEST_PACKET

#define PIN_LED             20
#define PIN_W5500_RST       40
#define PIN_ETH_CS          39
#define PIN_ETH_INT         38
#define PIN_ETH_MISO        37
#define PIN_ETH_SCLK        36
#define PIN_ETH_MOSI        35
#define PIN_I2C0_SDA        15
#define PIN_I2C0_SCL        16
#define PIN_I2C1_SDA        42
#define PIN_I2C1_SCL        41
#define PIN_UART_RX0        18
#define PIN_UART_TX0        17
#define PIN_UART_RX1        48
#define PIN_UART_TX1        47
#define PIN_BOOT            0

#define PERIOD_MAIN_LOOP        500 // 500 msec
#define PERIOD_CHARGER_MONITOR  60  // 60 = 30 sec, 120 = 1 min, 15*60*2 = 15 min
#define PERIOD_SERVER_CTRL      120

#define SZ_SERVER_IP_BUF        32

uint8_t nc_mac[] = {
  0x00, 0x08, 0xDC, 0x00, 0x00, 0x00
};
IPAddress nc_ip(192, 168, 1, 35);
IPAddress nc_dns(192, 168, 1, 1);
IPAddress nc_gateway(192, 168, 1, 1);
IPAddress nc_subnet(255, 255, 255, 0);
IPAddress nc_server(192, 168, 1, 2);

SPIClass* hspi;
DhcpClass* dhcp = new DhcpClass();

unsigned long sys_start_millis = 0;
uint8_t g_led_status = 0;

#define RS485_Serial  Serial1

uint16_t CRC16_TBL[256] = {

  0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
  0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
  0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
  0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
  0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
  0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
  0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
  0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
  0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
  0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
  0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
  0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
  0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
  0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
  0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
  0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
  0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
  0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
  0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
  0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
  0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
  0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
  0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
  0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
  0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
  0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
  0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
  0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
  0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
  0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
  0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
  0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040

};

#define ETH_BUF_MAX 96
#define RS_BUF_MAX  96
uint8_t m_buf[RS_BUF_MAX] = {0,};
uint8_t c_tx_buf[RS_BUF_MAX] = {0,};
uint8_t c_rx_buf[RS_BUF_MAX] = {0,};

struct sc_pkt_hdr_hash_str {
  
  uint8_t KEY[6];
  uint8_t SPARE[2];
};

struct sc_pkt_hdr_str {

  uint8_t CMD;
  uint8_t TYPE;
  uint8_t CHGID;
  struct sc_pkt_hdr_hash_str HASH;
  uint8_t SPARE[5];
};

struct sc_m_pkt_str {

  uint8_t STX[3];
  struct sc_pkt_hdr_str HEAD;
  uint8_t DATA[16];
  uint8_t ETX[3];
  uint8_t CRC16[2];
};

struct sc_c_pkt_str {

  uint8_t STX[3];
  struct sc_pkt_hdr_str HEAD;
  uint8_t DATA[8];
  uint8_t ETX[3];
  uint8_t CRC16[2];
};

struct sc_c_pkt_str sc_c_svr_pkt;
struct sc_c_pkt_str sc_c_chg_pkt;
struct sc_m_pkt_str sc_m_chg_pkt;

enum {
  e_RS_RCV_NONE,
  e_RS_RCV_START,
  e_RS_RCV_PROG,
  e_RS_RCV_END,
  e_RS_RCV_DONE,
};

enum {
  e_RS_RCV_PKT_NONE,
  e_RS_RCV_PKT_CTRL,
  e_RS_RCV_PKT_MON,
};

enum {
  e_ETH_RCV_NONE,
  e_ETH_RCV_DONE,
};

int rs_rcv_status = e_RS_RCV_NONE;
int rs_rcv_packet = e_RS_RCV_PKT_NONE;
int eth_rcv_status = e_ETH_RCV_NONE;

int link_initialized = 0;
uint16_t gv_port;
uint8_t gv_server[SZ_SERVER_IP_BUF] = {0,};
EthernetClient client;

Preferences prefs;

typedef struct {
  uint16_t gv_port;
  char gv_server[SZ_SERVER_IP_BUF];
} settings_t;

settings_t gv_settings;

uint16_t get_crc16(uint16_t* pdata, uint16_t len);
void make_ctrl_svr_pkt(void);
void make_ctrl_chg_pkt(void);
void make_monitoring_pkt(void);
void send_ctrl_pkt(sc_c_pkt_str* pctrl);
void send_monitoring_pkt(sc_m_pkt_str* pmon);
int proc_rs_receive(uint8_t* pbuf, int len);
int check_stx(uint8_t* pbuf, int len);
int check_etx(uint8_t* pbuf, int len);
int check_crc16(uint8_t* pbuf, int len);
int proc_eth_receive(uint8_t* pbuf, int len);

void sub_test_a(void);    // LED Test
void sub_test_b(void);    // Button Test
void sub_test_n(void);    // W5500 Network Function Test
void sub_test_o(void);    // UART Function Test
void sub_test_r(void);    // RS485 SC Packet Communication Test
void sub_test_w(void);    // Settings
void sub_test_loop(void);

void setup() {

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, PIN_UART_RX0, PIN_UART_TX0);  // RS232
  Serial2.begin(115200, SERIAL_8N1, PIN_UART_RX1, PIN_UART_TX1);  // RS232 TTL

  pinMode(PIN_LED, OUTPUT);

  pinMode(PIN_W5500_RST, OUTPUT);
  digitalWrite(PIN_W5500_RST, LOW);
  delay(100);
  digitalWrite(PIN_W5500_RST, HIGH);
  delay(100);

  hspi = new SPIClass(HSPI);
  hspi->begin(PIN_ETH_SCLK, PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_CS);

  Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, 400000);
  Wire1.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, 400000);

  esp_read_mac(nc_mac, ESP_MAC_ETH);
  Serial.printf("ETH MAC: %02x, %02x, %02x, %02x, %02x, %02x\r\n", nc_mac[0], nc_mac[1], nc_mac[2], nc_mac[3], nc_mac[4], nc_mac[5]);

#ifndef OP_CHARGER_SIDE
  Ethernet.init(PIN_ETH_CS);

  // Ethernet Initialize
  hspi->setFrequency(40000000);
  pCUR_SPI = hspi;
  Ethernet.begin(nc_mac);
  Ethernet._pinRST = PIN_W5500_RST;
  Ethernet._pinCS = PIN_ETH_CS;
  Ethernet.setHostname("SC-IF_001");
  Ethernet.setRetransmissionCount(3);
  Ethernet.setRetransmissionTimeout(4000);

  Serial.print("getChip(): "); Serial.println(Ethernet.getChip(), DEC);
  Serial.print("localIP(): "); Serial.println(Ethernet.localIP());

  if(Ethernet.linkStatus() == LinkON) {
    Serial.printf("Link ON\r\n");
    link_initialized = 1;
  } else {
    Serial.printf("Link is %d\r\n", Ethernet.linkStatus());
  }
#endif

  uint8_t plen;
  uint8_t rdata[sizeof(uint16_t) + SZ_SERVER_IP_BUF];
  prefs.begin("settings");
  memset((void*) &gv_settings, 0x00, sizeof(gv_settings));
  if(prefs.isKey("settings")) {
    plen = prefs.getBytesLength("settings");
    if(plen){
      prefs.getBytes("settings", rdata, plen);
      memcpy((void*)&gv_settings, rdata, plen);
      Serial.printf("Settings Server IP: %s:%d\r\n", gv_settings.gv_server, gv_settings.gv_port);
    }
    if(strlen(gv_settings.gv_server)) {
      strcpy((char*) gv_server, gv_settings.gv_server);
    }
    if(gv_settings.gv_port) {
      gv_port = gv_settings.gv_port;
    }
  }

  if(!gv_port) {
    gv_port = 5720;
  }
  if(!isdigit(gv_server[0])){
    strcpy((char*) gv_server, "192.148.1.149");
  }
  Serial.printf("Server IP: %s:%d\r\n", gv_server, gv_port);

  nc_server.fromString((char*) gv_server);
  if(link_initialized) {
//    Serial.print("dnsServerIP(): "); Serial.println(Ethernet.dnsServerIP());
//    if(client.connect(gv_server, gv_port)) {
    if(client.connect(nc_server, gv_port)) {
      Serial.print("Connected to "); Serial.print(client.remoteIP());
      Serial.printf(":%d\r\n", client.remotePort());
    } else {
      Serial.printf("Fail to connect to server\r\n");
    }
  }

}

void loop() {

  Serial.println();
  Serial.println("SC-IF Main Loop");
  Serial.println("(C) 2024 Dignsys");
  Serial.printf("VERSION: %s\r\n\r\n", VERSION_SCIF_FW);

  char c;
  int wret;
  int elen, erlen;
  int rlen, rtlen, ridx, plen;
  int chg_cnt = 0;
  int svr_cnt = 0;
  uint8_t rt_buf[RS_BUF_MAX];
  uint8_t et_buf[ETH_BUF_MAX];
  unsigned long cur_millis = 0;
  unsigned long prev_millis = 0;

  sys_start_millis = millis();

  while(1){

    prev_millis = millis();

    if(Serial.available()) {
      c = Serial.read();
    }
    if(c == '#'){
      Serial.println("Go to Sub Test!!!");
      sub_test_loop();
      Serial.println("Return to Main Loop!!!");
      c = 0;
    }
#ifdef OP_CHARGER_SIDE
    if(!chg_cnt) {
      make_monitoring_pkt();
      send_monitoring_pkt(&sc_m_chg_pkt);
      Serial.printf("Send Monitoring Packet To Server Side @%ld\r\n", cur_millis);
    }
    if((++chg_cnt) > PERIOD_CHARGER_MONITOR) {
      chg_cnt = 0;
    }

    // Receive Charger Packet
    ridx = 0;
    rtlen = 0;
    while(RS485_Serial.available()) {
      rlen = RS485_Serial.available();
      if((ridx + rlen) < RS_BUF_MAX) {
        RS485_Serial.read(&rt_buf[ridx], rlen);
        rtlen += rlen;
      } else {
        Serial.printf("Receive Buffer Overrun: rtlen - %d, rlen - %d\r\n", rtlen, rlen);
      }
      delay(10);
    }
    if(rtlen) {
      plen = proc_rs_receive(rt_buf, rtlen);
    }

    if(rs_rcv_status == e_RS_RCV_DONE) {
      make_ctrl_chg_pkt();
      send_ctrl_pkt(&sc_c_chg_pkt);
      Serial.printf("Send Response Packet To Server Side @%ld\r\n", cur_millis);
      rs_rcv_status = e_RS_RCV_NONE;
    }
#else
#ifdef OP_SERVER_TEST_PACKET
    if(svr_cnt == 1) {
      make_ctrl_svr_pkt();
      send_ctrl_pkt(&sc_c_svr_pkt);
      Serial.printf("Send Control Packet To Charger Side @%ld\r\n", cur_millis);
    }
    if((++svr_cnt) > PERIOD_SERVER_CTRL) {
      svr_cnt = 0;
    }
#endif
    // Receive Charger Packet
    ridx = 0;
    rtlen = 0;
    while(RS485_Serial.available()) {
      rlen = RS485_Serial.available();
      if((ridx + rlen) < RS_BUF_MAX) {
        RS485_Serial.read(&rt_buf[ridx], rlen);
        rtlen += rlen;
      } else {
        Serial.printf("Receive Buffer Overrun: rtlen - %d, rlen - %d\r\n", rtlen, rlen);
        break;
      }
      // delay(10) can make overrun (rtlen - 0, rlen - 72)
      delay(1);
    }
    if(rtlen) {
      plen = proc_rs_receive(rt_buf, rtlen);
      if(plen != rtlen) {
        Serial.printf("Receive Packet is not fully processed: rtlen - %d, plen - %d\r\n", rtlen, plen);
      }
    }

    if(rs_rcv_status == e_RS_RCV_DONE) {
      if(client.connected()) {
        if(rs_rcv_packet == e_RS_RCV_PKT_MON) {
          client.clearWriteError();
          wret = client.write(m_buf, sizeof(sc_m_pkt_str));
          Serial.printf("Monitor Packet Write (%d): %d\r\n", client.getWriteError(), wret);
        } else if(rs_rcv_packet == e_RS_RCV_PKT_CTRL) {
          client.clearWriteError();
          wret = client.write(c_rx_buf, sizeof(sc_c_pkt_str));
          Serial.printf("Control Packet Write (%d): %d\r\n", client.getWriteError(), wret);
        }
      }
      rs_rcv_status = e_RS_RCV_NONE;
    }

    if(client.connected()) {
      if(elen = client.available()) {
        if(elen > ETH_BUF_MAX) {
          Serial.printf("Received Ethernet Data is too large: %d\r\n", elen);
          elen = ETH_BUF_MAX;
        }
        erlen = client.read(et_buf, elen);
        Serial.printf("Received Ethernet Data: %d (%d)\r\n", erlen, elen);
        plen = proc_eth_receive(et_buf, erlen);
        if(plen == sizeof(sc_c_pkt_str)) {
          send_ctrl_pkt(&sc_c_svr_pkt);
        }
        eth_rcv_status = e_ETH_RCV_NONE;
      }
    }
#endif

    // wait until the interval
    while(1){
      cur_millis = millis();
      if((cur_millis - prev_millis) < PERIOD_MAIN_LOOP){
        delay(10);
      } else {
        break;
      }
    }

    if(g_led_status) {
      g_led_status = 0;
    } else {
      g_led_status = 1;
    }
    digitalWrite(PIN_LED, g_led_status);

  }
  Serial.println("Main Loop Exit!");
}

void sub_test_loop(void) {

  Serial.println();
  Serial.println("SC-IF Sub-Test Loop");
  Serial.println("(C) 2024 Dignsys"); Serial.println();
  Serial.printf("VERSION: %s\r\n\r\n", VERSION_SCIF_FW);

  char c;
  while(c != 'x') {
    Serial.printf("Input Command: ");
    while(1){
      if(Serial.available()) {
        c = Serial.read();
        if(isalnum(c)){
          break;
        }
      }
      delay(100);
    }
    Serial.printf("%c", c);
    Serial.println();

    switch(c) {
      case 'a': 
        sub_test_a();
        break;
      case 'b': 
        sub_test_b();
        break;
      case 'n':
        sub_test_n();
        break;
      case 'o':
        sub_test_o();
        break;
      case 'r':
        sub_test_r();
        break;
      case 'w':
        sub_test_w();
        break;
      default:
        break;
    }
  }
  Serial.println("Sub-Test Loop Exit!");

}

uint16_t get_crc16(uint16_t* pdata, uint16_t len) {

  uint8_t ntemp;
  uint16_t crc_word = 0xffff;

  while (len--) {

    ntemp = *pdata++ ^ crc_word;
    crc_word >>= 8;
    crc_word ^= CRC16_TBL[ntemp];
  }

  return crc_word;

}

void make_ctrl_svr_pkt(void) {

  uint16_t crc_temp;

  memset(&sc_c_svr_pkt, 0x00, sizeof(sc_c_pkt_str));

  // STX
  sc_c_svr_pkt.STX[0] = '$';
  sc_c_svr_pkt.STX[1] = 'S';
  sc_c_svr_pkt.STX[2] = 'C';

  // HEAD CMD
  sc_c_svr_pkt.HEAD.CMD = 'C';
  // HEAD TYPE
  sc_c_svr_pkt.HEAD.TYPE = 'S';
  // HEAD CHGID
  sc_c_svr_pkt.HEAD.CHGID = 0x01;
  // HEAD HASH KEY
  sc_c_svr_pkt.HEAD.HASH.KEY[0] = nc_mac[0];
  sc_c_svr_pkt.HEAD.HASH.KEY[1] = nc_mac[1];
  sc_c_svr_pkt.HEAD.HASH.KEY[2] = nc_mac[2];
  sc_c_svr_pkt.HEAD.HASH.KEY[3] = nc_mac[3];
  sc_c_svr_pkt.HEAD.HASH.KEY[4] = nc_mac[4];
  sc_c_svr_pkt.HEAD.HASH.KEY[5] = nc_mac[5];
  // HEAD HASH SPARE
  sc_c_svr_pkt.HEAD.HASH.SPARE[0] = 0x00;
  sc_c_svr_pkt.HEAD.HASH.SPARE[1] = 0x00;
  // HEAD SPARE
  sc_c_svr_pkt.HEAD.SPARE[0] = 0x00;
  sc_c_svr_pkt.HEAD.SPARE[1] = 0x00;
  sc_c_svr_pkt.HEAD.SPARE[2] = 0x00;
  sc_c_svr_pkt.HEAD.SPARE[3] = 0x00;
  sc_c_svr_pkt.HEAD.SPARE[4] = 0x00;

  // DATA
  sc_c_svr_pkt.DATA[0] = 10;  // DIM (1~10) x10 -> 10%~100%
  sc_c_svr_pkt.DATA[1] = 0xff;  // SW8 ~ SW1 (0:OFF, 1: ON)
  sc_c_svr_pkt.DATA[2] = 0x03;  // SW10 ~ SW9
  sc_c_svr_pkt.DATA[3] = 0x00;  // SPARE
  sc_c_svr_pkt.DATA[4] = 0x00;  // SPARE
  sc_c_svr_pkt.DATA[5] = 0x00;  // SPARE
  sc_c_svr_pkt.DATA[6] = 0x00;  // SPARE
  sc_c_svr_pkt.DATA[7] = 0x00;  // SPARE

  // ETX
  sc_c_svr_pkt.ETX[0] = 'S';
  sc_c_svr_pkt.ETX[1] = 'C';
  sc_c_svr_pkt.ETX[2] = '$';

  // CRC16
  crc_temp = get_crc16((uint16_t*)&sc_c_svr_pkt, (sizeof(sc_c_pkt_str) - 2)/2);

  sc_c_svr_pkt.CRC16[0] = (crc_temp & 0x00ff);
  sc_c_svr_pkt.CRC16[1] = ((crc_temp>>8) & 0x00ff);

}

void make_ctrl_chg_pkt(void) {
  
  uint16_t crc_temp;

  memset(&sc_c_chg_pkt, 0x00, sizeof(sc_c_pkt_str));

  // STX
  sc_c_chg_pkt.STX[0] = '$';
  sc_c_chg_pkt.STX[1] = 'S';
  sc_c_chg_pkt.STX[2] = 'C';

  // HEAD CMD
  sc_c_chg_pkt.HEAD.CMD = 'S';
  // HEAD TYPE
  sc_c_chg_pkt.HEAD.TYPE = 'S';
  // HEAD CHGID
  sc_c_chg_pkt.HEAD.CHGID = 0x01;
  // HEAD HASH KEY
  sc_c_chg_pkt.HEAD.HASH.KEY[0] = nc_mac[0];
  sc_c_chg_pkt.HEAD.HASH.KEY[1] = nc_mac[1];
  sc_c_chg_pkt.HEAD.HASH.KEY[2] = nc_mac[2];
  sc_c_chg_pkt.HEAD.HASH.KEY[3] = nc_mac[3];
  sc_c_chg_pkt.HEAD.HASH.KEY[4] = nc_mac[4];
  sc_c_chg_pkt.HEAD.HASH.KEY[5] = nc_mac[5];
  // HEAD HASH SPARE
  sc_c_chg_pkt.HEAD.HASH.SPARE[0] = 0x00;
  sc_c_chg_pkt.HEAD.HASH.SPARE[1] = 0x00;
  // HEAD SPARE
  sc_c_chg_pkt.HEAD.SPARE[0] = 0x00;
  sc_c_chg_pkt.HEAD.SPARE[1] = 0x00;
  sc_c_chg_pkt.HEAD.SPARE[2] = 0x00;
  sc_c_chg_pkt.HEAD.SPARE[3] = 0x00;
  sc_c_chg_pkt.HEAD.SPARE[4] = 0x00;

  // DATA
  sc_c_chg_pkt.DATA[0] = 1;  // RES : 0 - NG, 1 - OK
  sc_c_chg_pkt.DATA[1] = 0xff;  // RES8 ~ RES1 : 0 - NG, 1 - OK
  sc_c_chg_pkt.DATA[2] = 0x03;  // RES10 ~ RES9 : 0 - NG, 1 - OK
  sc_c_chg_pkt.DATA[3] = 0x00;  // Relay Board, RSTA : 0 - OFF, 1 - ON
  sc_c_chg_pkt.DATA[4] = 0x00;  // SPARE
  sc_c_chg_pkt.DATA[5] = 0x00;  // SPARE
  sc_c_chg_pkt.DATA[6] = 0x00;  // SPARE
  sc_c_chg_pkt.DATA[7] = 0x00;  // SPARE

  // ETX
  sc_c_chg_pkt.ETX[0] = 'S';
  sc_c_chg_pkt.ETX[1] = 'C';
  sc_c_chg_pkt.ETX[2] = '$';

  // CRC16
  crc_temp = get_crc16((uint16_t*)&sc_c_chg_pkt, (sizeof(sc_c_pkt_str) - 2)/2);

  sc_c_chg_pkt.CRC16[0] = (crc_temp & 0x00ff);
  sc_c_chg_pkt.CRC16[1] = ((crc_temp>>8) & 0x00ff);

}

void make_monitoring_pkt(void) {

  uint16_t crc_temp;
  uint16_t w_temp;

  memset(&sc_m_chg_pkt, 0x00, sizeof(sc_m_pkt_str));

  // STX
  sc_m_chg_pkt.STX[0] = '$';
  sc_m_chg_pkt.STX[1] = 'S';
  sc_m_chg_pkt.STX[2] = 'C';

  // HEAD CMD
  sc_m_chg_pkt.HEAD.CMD = 'M';
  // HEAD TYPE
  sc_m_chg_pkt.HEAD.TYPE = 'C';
  // HEAD CHGID
  sc_m_chg_pkt.HEAD.CHGID = 0x01;
  // HEAD HASH KEY
  sc_m_chg_pkt.HEAD.HASH.KEY[0] = nc_mac[0];
  sc_m_chg_pkt.HEAD.HASH.KEY[1] = nc_mac[1];
  sc_m_chg_pkt.HEAD.HASH.KEY[2] = nc_mac[2];
  sc_m_chg_pkt.HEAD.HASH.KEY[3] = nc_mac[3];
  sc_m_chg_pkt.HEAD.HASH.KEY[4] = nc_mac[4];
  sc_m_chg_pkt.HEAD.HASH.KEY[5] = nc_mac[5];
  // HEAD HASH SPARE
  sc_m_chg_pkt.HEAD.HASH.SPARE[0] = 0x00;
  sc_m_chg_pkt.HEAD.HASH.SPARE[1] = 0x00;
  // HEAD SPARE
  sc_m_chg_pkt.HEAD.SPARE[0] = 0x00;
  sc_m_chg_pkt.HEAD.SPARE[1] = 0x00;
  sc_m_chg_pkt.HEAD.SPARE[2] = 0x00;
  sc_m_chg_pkt.HEAD.SPARE[3] = 0x00;
  sc_m_chg_pkt.HEAD.SPARE[4] = 0x00;

  // DATA
  w_temp = 1000;
  sc_m_chg_pkt.DATA[0] = (w_temp & 0x00ff);  // PV Voltage, PVV 0~1000 [V]
  sc_m_chg_pkt.DATA[1] = ((w_temp>>8) & 0x00ff);
  w_temp = 100;
  sc_m_chg_pkt.DATA[2] = (w_temp & 0x00ff);  // PV Current, PVC 0~100 [A]
  sc_m_chg_pkt.DATA[3] = ((w_temp>>8) & 0x00ff);
  w_temp = 1000;
  sc_m_chg_pkt.DATA[4] = (w_temp & 0x00ff);  // BATT Voltage, BAV 0~1000 [V]
  sc_m_chg_pkt.DATA[5] = ((w_temp>>8) & 0x00ff);
  w_temp = 100;
  sc_m_chg_pkt.DATA[6] = (w_temp & 0x00ff);  // BATT Current, BAC 0~100 [A]
  sc_m_chg_pkt.DATA[7] = ((w_temp>>8) & 0x00ff);
  w_temp = 1000;
  sc_m_chg_pkt.DATA[8] = (w_temp & 0x00ff);  // Output Voltage, OUV 0~1000 [V]
  sc_m_chg_pkt.DATA[9] = ((w_temp>>8) & 0x00ff);
  w_temp = 100;
  sc_m_chg_pkt.DATA[10] = (w_temp & 0x00ff);  // Current Output, OUC 0~100 [A]
  sc_m_chg_pkt.DATA[11] = ((w_temp>>8) & 0x00ff);
  sc_m_chg_pkt.DATA[12] = 0xff;   // STA8 ~ STA1 : 0 - OFF, 1 - ON
  sc_m_chg_pkt.DATA[13] = 0x03;   // STA10 ~ STA9 : 0 - OFF, 1 - ON
  sc_m_chg_pkt.DATA[14] = 0xff;   // Relay Board, RSTA : 0 - OFF, 1 - ON
  sc_m_chg_pkt.DATA[15] = 0x00;

  // ETX
  sc_m_chg_pkt.ETX[0] = 'S';
  sc_m_chg_pkt.ETX[1] = 'C';
  sc_m_chg_pkt.ETX[2] = '$';

  // CRC16
  crc_temp = get_crc16((uint16_t*)&sc_m_chg_pkt, (sizeof(sc_m_pkt_str) - 2)/2);

  sc_m_chg_pkt.CRC16[0] = (crc_temp & 0x00ff);
  sc_m_chg_pkt.CRC16[1] = ((crc_temp>>8) & 0x00ff);

}

void send_ctrl_pkt(sc_c_pkt_str* pctrl) {

  int len;

  memcpy(c_tx_buf, pctrl, sizeof(sc_c_pkt_str));

  len = sizeof(sc_c_pkt_str);

  for(int i = 0; i < len; i++) {
    RS485_Serial.write(c_tx_buf[i]);
  }
#if 0
  Serial.println("Control Data Start");
  for(int i = 0; i < len; i++) {
    Serial.printf("%02d: %02x\r\n", i, c_tx_buf[i]);
  }
  Serial.println("Control Data End");
#endif
}

void send_monitoring_pkt(sc_m_pkt_str* pmon) {

  int len;

  memcpy(m_buf, pmon, sizeof(sc_m_pkt_str));

  len = sizeof(sc_m_pkt_str);

  for(int i = 0; i < len; i++) {
    RS485_Serial.write(m_buf[i]);
  }

}

int proc_rs_receive(uint8_t* pbuf, int len) {

  int ret = 0;
  int sidx, eidx;
  int crc_chk = 0;

  if(rs_rcv_status == e_RS_RCV_NONE) {

    sidx = check_stx(pbuf, len);
    if(sidx < 0) {
      Serial.println("proc_rs_receive: sidx is not detected");
      return ret;
    }
    eidx = check_etx(pbuf, len);
    if(eidx < 0) {
      Serial.println("proc_rs_receive: eidx is not detected");
      ret = sidx;
      return ret;
    }

    Serial.printf("Received RS Packet[%c]@%ld: sidx - %d, eidx - %d, len - %d\r\n", *(pbuf+sidx+3), millis(), sidx, eidx, len);

    if((eidx + 2 + 2) <= (len - 1)) {
      crc_chk = check_crc16(pbuf+sidx, (eidx + 2 + 2) - sidx + 1);
    }

    if(crc_chk) {

      if(*(pbuf+sidx+3) == 'M') {
        memcpy(m_buf, pbuf+sidx, (eidx + 2 + 2) - sidx + 1);
        rs_rcv_packet = e_RS_RCV_PKT_MON;
      } else {
        memcpy(c_rx_buf, pbuf+sidx, (eidx + 2 + 2) - sidx + 1);
        rs_rcv_packet = e_RS_RCV_PKT_CTRL;
      }
      ret = (eidx + 2 + 2) + 1;
      rs_rcv_status = e_RS_RCV_DONE;

    } else {
      Serial.println("proc_rs_receive: checksum is not correct");
    }
  }
  return ret;
}

int proc_eth_receive(uint8_t* pbuf, int len) {

  int ret = 0;
  int sidx, eidx;
  int crc_chk = 0;

  if(eth_rcv_status == e_ETH_RCV_NONE) {

    sidx = check_stx(pbuf, len);
    if(sidx < 0) {
      Serial.println("proc_eth_receive: sidx is not detected");
      return ret;
    }
    eidx = check_etx(pbuf, len);
    if(eidx < 0) {
      Serial.println("proc_eth_receive: eidx is not detected");
      ret = sidx;
      return ret;
    }

    Serial.printf("Received ETH Packet[%c]@%ld: sidx - %d, eidx - %d, len - %d\r\n", *(pbuf+sidx+3), millis(), sidx, eidx, len);

    if((eidx + 2 + 2) <= (len - 1)) {
      crc_chk = check_crc16(pbuf+sidx, (eidx + 2 + 2) - sidx + 1);
    }

    if(crc_chk) {

      if(*(pbuf+sidx+3) == 'C') {
        memcpy(&sc_c_svr_pkt, pbuf+sidx, (eidx + 2 + 2) - sidx + 1);
      } else {
        Serial.printf("Unknown Command Packet received: %02x\r\n", *(pbuf+sidx+3));
      }
      ret = (eidx + 2 + 2) + 1;
      eth_rcv_status = e_ETH_RCV_DONE;

    } else {
      Serial.println("proc_eth_receive: checksum is not correct");
    }
  }
  return ret;
}

int check_stx(uint8_t* pbuf, int len) {

  int ret = -1;

  for(int i = 0; i < len; i++) {
    if(pbuf[i] == '$') {
      if((i+2) < len) {
        if((pbuf[i+1] == 'S') && (pbuf[i+2] == 'C')){
          ret = i;
          break;
        }
      }
    }
  }
  return ret;
}

int check_etx(uint8_t* pbuf, int len) {

  int ret = -1;

  for(int i = 0; i < len; i++) {
    if(pbuf[i] == 'S') {
      if((i+2) < len) {
        if((pbuf[i+1] == 'C') && (pbuf[i+2] == '$')){
          ret = i;
          break;
        }
      }
    }
  }
  return ret;
}

int check_crc16(uint8_t* pbuf, int len) {

  int ret = 0;
  uint16_t crc_temp, crc_rcv;

  crc_temp = get_crc16((uint16_t*) pbuf, (len - 2)/2);
  crc_rcv = pbuf[len-2] + pbuf[len-1]*0x100;

#if 0
  for(int i = 0; i < len; i++){
    Serial.printf("pbuf[%d]: %02x\r\n", i, pbuf[i]);
  }

  Serial.printf("len: %d, crc_temp: %04x, crc_rcv: %04x\r\n", len, crc_temp, crc_rcv);
#endif

  if(crc_temp == crc_rcv){
    ret = 1;
  } else {
    Serial.printf("checksum is not correct: crc_calc: %04x, crc_rcv: %04x\r\n", crc_temp, crc_rcv);
  }
  return ret;
}

void sub_test_a(void) {

  Serial.println("Sub-test A - LED");

  char c = 0;
  uint8_t led_status = 0;

  Serial.println("Press q to quit: ");

  while(1) {
    if(Serial.available()) {
      if(isalnum(c)) Serial.println(c);
      Serial.println(c);
    }
    if(c == 'q'){
      Serial.println("Quit loop");
      break;
    }
    if(led_status) {
      led_status = 0;
      Serial.println("LED Off");
    } else {
      led_status = 1;
      Serial.println("LED On");
    }
    digitalWrite(PIN_LED, led_status);
    delay(1000);
  }
}

void sub_test_b(void) {

  Serial.println("Sub-test B - Button");

  char c = 0;
  uint8_t button_status = 0;

  Serial.println("Press q to quit: ");

  while(1) {
    if(Serial.available()) {
      c = Serial.read();
      Serial.println(c);
    }
    if(c == 'q'){
      Serial.println("Quit loop");
      break;
    }
    button_status = digitalRead(PIN_BOOT);
    Serial.printf("Button: %d\r\n", button_status);
    delay(1000);
  }
}

void sub_test_n(void) {

  uint8_t data;
  int numBytes;
  char c;
  Serial.println("Sub-test N - W5500");

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)){
        break;
      }
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {  // 
    // W5500
    pinMode(PIN_ETH_CS, OUTPUT);
    pinMode(PIN_ETH_SCLK, OUTPUT);
    pinMode(PIN_ETH_MISO, INPUT);
    pinMode(PIN_ETH_MOSI, OUTPUT);
    digitalWrite(PIN_ETH_CS, HIGH);
    digitalWrite(PIN_ETH_SCLK, LOW);

    Serial.print("pCUR_SPI->pinSS(): "); Serial.println(pCUR_SPI->pinSS(), DEC);

    SPI.begin(PIN_ETH_SCLK, PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_CS);
    pinMode(SPI.pinSS(), OUTPUT);

    Ethernet.init(PIN_ETH_CS);

    Serial.print("pCUR_SPI->pinSS(): "); Serial.println(pCUR_SPI->pinSS(), DEC);

    Serial.print("w5500: "); Serial.println(w5500, DEC);
    Serial.print("getChip(): "); Serial.println(Ethernet.getChip(), DEC);

  } else if(c == '1') {  // 

    pinMode(SS, OUTPUT);
    pinMode(SCK, OUTPUT);
    pinMode(MISO, INPUT);
    pinMode(MOSI, OUTPUT);
    digitalWrite(SS, HIGH);
    digitalWrite(SCK, LOW);

    SPI.begin(SCK, MISO, MOSI, SS);
    //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));  
    //SPI.beginTransaction(SPISettings(80000000, MSBFIRST, SPI_MODE0));  
    digitalWrite(SS, LOW);
    for(int i=0; i < 20; i++) {
      SPI.transfer(0x55);
    }
    digitalWrite(SS, HIGH);

  } else if(c == '2') {  // 

    pinMode(SS, OUTPUT);
    digitalWrite(SS, HIGH);

    SPIClass* vspi = new SPIClass(HSPI);

    vspi->begin(PIN_ETH_SCLK, PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_CS);
    vspi->beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
    digitalWrite(SS, LOW);
    for(int i=0; i < 20; i++) {
      vspi->transfer(0x55);
    }
    digitalWrite(SS, HIGH);
    vspi->endTransaction();
    delete vspi;

  } else if(c == '3') {  // 
    Ethernet.init(PIN_ETH_CS);

    hspi->setFrequency(40000000);
    pCUR_SPI = hspi;
    //Ethernet.begin(nc_mac, nc_ip, nc_dns, nc_gateway, nc_subnet);
    Ethernet.begin(nc_mac);
    hspi->setFrequency(40000000);
    Ethernet._pinRST = PIN_W5500_RST;
    Ethernet._pinCS = PIN_ETH_CS;
    Ethernet.setHostname("SC-IF_001");
    Ethernet.setRetransmissionCount(3);
    Ethernet.setRetransmissionTimeout(4000);

    Serial.print("getChip(): "); Serial.println(Ethernet.getChip(), DEC);
    Serial.print("localIP(): "); Serial.println(Ethernet.localIP());

  } else if(c == '4') {
    Serial.print("linkStatus: "); Serial.println(Ethernet.linkStatus(), DEC); //LINK_ON, LINK_OFF
    Serial.print("PhyState: "); Serial.println(Ethernet.phyState(), HEX);
    Serial.print("HardwareStatus: "); Serial.println(Ethernet.hardwareStatus());
    Serial.print("speed: "); Serial.println(Ethernet.speed(), DEC);
    Serial.print("duplex: "); Serial.println(Ethernet.duplex(), DEC);
    Serial.print("localIP(): "); Serial.println(Ethernet.localIP());
    Serial.print("dnsServerIP: "); Serial.println(Ethernet.dnsServerIP());
    Serial.print("gatewayIP(): "); Serial.println(Ethernet.gatewayIP());
    Serial.print("subnetMask(): "); Serial.println(Ethernet.subnetMask());
    Serial.print("hostName(): "); Serial.println(Ethernet.hostName());
    Serial.print("_pinRST: "); Serial.println(Ethernet._pinRST);
    Serial.print("_pinCS: "); Serial.println(Ethernet._pinCS);
    Serial.print("_maxSockNum: "); Serial.println(Ethernet._maxSockNum);

  } else if(c == '5') {
    unsigned int localPort = 1883;
    char packetBuffer[255];
    int packetSize;
    int len;
    
    EthernetUDP Udp;
    Udp.begin(localPort);

    while(1) {
      packetSize = Udp.parsePacket();

      if (packetSize)
      {
        Serial.print(F("Received packet of size "));
        Serial.println(packetSize);
        Serial.print(F("From "));
        IPAddress remoteIp = Udp.remoteIP();
        Serial.print(remoteIp);
        Serial.print(F(", port "));
        Serial.println(Udp.remotePort());

        // read the packet into packetBufffer
        len = Udp.read(packetBuffer, 255);

        if (len > 0)
        {
          packetBuffer[len] = 0;
        }

        Serial.println(F("Contents:"));
        Serial.println(packetBuffer);
      }

      if(Serial.available()) {
        c = Serial.read();
        if(c == 'q') {
          Serial.println("Exit Udp Receive");
          break;
        }
      }
      delay(100);
    }
  } else if(c == '6') {
    unsigned int localPort = 8080;
    unsigned int serverPort = 1883;
    int ret;
    size_t wsize;
    //IPAddress server_ip(192, 168, 1, 149);
    IPAddress server_ip(192, 168, 1, 187);
    
    EthernetUDP Udp;
    Udp.begin(localPort);

    ret = Udp.beginPacket(server_ip, serverPort);
    Serial.print("Return of beginPacket: "); Serial.println(ret, DEC);
    wsize = Udp.write("hello from esp");
    Serial.print("Return of write: "); Serial.println(wsize);
    ret = Udp.endPacket();
    Serial.print("Return of endPacket: "); Serial.println(ret, DEC);

  } else if(c == '7') {
    dhcp->beginWithDHCP(nc_mac);
    Serial.print("localIP(): "); Serial.println(dhcp->getLocalIp());
    Ethernet.setLocalIP(dhcp->getLocalIp());
  } else if(c == '8') {
    Serial.print("DhcpServerIp(): "); Serial.println(dhcp->getDhcpServerIp());
    Serial.print("localIP(): "); Serial.println(dhcp->getLocalIp());
  } else {
    Serial.println("Invalid Test Number");
    return;
  }

}

void sub_test_o(void) {

  uint8_t uport = 1;
  uint8_t d_in = 0;
  char c;
  uint8_t rdata[128] = {0,};
  uint16_t rlen, rsize;
  Serial.println("Sub-test O - UART TX/RX");

  Serial.print("Select Port (1:RS232, 2:RS232 TTL): ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '1') {
    uport = 1;
    Serial1.flush();
  } else if(c == '2') {
    uport = 2;
    Serial2.flush();
    //setRS485Dir(MAX485_DIR_SEND);
  } else {
    Serial.println("Invalid port number");
    return;
  }

  Serial.println("Input data: ");
  while(1) {
    if(Serial.available()) {
      c = Serial.read();
      d_in = 1;

      if(!isalnum(c)){
        Serial.println("Quit data input");
        break;
      }
      Serial.print("d_in: "); Serial.println(c);
    }

    if(uport == 1) {
      rlen = Serial1.available();
      if(rlen){
        if(rlen > 128) rlen = 128;
        rsize = Serial1.read(rdata, rlen);
        for(int i=0; i < (int) rsize; i++) {
          Serial.print("r_in["); Serial.print(i); Serial.print("]: "); Serial.println((char) rdata[i]);
        }
      } else if (d_in) {
        Serial1.write(c);
        d_in = 0;
      }
    } else if(uport == 2) {
      rlen = Serial2.available();
      if(rlen){
        if(rlen > 128) rlen = 128;
        rsize = Serial2.read(rdata, rlen);
        for(int i=0; i < (int) rsize; i++) {
          Serial.print("r_in["); Serial.print(i); Serial.print("]: "); Serial.println((char) rdata[i]);
        }
      } else if (d_in) {
        Serial2.write(c);
        d_in = 0;
      }
    }
    delay(100);
  }

}

void sub_test_r(void) {

  char c;
  Serial.println("Sub-test R - RS485 SC Packet Communication Test");

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {
    
    make_ctrl_chg_pkt();
    send_ctrl_pkt(&sc_c_chg_pkt);

  } else if(c == '1') {
  } else {
    Serial.println("Invalid Test Number");
    return;
  }

}

void sub_test_w(void) {

  char c;
  uint16_t port_tmp = 0;
  String strTmp = "";
 
  Serial.println("Sub-test W - Settings");

  Serial.print("Input Test Number: ");
  while(1){
    if(Serial.available()) {
      c = Serial.read();
      if(isalnum(c)) break;
    }
    delay(100);
  }
  Serial.println(c);

  if(c == '0') {  // Clear Preferences
    memset((void*) &gv_settings, 0x00, sizeof(gv_settings));
    prefs.clear();

  } else if(c == '1') {  // Server settings
    Serial.print("[Server] Enter IP: ");
    char cbuf[128] = {0,};
    int idx = 0;
    while(1){
      if(Serial.available()) {
        cbuf[idx] = Serial.read();
        Serial.print(cbuf[idx]);
        if(cbuf[idx]=='\n') {
          cbuf[idx] = 0;
          Serial.println();
          break;
        } else if(cbuf[idx]=='\r'){
          cbuf[idx] = 0;
        }
        idx++;
      }
    }
    Serial.printf("Input Data String: %s\r\n", cbuf);
    if(cbuf[0]){
      memset(gv_server, 0x00, sizeof(gv_server));
      strcpy((char*) gv_server, cbuf);
      memset(gv_settings.gv_server, 0x00, SZ_SERVER_IP_BUF);
      strcpy(gv_settings.gv_server, cbuf);
      prefs.putBytes("settings", (void*) &gv_settings, sizeof(gv_settings));
    }

  } else if(c == '2') {  // Port number settings
    Serial.print("[Server] Enter Port Number: ");
    char cbuf[128] = {0,};
    int idx = 0;
    while(1){
      if(Serial.available()) {
        cbuf[idx] = Serial.read();
        Serial.print(cbuf[idx]);
        if(cbuf[idx]=='\n') {
          cbuf[idx] = 0;
          Serial.println();
          break;
        } else if(cbuf[idx]=='\r'){
          cbuf[idx] = 0;
        }
        idx++;
      }
    }
    Serial.printf("Input Data String: %s\r\n", cbuf);
    for(int i = 0; i < strlen(cbuf); i++) {
      if(isdigit(cbuf[i])) {
        strTmp += (char)cbuf[i];
      }
    }

    port_tmp = strTmp.toInt();
    Serial.printf("Input Port Number: %d\r\n", port_tmp);
    gv_settings.gv_port = gv_port = port_tmp;
    prefs.putBytes("settings", (void*) &gv_settings, sizeof(gv_settings));

  } else if(c == '3') {
    uint8_t rdata[sizeof(uint16_t) + SZ_SERVER_IP_BUF];
    if(prefs.isKey("settings")){
      size_t plen = prefs.getBytesLength("settings");
      Serial.printf("plen: %d\r\n", plen);
      prefs.getBytes("settings", rdata, plen);
      prefs.isKey("settings");
      memcpy((void*)&gv_settings, rdata, plen);
      Serial.printf("Server IP: %s:%d\r\n", gv_settings.gv_server, gv_settings.gv_port);
    } else {
      Serial.printf("settings is not initialized\r\n");
    }

  } else {
    Serial.println("Invalid Test Number");
    return;
  }
}
