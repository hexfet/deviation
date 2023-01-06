/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Deviation is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Deviation.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef MODULAR
  //Allows the linker to properly relocate
  #define UDI_Cmds PROTO_Cmds
  #pragma long_calls
#endif
#include "common.h"
#include "interface.h"
#include "mixer.h"
#include "config/model.h"
#include "config/tx.h" // for Transmitter
#include "music.h"

#if 0
#ifdef MODULAR
  //Some versions of gcc applythis to definitions, others to calls
  //So just use long_calls everywhere
  //#pragma long_calls_off
  extern unsigned _data_loadaddr;
  const unsigned long protocol_type = (unsigned long)&_data_loadaddr;
#endif
#endif

#ifdef PROTO_HAS_NRF24L01

#include "iface_nrf24l01.h"

#ifdef EMULATOR
#define USE_FIXED_MFGID
#define BIND_COUNT 5
#else
#define BIND_COUNT 1000
#endif

// Timeout for callback in uSec, 4ms=4000us for UDI
// ???
//#define PACKET_PERIOD 4000

#define BIND_PACKET_PERIOD 5000
#define PACKET_PERIOD 15000

#define BIND_PACKETS_PER_CHANNEL 11
#define PACKETS_PER_CHANNEL 11

#define NUM_RF_CHANNELS 16


#define INITIAL_WAIT 50000

// Time to stay on a RF channel
#define CHANNEL_PERIOD 360000

#define PACKET_CHKTIME 100

// For readability
enum {
    FLAG_CAMERA  = 1,
    FLAG_VIDEO   = 2,
    FLAG_MODE2   = 4,
    FLAG_FLIP360 = 8,
    FLAG_FLIP    =16,
    FLAG_LIGHTS  =32
};

// For code readability
enum {
    CHANNEL1 = 0,
    CHANNEL2,
    CHANNEL3,
    CHANNEL4,
    CHANNEL5,
    CHANNEL6,
    CHANNEL7,
    CHANNEL8,
    CHANNEL9,
    CHANNEL10
};

// This is maximum payload size used in UDI protocols
#define PAYLOADSIZE 16



static u8 packet[PAYLOADSIZE];
static u8 payload_size;  // Bytes in payload for selected variant
static u8 bind_channel;
static u8 packets_to_hop;
static u8 packet_sent;
static u8 packets_to_check;  // BIND_RX phase needs to receive/auto-ack more than one packet for RX to switch to next phase, it seems
static u8 packets_to_send;   // Number of packets to send / check for in current bind phase
static u8 bind_step_success; // Indicates successfull transmission / receive of bind reply during current bind phase
static u8 tx_id[3];
static u8 rx_id[3];
static u8 random[3];         // 3 random bytes choosen by TX, sent in BIND packets. Lower nibble of first byte sets index in RF CH table to use for BIND2

static u8 rf_ch_num;
static u16 counter;
static u32 packet_counter;
static u8 tx_power;
//static u8 auto_flip; // Channel 6 <= 0 - disabled > 0 - enabled
static u8 throttle, rudder, elevator, aileron, flags;

static u8 print_packet_once;  // DEBUG: Prints next transmitted packet to serial console (flag gets cleared afterwards)


//
static u8 phase;
enum {
    UDI_INIT2 = 0,
    UDI_INIT2_NO_BIND,
    UDI_BIND1_TX,
    UDI_BIND1_RX,
    UDI_BIND2_TX,
    UDI_BIND2_RX,
    UDI_DATA
};


// Known UDI 2.4GHz protocol variants, all using BK2421
//  * UDI U819 coaxial 3ch helicoper
//  * UDI U816/817/818 quadcopters
//    - "V1" with orange LED on TX, U816 RX labeled '' , U817/U818 RX labeled 'UD-U817B'
//    - "V2" with red LEDs on TX, U816 RX labeled '', U817/U818 RX labeled 'UD-U817OG'
//    - "V3" with green LEDs on TX. Did not get my hands on yet.
//  * U830 mini quadcopter with tilt steering ("Protocol 2014")
//  * U839 nano quadcopter ("Protocol 2014")

static const char * const udi_opts[] = {
  _tr_noop("Format"),  _tr_noop("U816 V1 (orange)"), _tr_noop("U816 V2 (red)"), _tr_noop("U839 (2014)"), NULL,
  _tr_noop("Re-bind"),  _tr_noop("No"), _tr_noop("Yes"), NULL,
//  _tr_noop("Blink"),  _tr_noop("No"), _tr_noop("Yes"), NULL,
  NULL
};

enum {
    FORMAT_U816_V1 = 0,
    FORMAT_U816_V2,
    FORMAT_U839_2014
};

enum {
    PROTOOPTS_FORMAT = 0,
    PROTOOPTS_STARTBIND,
};
enum {
    STARTBIND_NO  = 0,
    STARTBIND_YES = 1,
};

// This are frequency hopping tables for UDI protocols

// U816 V1 (Orange LED) Bind CH 0x07
// TX ID 0x57, 0x5A, 0x2D
static const u8 freq_hopping_u816_v1[NUM_RF_CHANNELS] = {
 0x07, 0x21, 0x49, 0x0B, 0x39, 0x10, 0x25, 0x42,
 0x1D, 0x31, 0x35, 0x14, 0x28, 0x3D, 0x18, 0x2D
};

// Protocol 2014 (U830,U839,...) BIND CH 0x23 (second entry)
// DATA: hops ~ every 0.361s (0.350 ... 0.372)
static const u8 freq_hopping_u839[NUM_RF_CHANNELS] = {
 0x08, 0x23, 0x48, 0x0D, 0x3B, 0x12, 0x27, 0x44,
 0x1F, 0x33, 0x37, 0x16, 0x2A, 0x3F, 0x1A, 0x2F
};

// Points to proper table
static const u8 * rf_channels = NULL;


// Bit vector from bit position
#define BV(bit) (1 << bit)

// Packet ack status values
enum {
    PKT_PENDING = 0,
    PKT_ACKED,
    PKT_TIMEOUT
};

static u8 packet_ack()
{
    switch (NRF24L01_ReadReg(NRF24L01_07_STATUS) & (BV(NRF24L01_07_TX_DS) | BV(NRF24L01_07_MAX_RT))) {
    case BV(NRF24L01_07_TX_DS):
        return PKT_ACKED;
    case BV(NRF24L01_07_MAX_RT):
        return PKT_TIMEOUT;
    }
    return PKT_PENDING;
}

static void UDI_init()
{
    NRF24L01_Initialize();
    //NRF24L01_SetTxRxMode(TX_EN);
    
    switch (Model.proto_opts[PROTOOPTS_FORMAT]) {
    case FORMAT_U816_V1:
        rf_channels = freq_hopping_u816_v1;
        payload_size = 8;
        break;

    case FORMAT_U816_V2:
        rf_channels = NULL; // NO HOPPING !
        payload_size = 7;
        break;

    case FORMAT_U839_2014:
        // UDI 2014 Protocol (U830, U839, all other new products ?)
        rf_channels = freq_hopping_u839;
        payload_size = 8;
        break;
    }
    
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, payload_size);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x07);     // Clear status bits
    
    if ((Model.proto_opts[PROTOOPTS_FORMAT] == FORMAT_U816_V1) ||
        (Model.proto_opts[PROTOOPTS_FORMAT] == FORMAT_U816_V2)) {
        NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, 0x27);   // 
        NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x3A); // 
        NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x01);   // 3 byte address
        NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0 
        NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x3F);      // Auto-acknowledge on all data pipers, same as YD
        if (Model.proto_opts[PROTOOPTS_FORMAT] == FORMAT_U816_V1) {
            NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x7F);     // 
        } else {
            NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x7A);     // 
        }
    } else
    if (Model.proto_opts[PROTOOPTS_FORMAT] == FORMAT_U839_2014) {
        NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, 0x0F);   // 2Mbps air rate, 5dBm RF output power, high LNA gain
        NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x1A); // 500uS retransmit t/o, 10 tries (same as YD)
        NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x01);   // 3 byte address
        NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0 
        NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x3F);      // Auto-acknowledge on all data pipers, same as YD
        NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0F);     // Enable CRC, 2 byte CRC, PWR UP, PRIMARY RX
    }
    
#if 0    
    NRF24L01_ReadReg(NRF24L01_1D_FEATURE);
    NRF24L01_Activate(0x73);
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x00);
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);
    NRF24L01_ReadReg(NRF24L01_07_STATUS);
#endif    

    // Beken bank switch
    NRF24L01_Activate(0x53);
    if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & 0x80)
    {
        printf("BK2421 detected\n");
        NRF24L01_WriteRegisterMulti(0x00, (u8 *) "\x40\x4B\x01\xE2", 4); // same as V2x2
        NRF24L01_WriteRegisterMulti(0x01, (u8 *) "\xC0\x4B\x00\x00", 4); // same as V2x2
        NRF24L01_WriteRegisterMulti(0x02, (u8 *) "\xD0\xFC\x8C\x02", 4); // same as V2x2
        
        NRF24L01_WriteRegisterMulti(0x03, (u8 *) "\x99\x00\x39\x21", 4); // same as YD, V2x2: 0xF9003921
        NRF24L01_WriteRegisterMulti(0x04, (u8 *) "\xF9\x96\x82\x1B", 4); // YD717: 0xD996821B, V2x2: 0xC1969A1B, 
                                                                         // Datasheet: 0xD99E860B (High Power), 0xD99E8621 (single carrier mode), 0xD9BE8621 (single carrier mode in other BK doc)

        NRF24L01_WriteRegisterMulti(0x05, (u8 *) "\x24\x06\x7F\xA6", 4); // same as V2x2
        NRF24L01_WriteRegisterMulti(0x0C, (u8 *) "\x00\x12\x73\x00", 4); // same as V2x2
        NRF24L01_WriteRegisterMulti(0x0D, (u8 *) "\x46\xB4\x80\x00", 4); // same as V2x2
        NRF24L01_WriteRegisterMulti(0x0E, (u8 *) "\x41\x10\x04\x82\x20\x08\x08\xF2\x7D\xEF\xFF", 11);
        NRF24L01_WriteRegisterMulti(0x04, (u8 *) "\xFF\x96\x82\x1B", 4); // V2x2: 0xC7969A1B
        NRF24L01_WriteRegisterMulti(0x04, (u8 *) "\xF9\x96\x82\x1B", 4); // V2x2: 0xC1969A1B
        //NRF24L01_ReadReg(0x07);
    } else {
        printf("nRF24L01 detected\n");
    }
    NRF24L01_Activate(0x53); // switch bank back
    
    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    u8 status = NRF24L01_ReadReg(NRF24L01_07_STATUS);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, status);
    
    status = NRF24L01_ReadReg(NRF24L01_07_STATUS);
    NRF24L01_FlushTx();
    status = NRF24L01_ReadReg(NRF24L01_07_STATUS);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, status);

    // Implicit delay in callback
    // delayMicroseconds(120)
}

static void UDI_init2()
{
    NRF24L01_FlushTx();
    print_packet_once = 1;
    bind_step_success = 0;
    packet_sent = 0;

    switch (Model.proto_opts[PROTOOPTS_FORMAT]) {
    case FORMAT_U816_V1:
        rf_ch_num = 0;
        bind_channel = rf_channels[rf_ch_num++];
        break;
    case FORMAT_U816_V2:
        rf_ch_num = 0x07; // This is actual channel. No hopping here
        bind_channel = 0;
        break;
    case FORMAT_U839_2014:
        rf_ch_num = 1;
        bind_channel = rf_channels[rf_ch_num++];
        break;
    }
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, bind_channel);

    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, (u8 *) "\xe7\x7e\xe7", 3);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, (u8 *) "\xe7\x7e\xe7", 3);

    // Turn radio power on
    NRF24L01_SetTxRxMode(TX_EN);
    u8 config = BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO) | BV(NRF24L01_00_PWR_UP);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, config);
    // Implicit delay in callback
    // delayMicroseconds(150);
}

static void set_tx_id(u32 id)
{
    tx_id[0] = (id >> 16) & 0xFF;
    tx_id[1] = (id >> 8) & 0xFF;
    tx_id[2] = (id >> 0) & 0xFF;
    
    u32 val = rand32();
    random[0] = val & 0xff;
    random[1] = (val >> 8 ) & 0xff;
    random[2] = (val >> 16 ) & 0xff;
/*    
    // FIXME
    // This one has been observed, leads to RF CH 0x1F (#08) used for BIND2
    random[0] = 0x98;
    random[1] = 0x80;
    random[2] = 0x5B;
*/    
}

static void add_pkt_checksum()
{
  // CHECKSUM was introduced with 2014 protocol
  if (Model.proto_opts[PROTOOPTS_FORMAT] < FORMAT_U839_2014) return;
  u8 sum = 0;
  for (u8 i = 0; i < payload_size-1;  ++i) sum += packet[i];
  packet[payload_size-1] = sum & 0x3f; // *sick*
}


static u8 convert_channel(u8 num, u8 chn_max, u8 sign_ofs)
{
    s32 ch = Channels[num];
    if (ch < CHAN_MIN_VALUE) {
        ch = CHAN_MIN_VALUE;
    } else if (ch > CHAN_MAX_VALUE) {
        ch = CHAN_MAX_VALUE;
    }
    s32 chn_val;
    if (sign_ofs) chn_val = (((ch * chn_max / CHAN_MAX_VALUE) + sign_ofs) >> 1);
    else chn_val = (ch * chn_max / CHAN_MAX_VALUE);
    if (chn_val < 0) chn_val = 0;
    else if (chn_val > chn_max) chn_val = chn_max;
    return (u8) chn_val;
}


static void read_controls(u8* throttle, u8* rudder, u8* elevator, u8* aileron,
                          u8* flags)
{
    // Protocol is registered AETRG, that is
    // Aileron is channel 0, Elevator - 1, Throttle - 2, Rudder - 3
    // Sometimes due to imperfect calibration or mixer settings
    // throttle can be less than CHAN_MIN_VALUE or larger than
    // CHAN_MAX_VALUE. As we have no space here, we hard-limit
    // channels values by min..max range

    // Channel 3: throttle is 0-100
    *throttle = convert_channel(CHANNEL3, 0x64, 0);

    // Channel 4
    *rudder = convert_channel(CHANNEL4, 0x3f, 0x20);

    // Channel 2
    *elevator = convert_channel(CHANNEL2, 0x3f, 0x20);

    // Channel 1
    *aileron = convert_channel(CHANNEL1, 0x3f, 0x20);

    // Channel 5
    if (Channels[CHANNEL5] <= 0) *flags &= ~FLAG_FLIP360;
    else *flags |= FLAG_FLIP360;

    // Channel 6
    if (Channels[CHANNEL6] <= 0) *flags &= ~FLAG_FLIP;
    else *flags |= FLAG_FLIP;

    // Channel 7
    if (Channels[CHANNEL7] <= 0) *flags &= ~FLAG_CAMERA;
    else *flags |= FLAG_CAMERA;

    // Channel 8
    if (Channels[CHANNEL8] <= 0) *flags &= ~FLAG_VIDEO;
    else *flags |= FLAG_VIDEO;

    // Channel 9
    if (Channels[CHANNEL9] <= 0) *flags &= ~FLAG_LIGHTS;
    else *flags |= FLAG_LIGHTS;

    // Channel 10
    if (Channels[CHANNEL10] <= 0) *flags &= ~FLAG_MODE2;
    else *flags |= FLAG_MODE2;

    // Print channels every second or so
    if ((packet_counter & 0xFF) == 1) {
        printf("Raw channels: %d, %d, %d, %d, %d, %d, %d, %d\n",
               Channels[0], Channels[1], Channels[2], Channels[3],
               Channels[4], Channels[5], Channels[6], Channels[7]);
        printf("Aileron %d, elevator %d, throttle %d, rudder %d\n",
               (s16) *aileron, (s16) *elevator, (s16) *throttle, (s16) *rudder);
    }
}

static void print_packet(const u8* data, u8 size)
{
    printf("Payload ");
    for (int i=0;i<size;i++) printf("%02x ",data[i]);
    printf("\n");
}

static void send_packet(u8 bind)
{
    packet[7] = 0x4A;
    if (bind == 1) {
        // Bind phase 1
        // MAGIC
        packet[0] = 0x5A;  // NOTE: Also 0xF3, when RX does not ACK packets (U839, only TX on) ...
        // Current Address / TX ID
        if (Model.proto_opts[PROTOOPTS_FORMAT] == FORMAT_U839_2014) {
            // U839: Current RX/TX Addr
            packet[1] = 0xE7;
            packet[2] = 0x7E;
            packet[3] = 0xE7;
        } else {
            // U816: ID Fixed per TX
            packet[1] = tx_id[0];
            packet[2] = tx_id[1];
            packet[3] = tx_id[2];
        }
        // Pseudo random values (lower nibble of packet[4] determines index of RF CH used in BIND2)
        packet[4] = random[0];
        packet[5] = random[1];
        packet[6] = random[2];
        if (Model.proto_opts[PROTOOPTS_FORMAT] == FORMAT_U839_2014) {
            packet[7] = (packet_counter < 4) ? 0x3f : 0x04; // first four packets use 0x3f here, then 0x04
        }
    } else if (bind == 2) {
        // Bind phase 2
        // MAGIC
        packet[0] = 0xAA;
        // Current Address (RX "ID", pseudorandom again)
        packet[1] = rx_id[0];
        packet[2] = rx_id[1];
        packet[3] = rx_id[2];
        // Pseudo random values
        packet[4] = random[0];
        packet[5] = random[1];
        packet[6] = random[2];
        if (Model.proto_opts[PROTOOPTS_FORMAT] == FORMAT_U839_2014) {
            packet[7] = 0x04;
        }
    } else {
        // regular packet
        // Read channels (converts to required ranges)
        read_controls(&throttle, &rudder, &elevator, &aileron, &flags);
        // MAGIC
        packet[0] = 0x55;
        packet[1] = throttle; // throttle is 0-0x64
        // 3 Channels packed into 2 bytes (5bit per channel)
        u16 encoded = (rudder << 11) | (elevator << 6) | (aileron << 1);
        packet[2] = (encoded >> 8) & 0xff;
        packet[3] = encoded & 0xff;
        // Trims and flags (0x20 = center)
        packet[4] = 0x20; // rudder trim 6bit
        packet[5] = 0x20; // elev   trim 6bit
        packet[6] = 0x20; // ail    trim 6bit
        
        if (flags & FLAG_FLIP) packet[4] |= 0x80;    // "Directional" flip
        if (flags & FLAG_LIGHTS) packet[4] |= 0x40;  // Light on/off

        if (flags & FLAG_MODE2) packet[5] |= 0x80;   // High rate ("Mode2")
        if (flags & FLAG_FLIP360) packet[5] |= 0x40; // 360 degree flip

        if (flags & FLAG_VIDEO) packet[6] |= 0x80;   // Video recording on/off
        if (flags & FLAG_CAMERA) packet[6] |= 0x40;  // Take picture

        // NOTE: Only newer protocols have this (handled by routine)
        add_pkt_checksum();
    }

    if (print_packet_once) {
        print_packet_once = 0;
        print_packet(packet, payload_size);
    }
    
    u8 status = NRF24L01_ReadReg(NRF24L01_07_STATUS);
    NRF24L01_WriteReg(NRF24L01_07_STATUS,status);

    if (packet_sent && bind && (status & BV(NRF24L01_07_TX_DS))) {
        printf("BIND packet was acknowledged by RX\n");
        bind_step_success = 1;
    }

    packet_sent = 0;
    
    // Check if its time to change channel
    // This seems to be done by measuring time,
    // not by counting packets, on UDI transmitters
    // NOTE: Seems even in bind phase channels are changed
    
    // NOTE: Only hop in TX mode ???
    if (rf_channels && (bind == 0) && (packets_to_hop-- == 0)) {
        u8 rf_ch = rf_channels[rf_ch_num];
        printf("Switch to RF_CH %02x (#%d)\n",rf_ch,rf_ch_num);
        rf_ch_num++;
        rf_ch_num %= NUM_RF_CHANNELS;
        //Serial.print(rf_ch); Serial.write("\n");
        NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_ch);
        
        packets_to_hop = bind ? BIND_PACKETS_PER_CHANNEL : PACKETS_PER_CHANNEL;
    }
    NRF24L01_FlushTx();
    NRF24L01_WritePayload(packet, payload_size);
    ++packet_counter;
    packet_sent = 1;
//    radio.ce(HIGH);
//    delayMicroseconds(15);
    // It saves power to turn off radio after the transmission,
    // so as long as we have pins to do so, it is wise to turn
    // it back.
//    radio.ce(LOW);

    // Check and adjust transmission power. We do this after
    // transmission to not bother with timeout after power
    // settings change -  we have plenty of time until next
    // packet.
    
    
/*
    // TESTING: Remove this, maybe it hurts ?
    // (note: rf_setup member if NRF not set)    
    if (! rf_ch_num && tx_power != Model.tx_power) {
        //Keep transmit power updated
        tx_power = Model.tx_power;
        NRF24L01_SetPower(tx_power);
    }
*/    
}


// old?? MODULE_CALLTYPE
static u16 UDI_callback()
{
    switch (phase) {
    case UDI_INIT2:
        UDI_init2();
        MUSIC_Play(MUSIC_TELEMALARM1);
        phase = UDI_BIND1_TX;
        return 120;
        break;
    case UDI_INIT2_NO_BIND:
        // Do nothing (stay forever)
        // Cannot re-bind on UDI protocol since IDs are random
        return 10000; // 10ms
        break;
    case UDI_BIND1_TX:
        if (packet_sent && packet_ack() == PKT_ACKED) {
            printf("BIND1 packet was acknowledged by RX\n");
            bind_step_success = 1;
        }
        if (bind_step_success) {
            printf("UDI_BIND1_TX: Bind packets have been ACKed by RX\n");
            // All fine, wait for reply of receiver
            phase = UDI_BIND1_RX;
            
            // switch to RX mode (this is from NE260)
            //while (!(NRF24L01_ReadReg(NRF24L01_07_STATUS) & (BV(NRF24L01_07_MAX_RT) | BV(NRF24L01_07_TX_DS)))) ;
            //NRF24L01_WriteReg(NRF24L01_07_STATUS, BV(NRF24L01_07_TX_DS));
    
            NRF24L01_SetTxRxMode(RX_EN);
            NRF24L01_FlushRx();
            
            NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0F);
            bind_step_success = 0;
            //packets_to_check = 12; // according to SPI traces on U817B RX it receives 12 packets (and answers with 5)
            packets_to_check = 3;
        } else {
            send_packet(1);
        }
        return BIND_PACKET_PERIOD;
        break;
    case UDI_BIND1_RX:
        // Check if data has been received
        if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & BV(NRF24L01_07_RX_DR) ) {
            printf("UDI_BIND1_RX: Got data\n");
          
            u8 data[PAYLOADSIZE];
            u8 status = NRF24L01_ReadPayload(data, payload_size);
            NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x4E); // On original TX this is done on LAST packet check only !
            NRF24L01_FlushRx();
            
            print_packet(data,payload_size);
          
            // Verify MAGIC and Random ID
            // (may be reply to bind packet from other TX)
            if ((data[0] == 0xA5) &&
                (data[4] == random[0]) &&
                (data[5] == random[1]) &&
                (data[6] == random[2]) &&
                (data[7] == random[2])) {
                rx_id[0] = data[1];
                rx_id[1] = data[2];
                rx_id[2] = data[3];
                if (Model.proto_opts[PROTOOPTS_FORMAT] != FORMAT_U816_V2) {
                    rf_ch_num = random[0] & 0x0f;
                    printf("UDI_BIND1_RX: Got reply with RX ID: %02x %02x %02x random: %02x %02x %02x. Status %02x\n",
                           rx_id[0],rx_id[1],rx_id[2], random[0],random[1],random[2], status);
                }
                bind_step_success = 1;
            } else {
                printf("UDI_BIND1_RX: Unexpected reply (status %02x): ",status);
                for (int i=0;i<payload_size;i++) printf("%02x ",data[i]);
                printf("\n");
            }
        }
        // RX seems to need more than one ACK
        if (packets_to_check) packets_to_check--;
        //NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0F);
        if (bind_step_success && !packets_to_check) {
            printf("UDI_BIND1_RX: Bind reply packets have been received\n");
            // All fine, switch address and RF channel,
            // send bind packets with channel hopping now
            phase = UDI_BIND2_TX;
            print_packet_once = 1;
            
            packet_sent = 0;
            packets_to_send = 4;
            bind_step_success = 0;
            
            NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rx_id, 3);
            NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_id, 3);
            printf("Using RX/TX Addr %02x %02x %02x\n",rx_id[0],rx_id[1],rx_id[2]);
            
            if (Model.proto_opts[PROTOOPTS_FORMAT] != FORMAT_U816_V2) {
                // Switch RF channel
                printf("Switching to RF Channel %02x\n",rf_channels[rf_ch_num]);
                NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_channels[rf_ch_num++]);
                rf_ch_num %= NUM_RF_CHANNELS;
            }

            NRF24L01_FlushTx();
            NRF24L01_FlushRx();
            NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x7E);

            NRF24L01_SetTxRxMode(TX_EN);
            //NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0E)
            
            return 10; // 10 µs (start sending immediately)
        }
        return BIND_PACKET_PERIOD;
        break;

    case UDI_BIND2_TX:
        if (packet_sent && packet_ack() == PKT_ACKED) {
            printf("BIND2 packet was acknowledged by RX\n");
            bind_step_success = 1;
        }
        send_packet(2);
        if (packets_to_send) --packets_to_send;
        if (bind_step_success || !packets_to_send) {
            // Seems the original TX ignores AACK, too !
            // U816 V1: 3 packets send, U839: 4 packets send
            printf("UDI_BIND2_TX: Going to next phase (Got ACK: %d)\n",bind_step_success);
            // All fine, wait for reply of receiver
            phase = UDI_BIND2_RX;
            
            NRF24L01_SetTxRxMode(RX_EN);
            NRF24L01_FlushRx();
            
            NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0F);
            bind_step_success = 0;
            packets_to_check = 14; // ???
        }
        return bind_step_success ? 4000 : 12000; // 4ms if no packed acked yet, 12ms afterwards
//        return 120; // FIXME: Varies for first three packets !!!
        
        break;

    case UDI_BIND2_RX:
        // Check if data has been received
        if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & BV(NRF24L01_07_RX_DR) ) {
            printf("UDI_BIND2_RX: Got data\n");

            u8 data[PAYLOADSIZE];
            u8 status = NRF24L01_ReadPayload(data, payload_size);
            NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x4E);
            NRF24L01_FlushRx();

            print_packet(data,payload_size);
          
            // Verify MAGIC, RX Addr, Random ID
            // (may be reply to bind packet from other TX)
            if ((data[0] == 0xDD) &&
                (data[1] == rx_id[0]) &&
                (data[2] == rx_id[1]) &&
                (data[3] == rx_id[2]) &&
                (data[4] == random[0]) &&
                (data[5] == random[1]) &&
                (data[6] == random[2]) &&
                (data[7] == random[2])) {
                bind_step_success = 1;
                printf("UDI_BIND2_RX: Got valid reply from RX\n");
            } else {
                printf("UDI_BIND2_RX: Unexpected reply (status %02x): ",status);
                for (int i=0;i<payload_size;i++) printf("%02x ",data[i]);
                printf("\n");
            }
        }
        // RX seems to need more than one ACK
        if (packets_to_check) packets_to_check--;
        //NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0F);
        if (bind_step_success && !packets_to_check) {
            printf("UDI_BIND2_RX: Got reply from RX, switching to UDI_DATA\n");
            phase = UDI_DATA;
            NRF24L01_SetTxRxMode(TX_EN);
            NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x7E);
            NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0E);
            NRF24L01_FlushTx();
           
            // Switch RF channel
            if (Model.proto_opts[PROTOOPTS_FORMAT] == FORMAT_U816_V2) {
                // FIXED RF Channel
                NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_ch_num);
            } else {
                NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_channels[rf_ch_num++]);
                rf_ch_num %= NUM_RF_CHANNELS;
            }
            
            flags = 0;
            PROTOCOL_SetBindState(0);
            MUSIC_Play(MUSIC_DONE_BINDING);
        }
        return BIND_PACKET_PERIOD;
        break;
    case UDI_DATA:
        if (packet_sent && packet_ack() != PKT_ACKED) {
            //printf("Packet not sent yet\n");
            return PACKET_CHKTIME;
        }
        send_packet(0);
        break;
    }
    // Packet every 15ms
    return PACKET_PERIOD;
}

// Generate internal id from TX id and manufacturer id (STM32 unique id)
static void initialize_tx_id()
{
    u32 lfsr = 0xb2c54a2ful;

#ifndef USE_FIXED_MFGID
    u8 var[12];
    MCU_SerialNumber(var, 12);
    printf("Manufacturer id: ");
    for (int i = 0; i < 12; ++i) {
        printf("%02X", var[i]);
        rand32_r(&lfsr, var[i]);
    }
    printf("\r\n");
#endif

    if (Model.fixed_id) {
       for (u8 i = 0, j = 0; i < sizeof(Model.fixed_id); ++i, j += 8)
           rand32_r(&lfsr, (Model.fixed_id >> j) & 0xff);
    }
    // Pump zero bytes for LFSR to diverge more
    for (u8 i = 0; i < sizeof(lfsr); ++i) rand32_r(&lfsr, 0);

    // observed on U839 TX
    lfsr = 0x457C27;

    set_tx_id(lfsr);
}

static void initialize()
{
    CLOCK_StopTimer();
    tx_power = Model.tx_power;
    packet_counter = 0;
    UDI_init();
    phase = UDI_INIT2;
    counter = BIND_COUNT;
    PROTOCOL_SetBindState(BIND_COUNT * PACKET_PERIOD / 1000); //msec

    initialize_tx_id();

    CLOCK_StartTimer(INITIAL_WAIT, UDI_callback);
}

uintptr_t UDI_Cmds(enum ProtoCmds cmd)
{
    switch(cmd) {
        case PROTOCMD_INIT:
            initialize();
            return 0;
        case PROTOCMD_DEINIT:
        case PROTOCMD_RESET:
            CLOCK_StopTimer();
            return (NRF24L01_Reset() ? 1L : -1L);
        case PROTOCMD_CHECK_AUTOBIND: return 0L; //Never Autobind
        case PROTOCMD_BIND:  initialize(); return 0;
        case PROTOCMD_NUMCHAN: return  10L; // T, R, E, A, LED (on/off/blink), Auto flip, 4 unknown flags
        case PROTOCMD_DEFAULT_NUMCHAN: return 6L;
        case PROTOCMD_CURRENT_ID: return Model.fixed_id;
        case PROTOCMD_GETOPTIONS: return (uintptr_t)udi_opts;
        case PROTOCMD_TELEMETRYSTATE: return PROTO_TELEM_UNSUPPORTED;
#if 0
        case PROTOCMD_SET_TXPOWER:
            tx_power = Model.tx_power;
            NRF24L01_SetPower(tx_power);
            break;
#endif
        default: break;
    }
    return 0;
}
#endif
