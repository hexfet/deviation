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
  #define SBUS_Cmds PROTO_Cmds
  #pragma long_calls
#endif

#include "common.h"
#include "interface.h"
#include "mixer.h"
#include "config/model.h"
#include "config/tx.h"

#ifdef MODULAR
  #pragma long_calls_off
  extern unsigned _data_loadaddr;
  const unsigned long protocol_type = (unsigned long)&_data_loadaddr;
#endif

#define SBUS_CHANNELS 16
#define SBUS_PACKET_SIZE 25
static u8 packet[SBUS_PACKET_SIZE];
u8 num_channels;

volatile u8 state;

static void build_data_pkt()
{
    int ii;
    packet[0] = 0x0F; 
    for (ii = 0; ii < num_channels; ii++) {
        u16 value = ((((s32)Channels[ii] - 880) << 3) / 5);
        packet[ii * 2 + 1] = value >> 8;
        packet[ii * 2 + 2] = value & 0xFF;
    }
    packet[SBUS_PACKET_SIZE-1] = 0x00;
}

static u16 serial_cb()
{
    build_data_pkt();
    UART_Send(packet, sizeof (packet));
#ifdef EMULATOR
    return 3000;
#else
    return 10000;
#endif
}

static void initialize()
{
    CLOCK_StopTimer();
    if (PPMin_Mode())
    {
        return;
    }
    UART_SetDataRate(100000);
#if HAS_EXTENDED_AUDIO
#if HAS_AUDIO_UART5
    if (Transmitter.audio_uart5)
#endif
    Transmitter.audio_player = AUDIO_DISABLED; // disable voice commands on serial port
#endif
    num_channels = Model.num_channels;
    state = 0;
    CLOCK_StartTimer(1000, serial_cb);
}

const void * SBUS_Cmds(enum ProtoCmds cmd)
{
    switch(cmd) {
        case PROTOCMD_INIT:  initialize(); return 0;
        case PROTOCMD_DEINIT: UART_SetDataRate(0); return 0;
        case PROTOCMD_CHECK_AUTOBIND: return (void *)1L;
        case PROTOCMD_BIND:  initialize(); return 0;
        case PROTOCMD_NUMCHAN: return (void *)16L;
        case PROTOCMD_DEFAULT_NUMCHAN: return (void *)8L;
        case PROTOCMD_TELEMETRYSTATE: return (void *)(long)PROTO_TELEM_UNSUPPORTED;
        default: break;
    }
    return 0;
}
