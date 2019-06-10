#ifndef PROTODEF

#else
#ifdef PROTO_HAS_CYRF6936
PROTODEF(PROTOCOL_DEVO,   CYRF6936, EATRG, DEVO_Cmds, "DEVO")
PROTODEF(PROTOCOL_WK2801, CYRF6936, EATRG, WK2x01_Cmds, "WK2801")
#ifndef DEFINE_FUNCS
PROTODEF(PROTOCOL_WK2601, CYRF6936, EATRG, WK2x01_Cmds, "WK2601")
PROTODEF(PROTOCOL_WK2401, CYRF6936, EATRG, WK2x01_Cmds, "WK2401")
PROTODEF(PROTOCOL_DSM2,   CYRF6936, TAERG, DSM2_Cmds,   "DSM2")
#endif
PROTODEF(PROTOCOL_DSMX,   CYRF6936, TAERG, DSM2_Cmds,   "DSMX")
PROTODEF(PROTOCOL_J6PRO,  CYRF6936, AETRG, J6PRO_Cmds,  "J6Pro")
PROTODEF(PROTOCOL_WFLY ,  CYRF6936, AETRG, WFLY_Cmds,  "WFLY")
#endif //PROTO_HAS_CYRF6936
#ifdef PROTO_HAS_A7105
PROTODEF(PROTOCOL_FLYSKY, A7105, AETRG, FLYSKY_Cmds, "Flysky")
PROTODEF(PROTOCOL_AFHDS2A, A7105, AETRG, AFHDS2A_Cmds, "AFHDS-2A")
PROTODEF(PROTOCOL_HUBSAN, A7105, AETRG, HUBSAN_Cmds, "Hubsan4")
PROTODEF(PROTOCOL_JOYSWAY, A7105, AETRG, JOYSWAY_Cmds, "Joysway")
PROTODEF(PROTOCOL_BUGS3, A7105, AETRG, BUGS3_Cmds, "Bugs3")
#endif //PROTO_HAS_A7105
#ifdef PROTO_HAS_CC2500
PROTODEF(PROTOCOL_FRSKY2WAY, CC2500, AETRG, FRSKY2WAY_Cmds, "Frsky")
PROTODEF(PROTOCOL_FRSKY1WAY, CC2500, AETRG, FRSKY1WAY_Cmds, "Frsky-V8")
PROTODEF(PROTOCOL_FRSKYX, CC2500, AETRG, FRSKYX_Cmds, "FrskyX")
PROTODEF(PROTOCOL_SKYARTEC, CC2500, AETRG, SKYARTEC_Cmds, "Skyartec")
PROTODEF(PROTOCOL_SFHSS, CC2500, AETRG, SFHSS_Cmds, "S-FHSS")
PROTODEF(PROTOCOL_CORONA, CC2500, AETRG, Corona_Cmds, "Corona")
PROTODEF(PROTOCOL_HITEC,  CC2500, AETRG, Hitec_Cmds, "Hitec")
PROTODEF(PROTOCOL_E010, CC2500, AETRG, E010_Cmds, "E010")
#endif //PROTO_HAS_CC2500
#ifdef PROTO_HAS_NRF24L01
PROTODEF(PROTOCOL_V202, NRF24L01, AETRG, V202_Cmds, "V202")
PROTODEF(PROTOCOL_SLT, NRF24L01, AETRG, SLT_Cmds, "SLT")
PROTODEF(PROTOCOL_HiSky, NRF24L01, AETRG, HiSky_Cmds, "HiSky")
PROTODEF(PROTOCOL_YD717, NRF24L01, AETRG, YD717_Cmds, "YD717")
PROTODEF(PROTOCOL_SymaX, NRF24L01, AETRG, SYMAX_Cmds, "SymaX")
PROTODEF(PROTOCOL_CFlie, NRF24L01, AETRG, CFlie_Cmds, "CFlie")
PROTODEF(PROTOCOL_H377, NRF24L01, AETRG, H377_Cmds, "H377")
PROTODEF(PROTOCOL_HM830, NRF24L01, TAERG, HM830_Cmds, "HM830")
PROTODEF(PROTOCOL_KN, NRF24L01, TAERG, KN_Cmds, "KN")
PROTODEF(PROTOCOL_ESKY150, NRF24L01, TAERG, ESKY150_Cmds, "ESky150")
PROTODEF(PROTOCOL_ESKY, NRF24L01, AETRG, ESKY_Cmds, "ESky")
PROTODEF(PROTOCOL_BLUEFLY, NRF24L01, AETRG, BlueFly_Cmds, "BlueFly")
//PROTODEF(PROTOCOL_NE260, NRF24L01, AETRG, NE260_Cmds, "NE260")
PROTODEF(PROTOCOL_CX10, NRF24L01, AETRG, CX10_Cmds, "CX10")
PROTODEF(PROTOCOL_CG023, NRF24L01, AETRG, CG023_Cmds, "CG023")
PROTODEF(PROTOCOL_HONTAI, NRF24L01, AETRG, HonTai_Cmds, "HonTai")
PROTODEF(PROTOCOL_FY326, NRF24L01, AETRG, FY326_Cmds, "FY326")
PROTODEF(PROTOCOL_BAYANG, NRF24L01, AETRG, Bayang_Cmds, "Bayang")
PROTODEF(PROTOCOL_H8_3D, NRF24L01, AETRG, H8_3D_Cmds, "H8-3D")
PROTODEF(PROTOCOL_MJXQ, NRF24L01, AETRG, MJXq_Cmds, "MJXq")
PROTODEF(PROTOCOL_MT99XX, NRF24L01, AETRG, MT99XX_Cmds, "MT99XX")
PROTODEF(PROTOCOL_ASSAN, NRF24L01, AETRG, ASSAN_Cmds, "ASSAN")
PROTODEF(PROTOCOL_FQ777, NRF24L01, AETRG, FQ777_Cmds, "FQ777")
PROTODEF(PROTOCOL_INAV, NRF24L01, AETRG, INAV_Cmds, "iNAV")
PROTODEF(PROTOCOL_Q303, NRF24L01, AETRG, Q303_Cmds, "Q303")
PROTODEF(PROTOCOL_GW008, NRF24L01, AETRG, GW008_Cmds, "GW008")
PROTODEF(PROTOCOL_DM002, NRF24L01, AETRG, DM002_Cmds, "DM002")
PROTODEF(PROTOCOL_BUGS3MINI, NRF24L01, AETRG, BUGS3MINI_Cmds, "Bugs3Mini")
PROTODEF(PROTOCOL_E012, NRF24L01, AETRG, E012_Cmds, "E012")
PROTODEF(PROTOCOL_E015, NRF24L01, AETRG, E015_Cmds, "E015")
PROTODEF(PROTOCOL_NCC1701, NRF24L01, AETRG, NCC1701_Cmds, "NCC1701")
PROTODEF(PROTOCOL_V911S, NRF24L01, AETRG, V911S_Cmds, "V911S")
PROTODEF(PROTOCOL_GD00X, NRF24L01, AETRG, GD00X_Cmds, "GD00X")
PROTODEF(PROTOCOL_LOLI, NRF24L01, AETRG, LOLI_Cmds, "LOLI")
PROTODEF(PROTOCOL_E016H, NRF24L01, AETRG, E016H_Cmds, "E016H")
PROTODEF(PROTOCOL_PROPEL, NRF24L01, AETRG, Propel_Cmds, "Propel")
#endif //PROTO_HAS_NRF24L01

PROTODEF(PROTOCOL_PPM, TX_MODULE_LAST,  NULL, PPMOUT_Cmds, "PPM")
PROTODEF(PROTOCOL_USBHID, TX_MODULE_LAST,  NULL, USBHID_Cmds, "USBHID")

PROTODEF(PROTOCOL_CRSF, TX_MODULE_LAST,  NULL, CRSF_Cmds, "CRSF")
PROTODEF(PROTOCOL_SBUS, TX_MODULE_LAST,  NULL, SBUS_Cmds, "SBUS")
PROTODEF(PROTOCOL_SUMD, TX_MODULE_LAST,  NULL, SUMD_Cmds, "SUMD")
PROTODEF(PROTOCOL_PXX, R9M,  NULL, PXXOUT_Cmds, "PXX")

#ifdef BUILDTYPE_DEV
PROTODEF(PROTOCOL_TESTSER, TX_MODULE_LAST,  NULL, TESTSER_Cmds, "TESTSER")
PROTODEF(PROTOCOL_TESTRF, TX_MODULE_LAST,  NULL, TESTRF_Cmds, "TESTRF")
#endif
#if SUPPORT_XN297DUMP
PROTODEF(PROTOCOL_XN297DUMP, NRF24L01, NULL, XN297Dump_Cmds, "XN297Dump")
#endif
#if SUPPORT_SCANNER
PROTODEF(PROTOCOL_SCANNER_CYRF, CYRF6936, NULL, SCANNER_CYRF_Cmds, "Scan CYRF")
#endif  // SUPPORT_SCANNER
#endif //PROTODEF
