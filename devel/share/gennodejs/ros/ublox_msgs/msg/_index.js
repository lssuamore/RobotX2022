
"use strict";

let NavSBAS = require('./NavSBAS.js');
let RxmSVSI = require('./RxmSVSI.js');
let NavTIMEUTC = require('./NavTIMEUTC.js');
let NavDGPS = require('./NavDGPS.js');
let CfgRATE = require('./CfgRATE.js');
let CfgCFG = require('./CfgCFG.js');
let RxmSFRB = require('./RxmSFRB.js');
let MonGNSS = require('./MonGNSS.js');
let NavRELPOSNED9 = require('./NavRELPOSNED9.js');
let NavSOL = require('./NavSOL.js');
let EsfRAW_Block = require('./EsfRAW_Block.js');
let EsfINS = require('./EsfINS.js');
let CfgNMEA7 = require('./CfgNMEA7.js');
let NavPVT = require('./NavPVT.js');
let CfgNAVX5 = require('./CfgNAVX5.js');
let EsfMEAS = require('./EsfMEAS.js');
let NavSVINFO = require('./NavSVINFO.js');
let NavPVT7 = require('./NavPVT7.js');
let CfgPRT = require('./CfgPRT.js');
let RxmSFRBX = require('./RxmSFRBX.js');
let NavSTATUS = require('./NavSTATUS.js');
let NavTIMEGPS = require('./NavTIMEGPS.js');
let CfgINF_Block = require('./CfgINF_Block.js');
let AidALM = require('./AidALM.js');
let MonVER = require('./MonVER.js');
let RxmRTCM = require('./RxmRTCM.js');
let EsfSTATUS_Sens = require('./EsfSTATUS_Sens.js');
let Ack = require('./Ack.js');
let NavSBAS_SV = require('./NavSBAS_SV.js');
let CfgNMEA6 = require('./CfgNMEA6.js');
let NavATT = require('./NavATT.js');
let CfgMSG = require('./CfgMSG.js');
let NavCLOCK = require('./NavCLOCK.js');
let NavPOSECEF = require('./NavPOSECEF.js');
let MonHW6 = require('./MonHW6.js');
let CfgDAT = require('./CfgDAT.js');
let Inf = require('./Inf.js');
let AidHUI = require('./AidHUI.js');
let CfgSBAS = require('./CfgSBAS.js');
let NavSAT = require('./NavSAT.js');
let NavPOSLLH = require('./NavPOSLLH.js');
let CfgTMODE3 = require('./CfgTMODE3.js');
let CfgDGNSS = require('./CfgDGNSS.js');
let CfgRST = require('./CfgRST.js');
let CfgANT = require('./CfgANT.js');
let UpdSOS = require('./UpdSOS.js');
let RxmALM = require('./RxmALM.js');
let TimTM2 = require('./TimTM2.js');
let CfgINF = require('./CfgINF.js');
let RxmRAWX = require('./RxmRAWX.js');
let MonVER_Extension = require('./MonVER_Extension.js');
let RxmRAWX_Meas = require('./RxmRAWX_Meas.js');
let RxmRAW_SV = require('./RxmRAW_SV.js');
let NavDGPS_SV = require('./NavDGPS_SV.js');
let NavVELECEF = require('./NavVELECEF.js');
let CfgNMEA = require('./CfgNMEA.js');
let RxmEPH = require('./RxmEPH.js');
let NavSVINFO_SV = require('./NavSVINFO_SV.js');
let NavHPPOSECEF = require('./NavHPPOSECEF.js');
let NavSVIN = require('./NavSVIN.js');
let EsfRAW = require('./EsfRAW.js');
let NavVELNED = require('./NavVELNED.js');
let CfgHNR = require('./CfgHNR.js');
let CfgNAV5 = require('./CfgNAV5.js');
let NavHPPOSLLH = require('./NavHPPOSLLH.js');
let MgaGAL = require('./MgaGAL.js');
let RxmSVSI_SV = require('./RxmSVSI_SV.js');
let EsfSTATUS = require('./EsfSTATUS.js');
let NavSAT_SV = require('./NavSAT_SV.js');
let CfgUSB = require('./CfgUSB.js');
let CfgGNSS = require('./CfgGNSS.js');
let RxmRAW = require('./RxmRAW.js');
let AidEPH = require('./AidEPH.js');
let NavRELPOSNED = require('./NavRELPOSNED.js');
let MonHW = require('./MonHW.js');
let CfgGNSS_Block = require('./CfgGNSS_Block.js');
let NavDOP = require('./NavDOP.js');
let HnrPVT = require('./HnrPVT.js');
let UpdSOS_Ack = require('./UpdSOS_Ack.js');

module.exports = {
  NavSBAS: NavSBAS,
  RxmSVSI: RxmSVSI,
  NavTIMEUTC: NavTIMEUTC,
  NavDGPS: NavDGPS,
  CfgRATE: CfgRATE,
  CfgCFG: CfgCFG,
  RxmSFRB: RxmSFRB,
  MonGNSS: MonGNSS,
  NavRELPOSNED9: NavRELPOSNED9,
  NavSOL: NavSOL,
  EsfRAW_Block: EsfRAW_Block,
  EsfINS: EsfINS,
  CfgNMEA7: CfgNMEA7,
  NavPVT: NavPVT,
  CfgNAVX5: CfgNAVX5,
  EsfMEAS: EsfMEAS,
  NavSVINFO: NavSVINFO,
  NavPVT7: NavPVT7,
  CfgPRT: CfgPRT,
  RxmSFRBX: RxmSFRBX,
  NavSTATUS: NavSTATUS,
  NavTIMEGPS: NavTIMEGPS,
  CfgINF_Block: CfgINF_Block,
  AidALM: AidALM,
  MonVER: MonVER,
  RxmRTCM: RxmRTCM,
  EsfSTATUS_Sens: EsfSTATUS_Sens,
  Ack: Ack,
  NavSBAS_SV: NavSBAS_SV,
  CfgNMEA6: CfgNMEA6,
  NavATT: NavATT,
  CfgMSG: CfgMSG,
  NavCLOCK: NavCLOCK,
  NavPOSECEF: NavPOSECEF,
  MonHW6: MonHW6,
  CfgDAT: CfgDAT,
  Inf: Inf,
  AidHUI: AidHUI,
  CfgSBAS: CfgSBAS,
  NavSAT: NavSAT,
  NavPOSLLH: NavPOSLLH,
  CfgTMODE3: CfgTMODE3,
  CfgDGNSS: CfgDGNSS,
  CfgRST: CfgRST,
  CfgANT: CfgANT,
  UpdSOS: UpdSOS,
  RxmALM: RxmALM,
  TimTM2: TimTM2,
  CfgINF: CfgINF,
  RxmRAWX: RxmRAWX,
  MonVER_Extension: MonVER_Extension,
  RxmRAWX_Meas: RxmRAWX_Meas,
  RxmRAW_SV: RxmRAW_SV,
  NavDGPS_SV: NavDGPS_SV,
  NavVELECEF: NavVELECEF,
  CfgNMEA: CfgNMEA,
  RxmEPH: RxmEPH,
  NavSVINFO_SV: NavSVINFO_SV,
  NavHPPOSECEF: NavHPPOSECEF,
  NavSVIN: NavSVIN,
  EsfRAW: EsfRAW,
  NavVELNED: NavVELNED,
  CfgHNR: CfgHNR,
  CfgNAV5: CfgNAV5,
  NavHPPOSLLH: NavHPPOSLLH,
  MgaGAL: MgaGAL,
  RxmSVSI_SV: RxmSVSI_SV,
  EsfSTATUS: EsfSTATUS,
  NavSAT_SV: NavSAT_SV,
  CfgUSB: CfgUSB,
  CfgGNSS: CfgGNSS,
  RxmRAW: RxmRAW,
  AidEPH: AidEPH,
  NavRELPOSNED: NavRELPOSNED,
  MonHW: MonHW,
  CfgGNSS_Block: CfgGNSS_Block,
  NavDOP: NavDOP,
  HnrPVT: HnrPVT,
  UpdSOS_Ack: UpdSOS_Ack,
};
