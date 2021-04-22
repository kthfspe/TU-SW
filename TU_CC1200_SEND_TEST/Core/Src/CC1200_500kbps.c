#include "main.h"
void CC1200_500kbps_settings() {
	//
	// Rf settings for CC1200
	//
	halRfWriteReg(IOCFG2,0x06);        //GPIO2 IO Pin Configuration
	halRfWriteReg(SYNC_CFG1,0xA8);     //Sync Word Detection Configuration Reg. 1
	halRfWriteReg(SYNC_CFG0,0x13);     //Sync Word Detection Configuration Reg. 0
	halRfWriteReg(DEVIATION_M,0x99);   //Frequency Deviation Configuration
	halRfWriteReg(MODCFG_DEV_E,0x0D);  //Modulation Format and Frequency Deviation Configur..
	halRfWriteReg(DCFILT_CFG,0x26);    //Digital DC Removal Configuration
	halRfWriteReg(PREAMBLE_CFG0,0x8A); //Preamble Detection Configuration Reg. 0
	halRfWriteReg(IQIC,0x00);          //Digital Image Channel Compensation Configuration
	halRfWriteReg(CHAN_BW,0x02);       //Channel Filter Configuration
	halRfWriteReg(MDMCFG1,0x40);       //General Modem Parameter Configuration Reg. 1
	halRfWriteReg(MDMCFG0,0x05);       //General Modem Parameter Configuration Reg. 0
	halRfWriteReg(SYMBOL_RATE2,0xC9);  //Symbol Rate Configuration Exponent and Mantissa [1..
	halRfWriteReg(SYMBOL_RATE1,0x99);  //Symbol Rate Configuration Mantissa [15:8]
	halRfWriteReg(SYMBOL_RATE0,0x99);  //Symbol Rate Configuration Mantissa [7:0]
	halRfWriteReg(AGC_REF,0x2F);       //AGC Reference Level Configuration
	halRfWriteReg(AGC_CS_THR,0xEC);    //Carrier Sense Threshold Configuration
	halRfWriteReg(AGC_CFG1,0x16);      //Automatic Gain Control Configuration Reg. 1
	halRfWriteReg(AGC_CFG0,0x84);      //Automatic Gain Control Configuration Reg. 0
	halRfWriteReg(FIFO_CFG,0x00);      //FIFO Configuration
	halRfWriteReg(FS_CFG,0x12);        //Frequency Synthesizer Configuration
	halRfWriteReg(PKT_CFG2,0x00);      //Packet Configuration Reg. 2
	halRfWriteReg(PKT_CFG1,0x43);      //Packet Configuration Reg. 1
	halRfWriteReg(PKT_CFG0,0x20);      //Packet Configuration Reg. 0
	halRfWriteReg(PA_CFG1,0x6A);       //Power Amplifier Configuration Reg. 1
	halRfWriteReg(ASK_CFG,0x3F);       //ASK Configuration
	halRfWriteReg(PKT_LEN,0xFF);       //Packet Length Configuration
	halRfWriteReg(IF_MIX_CFG,0x18);    //IF Mix Configuration
	halRfWriteReg(FREQOFF_CFG,0x22);   //Frequency Offset Correction Configuration
	halRfWriteReg(TOC_CFG,0x03);       //Timing Offset Correction Configuration
	halRfWriteReg(MDMCFG2,0x00);       //General Modem Parameter Configuration Reg. 2
	halRfWriteReg(FREQ2,0x57);         //Frequency Configuration [23:16]
	halRfWriteReg(FREQ1,0x0F);         //Frequency Configuration [15:8]
	halRfWriteReg(FREQ0,0x5C);         //Frequency Configuration [7:0]
	halRfWriteReg(IF_ADC1,0xEE);       //Analog to Digital Converter Configuration Reg. 1
	halRfWriteReg(IF_ADC0,0x10);       //Analog to Digital Converter Configuration Reg. 0
	halRfWriteReg(FS_DIG1,0x04);       //Frequency Synthesizer Digital Reg. 1
	halRfWriteReg(FS_DIG0,0x50);       //Frequency Synthesizer Digital Reg. 0
	halRfWriteReg(FS_CAL1,0x40);       //Frequency Synthesizer Calibration Reg. 1
	halRfWriteReg(FS_CAL0,0x0E);       //Frequency Synthesizer Calibration Reg. 0
	halRfWriteReg(FS_DIVTWO,0x03);     //Frequency Synthesizer Divide by 2
	halRfWriteReg(FS_DSM0,0x33);       //FS Digital Synthesizer Module Configuration Reg. 0
	halRfWriteReg(FS_DVC1,0xF7);       //Frequency Synthesizer Divider Chain Configuration ..
	halRfWriteReg(FS_DVC0,0x0F);       //Frequency Synthesizer Divider Chain Configuration ..
	halRfWriteReg(FS_PFD,0x00);        //Frequency Synthesizer Phase Frequency Detector Con..
	halRfWriteReg(FS_PRE,0x6E);        //Frequency Synthesizer Prescaler Configuration
	halRfWriteReg(FS_REG_DIV_CML,0x1C);//Frequency Synthesizer Divider Regulator Configurat..
	halRfWriteReg(FS_SPARE,0xAC);      //Frequency Synthesizer Spare
	halRfWriteReg(FS_VCO0,0xB5);       //FS Voltage Controlled Oscillator Configuration Reg..
	halRfWriteReg(IFAMP,0x0D);         //Intermediate Frequency Amplifier Configuration
	halRfWriteReg(XOSC5,0x0E);         //Crystal Oscillator Configuration Reg. 5
	halRfWriteReg(XOSC1,0x03);         //Crystal Oscillator Configuration Reg. 1
	halRfWriteReg(PARTNUMBER,0x20);    //Part Number
	halRfWriteReg(PARTVERSION,0x11);   //Part Revision
	halRfWriteReg(MODEM_STATUS1,0x10); //Modem Status Reg. 1

}
