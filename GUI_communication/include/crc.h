#ifndef CRCCOMPUTE_H
#define CRCCOMPUTE_H

#include <stdint.h>
#include <stdio.h>
#include <iostream>

template <typename TYPE>
class CRC {
 public:
  CRC();
  CRC(TYPE polynomial, TYPE init_remainder, TYPE final_xor_value);
  void build(TYPE polynomial, TYPE init_remainder, TYPE final_xor_value);
  /**
   * Compute the CRC checksum of a binary message block.
   * @para message, 用来计算的数据
   * @para nBytes, 数据的长度
   */
  TYPE crcCompute(const char* message, unsigned int nBytes);
  TYPE crcCompute(const char* message, unsigned int nBytes, bool reinit);

 protected:
  TYPE m_polynomial;
  TYPE m_initial_remainder;
  TYPE m_final_xor_value;
  TYPE m_remainder;
  TYPE crcTable[256];
  int m_width;
  int m_topbit;
  /**
   * Initialize the CRC lookup table.
   * This table is used by crcCompute() to make CRC computation faster.
   */
  void crcInit(void) {
    TYPE remainder;
    TYPE dividend;
    int bit;
    /* Perform binary long division, a bit at a time. */
    for (dividend = 0; dividend < 256; dividend++) {
      /* Initialize the remainder.  */
      remainder = dividend << (m_width - 8);
      /* Shift and XOR with the polynomial.   */
      for (bit = 0; bit < 8; bit++) {
        /* Try to divide the current data bit.  */
        if (remainder & m_topbit) {
          remainder = (remainder << 1) ^ m_polynomial;
        } else {
          remainder = remainder << 1;
        }
      }
      /* Save the result in the table. */
      crcTable[dividend] = remainder;
    }
  }
};

template <typename TYPE>
CRC<TYPE>::CRC() {
  m_width = 8 * sizeof(TYPE);
  m_topbit = 1 << (m_width - 1);
}

template <typename TYPE>
CRC<TYPE>::CRC(TYPE polynomial, TYPE init_remainder, TYPE final_xor_value) {
  m_width = 8 * sizeof(TYPE);
  m_topbit = 1 << (m_width - 1);
  m_polynomial = polynomial;
  m_initial_remainder = init_remainder;
  m_final_xor_value = final_xor_value;

  crcInit();
}

template <typename TYPE>
void CRC<TYPE>::build(TYPE polynomial, TYPE init_remainder,
                      TYPE final_xor_value) {
  m_polynomial = polynomial;
  m_initial_remainder = init_remainder;
  m_final_xor_value = final_xor_value;

  crcInit();
}

template <typename TYPE>
TYPE CRC<TYPE>::crcCompute(const char* message, unsigned int nBytes) {
  unsigned int offset;
  unsigned char byte;
  TYPE remainder = m_initial_remainder;
  /* Divide the message by the polynomial, a byte at a time. */
  for (offset = 0; offset < nBytes; offset++) {
    byte = (remainder >> (m_width - 8)) ^ message[offset];
    remainder = crcTable[byte] ^ (remainder << 8);
  }
  /* The final remainder is the CRC result. */
  return (remainder ^ m_final_xor_value);
}

template <typename TYPE>
TYPE CRC<TYPE>::crcCompute(const char* message, unsigned int nBytes,
                           bool reinit) {
  unsigned int offset;
  unsigned char byte;
  if (reinit) {
    m_remainder = m_initial_remainder;
  }
  /* Divide the message by the polynomial, a byte at a time. */
  for (offset = 0; offset < nBytes; offset++) {
    byte = (m_remainder >> (m_width - 8)) ^ message[offset];
    m_remainder = crcTable[byte] ^ (m_remainder << 8);
  }
  /* The final remainder is the CRC result. */
  return (m_remainder ^ m_final_xor_value);
}

class CRC8 : public CRC<uint8_t> {
 public:
  enum CRC8_TYPE {
    eCRC8,
    eAUTOSAR,
    eCDMA2000,
    eDARC,
    eDVB_S2,
    eEBU,
    eAES,
    eGSM_A,
    eGSM_B,
    eI_CODE,
    eITU,
    eLTE,
    eMAXIM,
    eOPENSAFETY,
    eROHC,
    eSAE_J1850,
    eWCDMA
  };
  CRC8(CRC8_TYPE type) {
    switch (type) {
      case eCRC8:
        m_polynomial =
            0x07;  // http://reveng.sourceforge.net/crc-catalogue/all.htm
        m_initial_remainder = 0x00;
        m_final_xor_value = 0x00;
        break;
      case eAUTOSAR:
        m_polynomial = 0x2f;
        m_initial_remainder = 0xff;
        m_final_xor_value = 0xff;
        break;
      case eCDMA2000:
        m_polynomial = 0x9b;
        m_initial_remainder = 0xFF;
        m_final_xor_value = 0x00;
        break;
      case eDARC:
        m_polynomial = 0x39;
        m_initial_remainder = 0x00;
        m_final_xor_value = 0x00;
        break;
      case eDVB_S2:
        m_polynomial = 0xd5;
        m_initial_remainder = 0x00;
        m_final_xor_value = 0x00;
        break;
      case eEBU:
      case eAES:
        m_polynomial = 0x1d;
        m_initial_remainder = 0xFF;
        m_final_xor_value = 0x00;
        break;
      case eGSM_A:
        m_polynomial = 0x1d;
        m_initial_remainder = 0x00;
        m_final_xor_value = 0x00;
        break;
      case eGSM_B:
        m_polynomial = 0x49;
        m_initial_remainder = 0x00;
        m_final_xor_value = 0xFF;
        break;
      case eI_CODE:
        m_polynomial = 0x1d;
        m_initial_remainder = 0xFD;
        m_final_xor_value = 0x00;
        break;
      case eITU:
        m_polynomial = 0x07;
        m_initial_remainder = 0x00;
        m_final_xor_value = 0x55;
        break;
      case eLTE:
        m_polynomial = 0x9b;
        m_initial_remainder = 0x00;
        m_final_xor_value = 0x00;
        break;
      case eMAXIM:
        m_polynomial = 0x31;
        m_initial_remainder = 0x00;
        m_final_xor_value = 0x00;
        break;
      case eOPENSAFETY:
        m_polynomial = 0x2f;
        m_initial_remainder = 0x00;
        m_final_xor_value = 0x00;
        break;
      case eROHC:
        m_polynomial = 0x07;
        m_initial_remainder = 0xff;
        m_final_xor_value = 0x00;
        break;
      case eSAE_J1850:
        m_polynomial = 0x1d;
        m_initial_remainder = 0xff;
        m_final_xor_value = 0xff;
        break;
      case eWCDMA:
        m_polynomial = 0x9b;
        m_initial_remainder = 0x00;
        m_final_xor_value = 0x00;
        break;
      default:
        m_polynomial = 0x07;
        m_initial_remainder = 0x00;
        m_final_xor_value = 0x00;
        break;
    }
    crcInit();
  }
  CRC8(uint8_t polynomial, uint8_t init_remainder, uint8_t final_xor_value)
      : CRC<uint8_t>(polynomial, init_remainder, final_xor_value) {}
};

class CRC16 : public CRC<uint16_t> {
 public:
  enum CRC16_TYPE {
    eCCITT,
    eKERMIT,
    eCCITT_FALSE,
    eIBM,
    eARC,
    eLHA,
    eSPI_FUJITSU,
    eBUYPASS,
    eVERIFONE,
    eUMTS,
    eCDMA2000,
    eCMS,
    eDDS_110,
    eDECT_R,
    eDECT_X,
    eDNP,
    eEN_13757,
    eGENIBUS,
    eEPC,
    eDARC,
    eI_CODE,
    eGSM,
    eLJ1200,
    eMAXIM,
    eMCRF4XX,
    eOPENSAFETY_A,
    eOPENSAFETY_B,
    ePROFIBUS,
    eIEC_61158_2,
    eRIELLO,
    eT10_DIF,
    eTELEDISK,
    eTMS37157,
    eUSB,
    eCRC_A,
    eMODBUS,
    eX_25,
    eCRC_B,
    eISO_HDLC,
    eIBM_SDLC,
    eXMODEM,
    eZMODEM,
    eACORN,
    eLTE
  };
  CRC16(CRC16_TYPE type) {
    switch (type) {
      case eCCITT_FALSE:
      case eMCRF4XX:
        m_polynomial = 0x1021;
        m_initial_remainder = 0xFFFF;
        m_final_xor_value = 0x0000;
        break;
      case eIBM:
      case eARC:
      case eLHA:
      case eBUYPASS:
      case eVERIFONE:
      case eUMTS:
        m_polynomial = 0x8005;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0x0000;
        break;
      case eSPI_FUJITSU:
        m_polynomial = 0x1021;
        m_initial_remainder = 0x1d0f;
        m_final_xor_value = 0x0000;
        break;
      case eCCITT:
      case eKERMIT:
      case eXMODEM:
      case eZMODEM:
      case eACORN:
      case eLTE:
        m_polynomial = 0x1021;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0x0000;
        break;
      case eCDMA2000:
        m_polynomial = 0xc867;
        m_initial_remainder = 0xffff;
        m_final_xor_value = 0x0000;
        break;
      case eCMS:
      case eMODBUS:
        m_polynomial = 0x8005;
        m_initial_remainder = 0xffff;
        m_final_xor_value = 0x0000;
        break;
      case eDDS_110:
        m_polynomial = 0x8005;
        m_initial_remainder = 0x800d;
        m_final_xor_value = 0x0000;
        break;
      case eDECT_R:
        m_polynomial = 0x0589;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0x0001;
        break;
      case eDECT_X:
        m_polynomial = 0x0589;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0x0000;
        break;
      case eDNP:
      case eEN_13757:
        m_polynomial = 0x3d65;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0xffff;
        break;
      case eGENIBUS:
      case eEPC:
      case eDARC:
      case eI_CODE:
      case eX_25:
      case eCRC_B:
      case eISO_HDLC:
      case eIBM_SDLC:
        m_polynomial = 0x1021;
        m_initial_remainder = 0xffff;
        m_final_xor_value = 0xffff;
        break;
      case eGSM:
        m_polynomial = 0x1021;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0xffff;
        break;
      case eLJ1200:
        m_polynomial = 0x6f63;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0x0000;
        break;
      case eMAXIM:
        m_polynomial = 0x8005;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0xffff;
        break;
      case eOPENSAFETY_A:
        m_polynomial = 0x5935;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0x0000;
        break;
      case eOPENSAFETY_B:
        m_polynomial = 0x755b;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0x0000;
        break;
      case ePROFIBUS:
      case eIEC_61158_2:
        m_polynomial = 0x1dcf;
        m_initial_remainder = 0xffff;
        m_final_xor_value = 0xffff;
        break;
      case eRIELLO:
        m_polynomial = 0x1021;
        m_initial_remainder = 0xb2aa;
        m_final_xor_value = 0x0000;
        break;
      case eT10_DIF:
        m_polynomial = 0x8bb7;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0x0000;
        break;
      case eTELEDISK:
        m_polynomial = 0xa097;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0x0000;
        break;
      case eTMS37157:
        m_polynomial = 0x1021;
        m_initial_remainder = 0x89ec;
        m_final_xor_value = 0x0000;
        break;
      case eUSB:
        m_polynomial = 0x8005;
        m_initial_remainder = 0xffff;
        m_final_xor_value = 0xffff;
        break;
      case eCRC_A:
        m_polynomial = 0x1021;
        m_initial_remainder = 0xc6c6;
        m_final_xor_value = 0x0000;
        break;
      default:
        m_polynomial = 0x8005;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0x0000;
        break;
    }
    crcInit();
  }
  CRC16(uint16_t polynomial, uint16_t init_remainder, uint16_t final_xor_value)
      : CRC<uint16_t>(polynomial, init_remainder, final_xor_value) {}
};

class CRC32 : public CRC<uint32_t> {
 public:
  enum CRC32_TYPE {
    eADCCP,
    ePKZIP,
    eCRC32,
    eAAL5,
    eDECT_B,
    eB_CRC32,
    eBZIP2,
    eAUTOSAR,
    eCRC32C,
    eCRC32D,
    eMPEG2,
    ePOSIX,
    eCKSUM,
    eCRC32Q,
    eJAMCRC,
    eXFER
  };
  CRC32(CRC32_TYPE type) {
    switch (type) {
      case eADCCP:
      case ePKZIP:
      case eCRC32:
      case eBZIP2:
      case eAAL5:
      case eDECT_B:
      case eB_CRC32:
        m_polynomial = 0x04c11db7;
        m_initial_remainder = 0xFFFFFFFF;
        m_final_xor_value = 0xFFFFFFFF;
        break;
      case eAUTOSAR:
        m_polynomial = 0xf4acfb13;
        m_initial_remainder = 0xFFFFFFFF;
        m_final_xor_value = 0xFFFFFFFF;
        break;
      case eCRC32C:
        m_polynomial = 0x1edc6f41;
        m_initial_remainder = 0xFFFFFFFF;
        m_final_xor_value = 0xFFFFFFFF;
        break;
      case eCRC32D:
        m_polynomial = 0xa833982b;
        m_initial_remainder = 0xFFFFFFFF;
        m_final_xor_value = 0xFFFFFFFF;
        break;
      case eMPEG2:
      case eJAMCRC:
        m_polynomial = 0x04c11db7;
        m_initial_remainder = 0xFFFFFFFF;
        m_final_xor_value = 0x00000000;
        break;
      case ePOSIX:
      case eCKSUM:
        m_polynomial = 0x04c11db7;
        m_initial_remainder = 0x00000000;
        m_final_xor_value = 0xFFFFFFFF;
        break;
      case eCRC32Q:
        m_polynomial = 0x814141ab;
        m_initial_remainder = 0x00000000;
        m_final_xor_value = 0x00000000;
        break;
      case eXFER:
        m_polynomial = 0x000000af;
        m_initial_remainder = 0x00000000;
        m_final_xor_value = 0x00000000;
        break;
      default:
        m_polynomial = 0x04C11DB7;
        m_initial_remainder = 0xFFFFFFFF;
        m_final_xor_value = 0xFFFFFFFF;
        break;
    }
    crcInit();
  }
};

#endif // CRCCOMPUTE_H