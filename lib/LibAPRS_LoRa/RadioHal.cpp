#include "RadioHal.hpp"

template<>
int16_t RadioHal<SX1272>::begin(float freq, float bw, uint8_t sf, uint8_t cr, uint8_t syncWord, int8_t power, uint16_t preambleLength, uint8_t gain, float tcxoVoltage)
{   
    if (power>=17) radio->setCurrentLimit(150);
    return radio->begin(freq, bw, sf, cr, syncWord, power, preambleLength, gain);
}
template<>
int16_t RadioHal<SX1272>::begin()
{
    return radio->begin();
}

template<>
int16_t RadioHal<SX1273>::begin(float freq, float bw, uint8_t sf, uint8_t cr, uint8_t syncWord, int8_t power, uint16_t preambleLength, uint8_t gain, float tcxoVoltage)
{   
    if (power>=17) radio->setCurrentLimit(150);
    return radio->begin(freq, bw, sf, cr, syncWord, power, preambleLength, gain);
}
template<>
int16_t RadioHal<SX1273>::begin()
{
    return radio->begin();
}

template<>
int16_t RadioHal<SX1276>::begin(float freq, float bw, uint8_t sf, uint8_t cr, uint8_t syncWord, int8_t power, uint16_t preambleLength, uint8_t gain, float tcxoVoltage)
{   
    if (power>=17) radio->setCurrentLimit(150);
    return radio->begin(freq, bw, sf, cr, syncWord, power, preambleLength, gain);
}
template<>
int16_t RadioHal<SX1276>::begin()
{
    return radio->begin();
}

template<>
int16_t RadioHal<SX1278>::begin(float freq, float bw, uint8_t sf, uint8_t cr, uint8_t syncWord, int8_t power, uint16_t preambleLength, uint8_t gain, float tcxoVoltage)
{ 
    
    if (power>=17) radio->setCurrentLimit(150);
    return radio->begin(freq, bw, sf, cr, syncWord, power, preambleLength, gain);
}
template<>
int16_t RadioHal<SX1278>::begin()
{
    return radio->begin();
}

template<>
int16_t RadioHal<SX1279>::begin(float freq, float bw, uint8_t sf, uint8_t cr, uint8_t syncWord, int8_t power, uint16_t preambleLength, uint8_t gain, float tcxoVoltage)
{   
    if (power>=17) radio->setCurrentLimit(150);
    return radio->begin(freq, bw, sf, cr, syncWord, power, preambleLength, gain);
}
template<>
int16_t RadioHal<SX1279>::begin()
{
    return radio->begin();
}


template<>
int16_t RadioHal<SX1268>::begin(float freq, float bw, uint8_t sf, uint8_t cr, uint8_t syncWord, int8_t power, uint16_t preambleLength, uint8_t gain, float tcxoVoltage)
{
    if (power>=17) radio->setCurrentLimit(150);
    return radio->begin(freq, bw, sf, cr, syncWord, power, preambleLength, tcxoVoltage);
}
template<>
int16_t RadioHal<SX1268>::begin()
{
    return radio->begin();
}

template<>
int16_t RadioHal<SX1262>::begin(float freq, float bw, uint8_t sf, uint8_t cr, uint8_t syncWord, int8_t power, uint16_t preambleLength, uint8_t gain, float tcxoVoltage)
{
    if (power>=17) radio->setCurrentLimit(150);
    return radio->begin(freq, bw, sf, cr, syncWord, power, preambleLength, tcxoVoltage);
}
template<>
int16_t RadioHal<SX1262>::begin()
{
    return radio->begin();
}

template<>
int16_t RadioHal<SX1231>::beginFSK(float freq, float br, float freqDev, float rxBw, int8_t power, uint16_t preambleLength, bool enableOOK, float tcxoVoltage, bool useRegulatorLDO)
{
    return radio->begin(freq, br, freqDev, rxBw, power, preambleLength);
}

template<>
int16_t RadioHal<SX1233>::beginFSK(float freq, float br, float freqDev, float rxBw, int8_t power, uint16_t preambleLength, bool enableOOK, float tcxoVoltage, bool useRegulatorLDO)
{
    return radio->begin(freq, br, freqDev, rxBw, power, preambleLength);
}

template<>
int16_t RadioHal<SX1261>::begin(float freq, float bw, uint8_t sf, uint8_t cr, uint8_t syncWord, int8_t power, uint16_t preambleLength, uint8_t gain, float tcxoVoltage)
{
    if (power>=17) radio->setCurrentLimit(150);
    return radio->begin(freq, bw, sf, cr, syncWord, power, preambleLength, tcxoVoltage);
}
template<>
int16_t RadioHal<SX1261>::begin()
{
    return radio->begin();
}


template<>
int16_t RadioHal<SX1280>::begin(float freq, float bw, uint8_t sf, uint8_t cr, uint8_t syncWord, int8_t power, uint16_t preambleLength, uint8_t gain, float tcxoVoltage)
{
    return radio->begin(freq, bw, sf, cr, syncWord, power, preambleLength);
}
template<>
int16_t RadioHal<SX1280>::begin()
{
    return radio->begin();
}

template<>
int16_t RadioHal<SX1281>::begin(float freq, float bw, uint8_t sf, uint8_t cr, uint8_t syncWord, int8_t power, uint16_t preambleLength, uint8_t gain, float tcxoVoltage)
{
    return radio->begin(freq, bw, sf, cr, syncWord, power, preambleLength);
}
template<>
int16_t RadioHal<SX1281>::begin()
{
    return radio->begin();
}

template<>
int16_t RadioHal<SX1282>::begin(float freq, float bw, uint8_t sf, uint8_t cr, uint8_t syncWord, int8_t power, uint16_t preambleLength, uint8_t gain, float tcxoVoltage)
{
    return radio->begin(freq, bw, sf, cr, syncWord, power, preambleLength);
}
template<>
int16_t RadioHal<SX1282>::begin()
{
    return radio->begin();
}

template<>
int16_t RadioHal<SX1279>::beginFSK(float freq, float br, float freqDev, float rxBw, int8_t power, uint16_t preambleLength, bool enableOOK, float tcxoVoltage, bool useRegulatorLDO)
{
    if (power>=17) radio->setCurrentLimit(150);
    return radio->beginFSK(freq, br, freqDev, rxBw, power, preambleLength, enableOOK);
}

template<>
int16_t RadioHal<SX1278>::beginFSK(float freq, float br, float freqDev, float rxBw, int8_t power, uint16_t preambleLength, bool enableOOK, float tcxoVoltage, bool useRegulatorLDO)
{
    if (power>=17) radio->setCurrentLimit(150);
    return radio->beginFSK(freq, br, freqDev, rxBw, power, preambleLength, enableOOK);
}

template<>
int16_t RadioHal<SX1276>::beginFSK(float freq, float br, float freqDev, float rxBw, int8_t power, uint16_t preambleLength, bool enableOOK, float tcxoVoltage, bool useRegulatorLDO)
{
    if (power>=17) radio->setCurrentLimit(150);
    return radio->beginFSK(freq, br, freqDev, rxBw, power, preambleLength, enableOOK);
}

template<>
int16_t RadioHal<SX1272>::beginFSK(float freq, float br, float freqDev, float rxBw, int8_t power, uint16_t preambleLength, bool enableOOK, float tcxoVoltage, bool useRegulatorLDO)
{
    if (power>=17) radio->setCurrentLimit(150);
    return radio->beginFSK(freq, br, freqDev, rxBw, power, preambleLength, enableOOK);
}

template<>
int16_t RadioHal<SX1273>::beginFSK(float freq, float br, float freqDev, float rxBw, int8_t power, uint16_t preambleLength, bool enableOOK, float tcxoVoltage, bool useRegulatorLDO)
{
    if (power>=17) radio->setCurrentLimit(150);
    return radio->beginFSK(freq, br, freqDev, rxBw, power, preambleLength, enableOOK);
}

template<>
int16_t RadioHal<SX1268>::beginFSK(float freq, float br, float freqDev, float rxBw, int8_t power, uint16_t preambleLength, bool enableOOK, float tcxoVoltage, bool useRegulatorLDO)
{
    if (power>=17) radio->setCurrentLimit(150);
    return radio->beginFSK(freq, br, freqDev, rxBw, power, preambleLength, tcxoVoltage, useRegulatorLDO);
}

template<>
int16_t RadioHal<SX1262>::beginFSK(float freq, float br, float freqDev, float rxBw, int8_t power, uint16_t preambleLength, bool enableOOK, float tcxoVoltage, bool useRegulatorLDO)
{
    if (power>=17) radio->setCurrentLimit(150);
    return radio->beginFSK(freq, br, freqDev, rxBw, power, preambleLength, tcxoVoltage, useRegulatorLDO);
}

template<>
int16_t RadioHal<SX1261>::beginFSK(float freq, float br, float freqDev, float rxBw, int8_t power, uint16_t preambleLength, bool enableOOK, float tcxoVoltage, bool useRegulatorLDO)
{
    if (power>=17) radio->setCurrentLimit(150);
    return radio->beginFSK(freq, br, freqDev, rxBw, power, preambleLength, tcxoVoltage, useRegulatorLDO);
}

template<>
int16_t RadioHal<SX1280>::beginFSK(float freq, float br, float freqDev, float rxBw, int8_t power, uint16_t preambleLength, bool enableOOK, float tcxoVoltage, bool useRegulatorLDO)
{
    return radio->beginGFSK(freq, br, freqDev, power, preambleLength);
}

template<>
float RadioHal<SX1280>::getRSSI(bool packet,bool skipReceive)
{
    return radio->getRSSI();
}

template<>
int16_t RadioHal<SX1281>::beginFSK(float freq, float br, float freqDev, float rxBw, int8_t power, uint16_t preambleLength, bool enableOOK, float tcxoVoltage, bool useRegulatorLDO)
{
    return radio->beginGFSK(freq, br, freqDev, power, preambleLength);
}

template<>
float RadioHal<SX1281>::getRSSI(bool packet,bool skipReceive)
{
    return radio->getRSSI();
}

template<>
int16_t RadioHal<SX1282>::beginFSK(float freq, float br, float freqDev, float rxBw, int8_t power, uint16_t preambleLength, bool enableOOK, float tcxoVoltage, bool useRegulatorLDO)
{
    return radio->beginGFSK(freq, br, freqDev, power, preambleLength);
}

template<>
float RadioHal<SX1282>::getRSSI(bool packet,bool skipReceive)
{
    return radio->getRSSI();
}

template<>
float RadioHal<SX1268>::getRSSI(bool packet,bool skipReceive)
{
    return radio->getRSSI(packet);
}

template<>
float RadioHal<SX1262>::getRSSI(bool packet,bool skipReceive)
{
    return radio->getRSSI(packet);
}

template<>
float RadioHal<SX1261>::getRSSI(bool packet,bool skipReceive)
{
    return radio->getRSSI(packet);
}

template<>
float RadioHal<SX1231>::getRSSI(bool packet,bool skipReceive)
{
    return radio->getRSSI();
}

template<>
float RadioHal<SX1233>::getRSSI(bool packet,bool skipReceive)
{
    return radio->getRSSI();
}

template<>
float RadioHal<SX1279>::getRSSI(bool packet,bool skipReceive)
{
    return radio->getRSSI(packet,skipReceive);
}

template<>
float RadioHal<SX1278>::getRSSI(bool packet,bool skipReceive)
{
    return radio->getRSSI(packet,skipReceive);
}

template<>
float RadioHal<SX1276>::getRSSI(bool packet,bool skipReceive)
{
    return radio->getRSSI(packet,skipReceive);
}

template<>
float RadioHal<SX1273>::getRSSI(bool packet,bool skipReceive)
{
    return radio->getRSSI(packet,skipReceive);
}

template<>
float RadioHal<SX1272>::getRSSI(bool packet,bool skipReceive)
{
    return radio->getRSSI(packet,skipReceive);
}



template<>
float RadioHal<SX1268>::getFrequencyError(bool autoCorrect)
{
    return radio->getFrequencyError();
}

template<>
float RadioHal<SX1262>::getFrequencyError(bool autoCorrect)
{
    return radio->getFrequencyError();
}

template<>
float RadioHal<SX1261>::getFrequencyError(bool autoCorrect)
{
    return radio->getFrequencyError();
}


template<>
float RadioHal<SX1280>::getFrequencyError(bool autoCorrect)
{
    return radio->getFrequencyError();
}

template<>
float RadioHal<SX1281>::getFrequencyError(bool autoCorrect)
{
    return radio->getFrequencyError();
}

template<>
float RadioHal<SX1282>::getFrequencyError(bool autoCorrect)
{
    return radio->getFrequencyError();
}

template<>
float RadioHal<SX1279>::getFrequencyError(bool autoCorrect)
{
    return radio->getFrequencyError(autoCorrect);
}

template<>
float RadioHal<SX1278>::getFrequencyError(bool autoCorrect)
{
    return radio->getFrequencyError(autoCorrect);
}

template<>
float RadioHal<SX1276>::getFrequencyError(bool autoCorrect)
{
    return radio->getFrequencyError(autoCorrect);
}

template<>
float RadioHal<SX1273>::getFrequencyError(bool autoCorrect)
{
    return radio->getFrequencyError(autoCorrect);
}

template<>
float RadioHal<SX1272>::getFrequencyError(bool autoCorrect)
{
    return radio->getFrequencyError(autoCorrect);
}

template<>
void RadioHal<SX1268>::setDio0Action(void (*func)(void))
{
    radio->setPacketReceivedAction(func);
}

template<>
void RadioHal<SX1262>::setDio0Action(void (*func)(void))
{
    radio->setPacketReceivedAction(func);
}

template<>
void RadioHal<SX1261>::setDio0Action(void (*func)(void))
{
    radio->setPacketReceivedAction(func);
}

template<>
void RadioHal<SX1231>::setDio0Action(void (*func)(void))
{
    radio->setPacketReceivedAction(func);
}

template<>
void RadioHal<SX1233>::setDio0Action(void (*func)(void))
{
    radio->setPacketReceivedAction(func);
}

template<>
void RadioHal<SX1279>::setDio0Action(void (*func)(void))
{
    radio->setPacketReceivedAction(func);
}

template<>
void RadioHal<SX1278>::setDio0Action(void (*func)(void))
{
    radio->setPacketReceivedAction(func);

}

template<>
void RadioHal<SX1276>::setDio0Action(void (*func)(void))
{
    radio->setPacketReceivedAction(func);
}

template<>
void RadioHal<SX1273>::setDio0Action(void (*func)(void))
{
    radio->setPacketReceivedAction(func);
}

template<>
void RadioHal<SX1272>::setDio0Action(void (*func)(void))
{
    radio->setPacketReceivedAction(func);
}

template<>
void RadioHal<SX1280>::setDio0Action(void (*func)(void))
{
    radio->setPacketReceivedAction(func);
}

template<>
void RadioHal<SX1281>::setDio0Action(void (*func)(void))
{
    radio->setPacketReceivedAction(func);
}

template<>
void RadioHal<SX1282>::setDio0Action(void (*func)(void))
{
    radio->setPacketReceivedAction(func);
}

template<>
int16_t RadioHal<SX1279>::startReceive(uint8_t len, uint8_t mode)
{
    return radio->startReceive(len, mode);
}

template<>
int16_t RadioHal<SX1278>::startReceive(uint8_t len, uint8_t mode)
{
    //radio->setRSSIThreshold(-90.0);
    //radio->setOOK(false);
    // radio->disableAddressFiltering();
    //radio->disableBitSync();
    // radio->fixedPacketLengthMode(64);
    //radio->setGain(6);
    return radio->startReceive(len, mode);
}

template<>
int16_t RadioHal<SX1276>::startReceive(uint8_t len, uint8_t mode)
{
    return radio->startReceive(len, mode);
}

template<>
int16_t RadioHal<SX1273>::startReceive(uint8_t len, uint8_t mode)
{
    return radio->startReceive(len, mode);
}

template<>
int16_t RadioHal<SX1272>::startReceive(uint8_t len, uint8_t mode)
{
    return radio->startReceive(len, mode);
}

template<>
int16_t RadioHal<SX1268>::startReceive(uint8_t len, uint8_t mode)
{
    return radio->startReceive(len);
}

template<>
int16_t RadioHal<SX1262>::startReceive(uint8_t len, uint8_t mode)
{
    return radio->startReceive(len);
}

template<>
int16_t RadioHal<SX1261>::startReceive(uint8_t len, uint8_t mode)
{
    return radio->startReceive(len);
}

template<>
int16_t RadioHal<SX1231>::startReceive(uint8_t len, uint8_t mode)
{
    return radio->startReceive();
}

template<>
int16_t RadioHal<SX1233>::startReceive(uint8_t len, uint8_t mode)
{
    return radio->startReceive();
}

template<>
int16_t RadioHal<SX1280>::startReceive(uint8_t len, uint8_t mode)
{
    return radio->startReceive(len);
}

template<>
int16_t RadioHal<SX1280>::autoLDRO()
{
    return 0;
}

template<>
int16_t RadioHal<SX1281>::startReceive(uint8_t len, uint8_t mode)
{
    return radio->startReceive(len);
}

template<>
int16_t RadioHal<SX1281>::autoLDRO()
{
    return 0;
}

template<>
int16_t RadioHal<SX1282>::startReceive(uint8_t len, uint8_t mode)
{
    return radio->startReceive(len);
}

template<>
int16_t RadioHal<SX1282>::autoLDRO()
{
    return 0;
}

template<>
int16_t RadioHal<SX1279>::autoLDRO()
{
    return radio->autoLDRO();
}

template<>
int16_t RadioHal<SX1278>::autoLDRO()
{
    return radio->autoLDRO();
}

template<>
int16_t RadioHal<SX1276>::autoLDRO()
{
    return radio->autoLDRO();
}

template<>
int16_t RadioHal<SX1273>::autoLDRO()
{
    return radio->autoLDRO();
}

template<>
int16_t RadioHal<SX1272>::autoLDRO()
{
    return radio->autoLDRO();
}

template<>
int16_t RadioHal<SX1268>::autoLDRO()
{
    return radio->autoLDRO();
}

template<>
int16_t RadioHal<SX1262>::autoLDRO()
{
    return radio->autoLDRO();
}

template<>
int16_t RadioHal<SX1261>::autoLDRO()
{
    return radio->autoLDRO();
}

template<>
int16_t RadioHal<SX1279>::forceLDRO(bool enable)
{
    return radio->forceLDRO(enable);
}

template<>
int16_t RadioHal<SX1278>::forceLDRO(bool enable)
{
    return radio->forceLDRO(enable);
}

template<>
int16_t RadioHal<SX1276>::forceLDRO(bool enable)
{
    return radio->forceLDRO(enable);
}

template<>
int16_t RadioHal<SX1273>::forceLDRO(bool enable)
{
    return radio->forceLDRO(enable);
}

template<>
int16_t RadioHal<SX1272>::forceLDRO(bool enable)
{
    return radio->forceLDRO(enable);
}

template<>
int16_t RadioHal<SX1268>::forceLDRO(bool enable)
{
    return radio->forceLDRO(enable);
}

template<>
int16_t RadioHal<SX1262>::forceLDRO(bool enable)
{
    return radio->forceLDRO(enable);
}

template<>
int16_t RadioHal<SX1261>::forceLDRO(bool enable)
{
    return radio->forceLDRO(enable);
}

template<>
int16_t RadioHal<SX1280>::forceLDRO(bool enable)
{
    return 0;
}

template<>
int16_t RadioHal<SX1281>::forceLDRO(bool enable)
{
    return 0;
}

template<>
int16_t RadioHal<SX1282>::forceLDRO(bool enable)
{
    return 0;
}

template<>
int16_t RadioHal<SX1279>::fixedPacketLengthMode(uint8_t len)
{   if (len>64) len=64;
    return radio->fixedPacketLengthMode(len);
}

template<>
int16_t RadioHal<SX1278>::fixedPacketLengthMode(uint8_t len)
{   if (len>64) len=64;
    return radio->fixedPacketLengthMode(len);
}

template<>
int16_t RadioHal<SX1276>::fixedPacketLengthMode(uint8_t len)
{   if (len>64) len=64;
    return radio->fixedPacketLengthMode(len);
}

template<>
int16_t RadioHal<SX1273>::fixedPacketLengthMode(uint8_t len)
{   if (len>64) len=64;
    return radio->fixedPacketLengthMode(len);
}

template<>
int16_t RadioHal<SX1272>::fixedPacketLengthMode(uint8_t len)
{   if (len>64) len=64;
    return radio->fixedPacketLengthMode(len);
}

template<>
int16_t RadioHal<SX1268>::fixedPacketLengthMode(uint8_t len)
{
    return radio->fixedPacketLengthMode(len);
}

template<>
int16_t RadioHal<SX1262>::fixedPacketLengthMode(uint8_t len)
{
    return radio->fixedPacketLengthMode(len);
}

template<>
int16_t RadioHal<SX1261>::fixedPacketLengthMode(uint8_t len)
{
    return radio->fixedPacketLengthMode(len);
}

template<>
int16_t RadioHal<SX1231>::fixedPacketLengthMode(uint8_t len)
{
    return radio->fixedPacketLengthMode(len);
}

template<>
int16_t RadioHal<SX1233>::fixedPacketLengthMode(uint8_t len)
{
    return radio->fixedPacketLengthMode(len);
}

template<>
int16_t RadioHal<SX1280>::fixedPacketLengthMode(uint8_t len)
{
    return 0;
}

template<>
int16_t RadioHal<SX1281>::fixedPacketLengthMode(uint8_t len)
{
    return 0;
}

template<>
int16_t RadioHal<SX1282>::fixedPacketLengthMode(uint8_t len)
{
    return 0;
}

template<>
int16_t RadioHal<SX1279>::setCRC(uint8_t len,	uint16_t initial , uint16_t polynomial , bool inverted )
{
     return radio->setCRC(len==1);
}

template<>
int16_t RadioHal<SX1278>::setCRC(uint8_t len,	uint16_t initial , uint16_t polynomial , bool inverted )
{
    return radio->setCRC(len==1);
}

template<>
int16_t RadioHal<SX1276>::setCRC(uint8_t len,	uint16_t initial , uint16_t polynomial , bool inverted )
{
     return radio->setCRC(len==1);
}

template<>
int16_t RadioHal<SX1273>::setCRC(uint8_t len,	uint16_t initial , uint16_t polynomial , bool inverted )
{
     return radio->setCRC(len==1);
}

template<>
int16_t RadioHal<SX1272>::setCRC(uint8_t len,	uint16_t initial , uint16_t polynomial , bool inverted )
{
     return radio->setCRC(len==1);
}

template<>
int16_t RadioHal<SX1268>::setCRC(uint8_t len,	uint16_t initial , uint16_t polynomial , bool inverted )
{
    return radio->setCRC(len,initial,polynomial,inverted);
}

template<>
int16_t RadioHal<SX1262>::setCRC(uint8_t len,	uint16_t initial , uint16_t polynomial , bool inverted )
{
    return radio->setCRC(len,initial,polynomial,inverted);
}

template<>
int16_t RadioHal<SX1261>::setCRC(uint8_t len,	uint16_t initial , uint16_t polynomial , bool inverted )
{
    return radio->setCRC(len,initial,polynomial,inverted);
}

template<>
int16_t RadioHal<SX1280>::setCRC(uint8_t len,	uint16_t initial , uint16_t polynomial , bool inverted )
{
    return 0;
}

template<>
int16_t RadioHal<SX1281>::setCRC(uint8_t len,	uint16_t initial , uint16_t polynomial , bool inverted )
{
    return 0;
}

template<>
int16_t RadioHal<SX1282>::setCRC(uint8_t len,	uint16_t initial , uint16_t polynomial , bool inverted )
{
    return 0;
}

template<>
int16_t RadioHal<SX1272>::setEncoding(uint8_t encoding) 
{
    if (encoding == 10)
        encoding = 0;

    return radio->setEncoding(encoding);
}

template<>
int16_t RadioHal<SX1273>::setEncoding(uint8_t encoding) 
{
    if (encoding == 10)
        encoding = 0;

    return radio->setEncoding(encoding);
}

template<>
int16_t RadioHal<SX1276>::setEncoding(uint8_t encoding) 
{
    if (encoding == 10)
        encoding = 0;

    return radio->setEncoding(encoding);
}

template<>
int16_t RadioHal<SX1278>::setEncoding(uint8_t encoding) 
{
    if (encoding == 10)
        encoding = 0; 
    return radio->setEncoding(encoding);
}

template<>
int16_t RadioHal<SX1279>::setEncoding(uint8_t encoding) 
{
    if (encoding == 10)
        encoding = 0;

    return radio->setEncoding(encoding);
}

template<>
int16_t RadioHal<SX1262>::setEncoding(uint8_t encoding) 
{
    if (encoding == 10)
        encoding = 1;

    return radio->setEncoding(encoding);
}

template<>
int16_t RadioHal<SX1261>::setEncoding(uint8_t encoding) 
{
    if (encoding == 10)
        encoding = 1;

    return radio->setEncoding(encoding);
}

template<>
int16_t RadioHal<SX1231>::setEncoding(uint8_t encoding) 
{
    if (encoding == 10)
        encoding = 1;

    return radio->setEncoding(encoding);
}

template<>
int16_t RadioHal<SX1233>::setEncoding(uint8_t encoding) 
{
    if (encoding == 10)
        encoding = 1;

    return radio->setEncoding(encoding);
}

template<>
int16_t RadioHal<SX1268>::setEncoding(uint8_t encoding) 
{
    if (encoding == 10)
        encoding = 1;

    return radio->setEncoding(encoding);
}

template<>
int16_t RadioHal<SX1280>::setEncoding(uint8_t encoding) 
{
    if (encoding == 10)
        encoding = 1;

    return radio->setEncoding(encoding);
}

template<>
int16_t RadioHal<SX1281>::setEncoding(uint8_t encoding) 
{
    if (encoding == 10)
        encoding = 1;

    return radio->setEncoding(encoding);
}

template<>
int16_t RadioHal<SX1282>::setEncoding(uint8_t encoding) 
{
    if (encoding == 10)
        encoding = 1;

    return radio->setEncoding(encoding);
}

template<>
bool RadioHal<SX1272>::fifoAdd(uint8_t* data, int totalLen, int* remLen)
{
    return radio->fifoAdd(data,totalLen,remLen);
}

template<>
bool RadioHal<SX1273>::fifoAdd(uint8_t* data, int totalLen, int* remLen)
{
    return radio->fifoAdd(data,totalLen,remLen);
}

template<>
bool RadioHal<SX1276>::fifoAdd(uint8_t* data, int totalLen, int* remLen)
{
    return radio->fifoAdd(data,totalLen,remLen);
}

template<>
bool RadioHal<SX1278>::fifoAdd(uint8_t* data, int totalLen, int* remLen)
{
    return radio->fifoAdd(data,totalLen,remLen);
}

template<>
bool RadioHal<SX1279>::fifoAdd(uint8_t* data, int totalLen, int* remLen)
{
    return radio->fifoAdd(data,totalLen,remLen);
}

template<>
bool RadioHal<SX1268>::fifoAdd(uint8_t* data, int totalLen, int* remLen)
{
    return false;
}

template<>
bool RadioHal<SX1262>::fifoAdd(uint8_t* data, int totalLen, int* remLen)
{
    return false;
}

template<>
bool RadioHal<SX1261>::fifoAdd(uint8_t* data, int totalLen, int* remLen)
{
    return false;
}

template<>
bool RadioHal<SX1280>::fifoAdd(uint8_t* data, int totalLen, int* remLen)
{
    return false;
}

template<>
bool RadioHal<SX1281>::fifoAdd(uint8_t* data, int totalLen, int* remLen)
{
    return false;
}

template<>
bool RadioHal<SX1282>::fifoAdd(uint8_t* data, int totalLen, int* remLen)
{
    return false;
}

template<>
void RadioHal<SX1272>::setFifoEmptyAction(void (*func)(void))
{
    radio->setFifoEmptyAction(func);
}

template<>
void RadioHal<SX1273>::setFifoEmptyAction(void (*func)(void))
{
    radio->setFifoEmptyAction(func);
}

template<>
void RadioHal<SX1276>::setFifoEmptyAction(void (*func)(void))
{
    radio->setFifoEmptyAction(func);
}

template<>
void RadioHal<SX1278>::setFifoEmptyAction(void (*func)(void))
{
    radio->setFifoEmptyAction(func);
}

template<>
void RadioHal<SX1279>::setFifoEmptyAction(void (*func)(void))
{
    radio->setFifoEmptyAction(func);
}

template<>
void RadioHal<SX1261>::setFifoEmptyAction(void (*func)(void))
{

}

template<>
void RadioHal<SX1262>::setFifoEmptyAction(void (*func)(void))
{
    
}

template<>
void RadioHal<SX1268>::setFifoEmptyAction(void (*func)(void))
{
    
}

template<>
void RadioHal<SX1280>::setFifoEmptyAction(void (*func)(void))
{
    
}

template<>
void RadioHal<SX1281>::setFifoEmptyAction(void (*func)(void))
{
    
}

template<>
void RadioHal<SX1282>::setFifoEmptyAction(void (*func)(void))
{
    
}

template<>
void RadioHal<SX1272>::setFifoFullAction(void (*func)(void))
{
    radio->setFifoFullAction(func);
}

template<>
void RadioHal<SX1273>::setFifoFullAction(void (*func)(void))
{
    radio->setFifoFullAction(func);
}

template<>
void RadioHal<SX1276>::setFifoFullAction(void (*func)(void))
{
    radio->setFifoFullAction(func);
}

template<>
void RadioHal<SX1278>::setFifoFullAction(void (*func)(void))
{
    radio->clearFifoFullAction();
    radio->setFifoFullAction(func);
}

template<>
void RadioHal<SX1279>::setFifoFullAction(void (*func)(void))
{
    radio->setFifoFullAction(func);
}

template<>
void RadioHal<SX1261>::setFifoFullAction(void (*func)(void))
{

}

template<>
void RadioHal<SX1262>::setFifoFullAction(void (*func)(void))
{
    
}

template<>
void RadioHal<SX1268>::setFifoFullAction(void (*func)(void))
{
    
}

template<>
void RadioHal<SX1280>::setFifoFullAction(void (*func)(void))
{
    
}

template<>
void RadioHal<SX1281>::setFifoFullAction(void (*func)(void))
{
    
}

template<>
void RadioHal<SX1282>::setFifoFullAction(void (*func)(void))
{
    
}

template<>
bool RadioHal<SX1272>::fifoGet(volatile uint8_t* data, int totalLen, volatile int* rcvLen)
{
    return radio->fifoGet(data,totalLen,rcvLen);
}

template<>
bool RadioHal<SX1273>::fifoGet(volatile uint8_t* data, int totalLen, volatile int* rcvLen)
{
    return radio->fifoGet(data,totalLen,rcvLen);
}

template<>
bool RadioHal<SX1276>::fifoGet(volatile uint8_t* data, int totalLen, volatile int* rcvLen)
{
    return radio->fifoGet(data,totalLen,rcvLen);
}

template<>
bool RadioHal<SX1278>::fifoGet(volatile uint8_t* data, int totalLen, volatile int* rcvLen)
{
    return radio->fifoGet(data,totalLen,rcvLen);
}

template<>
bool RadioHal<SX1279>::fifoGet(volatile uint8_t* data, int totalLen, volatile int* rcvLen)
{
    return radio->fifoGet(data,totalLen,rcvLen);
}

template<>
bool RadioHal<SX1268>::fifoGet(volatile uint8_t* data, int totalLen, volatile int* rcvLen)
{
    return false;
}

template<>
bool RadioHal<SX1262>::fifoGet(volatile uint8_t* data, int totalLen, volatile int* rcvLen)
{
    return false;
}

template<>
bool RadioHal<SX1261>::fifoGet(volatile uint8_t* data, int totalLen, volatile int* rcvLen)
{
    return false;
}

template<>
bool RadioHal<SX1280>::fifoGet(volatile uint8_t* data, int totalLen, volatile int* rcvLen)
{
    return false;
}

template<>
bool RadioHal<SX1281>::fifoGet(volatile uint8_t* data, int totalLen, volatile int* rcvLen)
{
    return false;
}

template<>
bool RadioHal<SX1282>::fifoGet(volatile uint8_t* data, int totalLen, volatile int* rcvLen)
{
    return false;
}

template<>
int16_t RadioHal<SX1272>::setCurrentLimit(float currentLimit)
{
    return radio->setCurrentLimit(currentLimit);
}

template<>
int16_t RadioHal<SX1273>::setCurrentLimit(float currentLimit)
{
    return radio->setCurrentLimit(currentLimit);
}

template<>
int16_t RadioHal<SX1276>::setCurrentLimit(float currentLimit)
{
    return radio->setCurrentLimit(currentLimit);
}

template<>
int16_t RadioHal<SX1278>::setCurrentLimit(float currentLimit)
{
    return radio->setCurrentLimit(currentLimit);
}

template<>
int16_t RadioHal<SX1279>::setCurrentLimit(float currentLimit)
{
    return radio->setCurrentLimit(currentLimit);
}

template<>
int16_t RadioHal<SX1268>::setCurrentLimit(float currentLimit)
{
    return radio->setCurrentLimit(currentLimit);
}

template<>
int16_t RadioHal<SX1262>::setCurrentLimit(float currentLimit)
{
    return radio->setCurrentLimit(currentLimit);
}

template<>
int16_t RadioHal<SX1261>::setCurrentLimit(float currentLimit)
{
    return radio->setCurrentLimit(currentLimit);
}

template<>
int16_t RadioHal<SX1280>::setCurrentLimit(float currentLimit)
{
    return 0;
}

template<>
int16_t RadioHal<SX1281>::setCurrentLimit(float currentLimit)
{
    return 0;
}

template<>
int16_t RadioHal<SX1282>::setCurrentLimit(float currentLimit)
{
    return 0;
}

template<>
int16_t RadioHal<SX1272>::setOutputPower(int8_t power)
{
    if(power>17) power=20;
    return radio->setOutputPower(power);
}

template<>
int16_t RadioHal<SX1273>::setOutputPower(int8_t power)
{
    if(power>17) power=20;
    return radio->setOutputPower(power);
}

template<>
int16_t RadioHal<SX1276>::setOutputPower(int8_t power)
{
    if(power>17) power=20;
    return radio->setOutputPower(power);
}

template<>
int16_t RadioHal<SX1278>::setOutputPower(int8_t power)
{
    if(power>17) power=20;
    return radio->setOutputPower(power);
}

template<>
int16_t RadioHal<SX1279>::setOutputPower(int8_t power)
{
    if(power>17) power=20;
    return radio->setOutputPower(power);
}

template<>
int16_t RadioHal<SX1268>::setOutputPower(int8_t power)
{
    return false;
}

template<>
int16_t RadioHal<SX1262>::setOutputPower(int8_t power)
{
    return false;
}

template<>
int16_t RadioHal<SX1261>::setOutputPower(int8_t power)
{
    return false;
}

template<>
int16_t RadioHal<SX1280>::setOutputPower(int8_t power)
{
    return false;
}

template<>
int16_t RadioHal<SX1281>::setOutputPower(int8_t power)
{
    return false;
}

template<>
int16_t RadioHal<SX1282>::setOutputPower(int8_t power)
{
    return false;
}