
#include "ITG3200.h"
#include <Wire.h>

ITG3200::ITG3200(bool hl) : I2CDevice()
{
    setOffsets(0,0,0);
    if(hl)
        set_address(ITG3200_ADDR_AD0_HIGH);
    else
        set_address(ITG3200_ADDR_AD0_LOW);
}

void ITG3200::init() 
{
    init(NOSRDIVIDER, RANGE2000, BW256_SR8, PLL_ZGYRO_REF, true, true);
}

void ITG3200::init(byte _SRateDiv, byte _Range, byte _filterBW, byte _ClockSrc, bool _ITGReady, bool _INTRawDataReady) 
{
    setSampleRateDiv(_SRateDiv);
    setFSRange(_Range);
    setFilterBW(_filterBW);
    setClockSource(_ClockSrc);
    setITGReady(_ITGReady);
    setRawDataReady(_INTRawDataReady);
    delay(GYROSTART_UP_DELAY);
}

byte ITG3200::getSampleRateDiv() 
{
	return read_reg(SMPLRT_DIV);
}

void ITG3200::setSampleRateDiv(byte _SampleRate) 
{
    write_reg(SMPLRT_DIV, _SampleRate);
}

byte ITG3200::getFSRange() 
{
    return ((read_reg(DLPF_FS) & DLPFFS_FS_SEL) >> 3);
}

void ITG3200::setFSRange(byte _Range) 
{
    write_reg(DLPF_FS, ((read_reg(DLPF_FS) & ~DLPFFS_FS_SEL) | (_Range << 3)) );
}

byte ITG3200::getFilterBW() 
{
    return (read_reg(DLPF_FS) & DLPFFS_DLPF_CFG);
}

void ITG3200::setFilterBW(byte _BW)
{
    write_reg(DLPF_FS, ((read_reg(DLPF_FS) & ~DLPFFS_DLPF_CFG) | _BW));
}

bool ITG3200::isINTActiveOnLow() 
{
    return ((read_reg(INT_CFG) & INTCFG_ACTL) >> 7);
}

void ITG3200::setINTLogiclvl(bool _State)
{
    write_reg(INT_CFG, ((read_reg(INT_CFG) & ~INTCFG_ACTL) | (_State << 7)));
}

bool ITG3200::isINTOpenDrain()
{
    return ((read_reg(INT_CFG) & INTCFG_OPEN) >> 6);
}

void ITG3200::setINTDriveType(bool _State)
{
    write_reg(INT_CFG, ((read_reg(INT_CFG) & ~INTCFG_OPEN) | _State << 6));
}

bool ITG3200::isLatchUntilCleared() 
{
    return ((read_reg(INT_CFG) & INTCFG_LATCH_INT_EN) >> 5);
}

void ITG3200::setLatchMode(bool _State)
{
    write_reg(INT_CFG, ((read_reg(INT_CFG) & ~INTCFG_LATCH_INT_EN) | _State << 5));
}

bool ITG3200::isAnyRegClrMode() 
{
    return ((read_reg(INT_CFG) & INTCFG_INT_ANYRD_2CLEAR) >> 4);
}

void ITG3200::setLatchClearMode(bool _State) 
{
    write_reg(INT_CFG, ((read_reg(INT_CFG) & ~INTCFG_INT_ANYRD_2CLEAR) | _State << 4));
}

bool ITG3200::isITGReadyOn()
{
    return ((read_reg(INT_CFG) & INTCFG_ITG_RDY_EN) >> 2);
}

void ITG3200::setITGReady(bool _State)
{
    write_reg(INT_CFG, ((read_reg(INT_CFG) & ~INTCFG_ITG_RDY_EN) | _State << 2));
}

bool ITG3200::isRawDataReadyOn() 
{
    uint8_t r = read_reg(INT_CFG);
    return (r & INTCFG_RAW_RDY_EN);
}

void ITG3200::setRawDataReady(bool _State) 
{
    write_reg(INT_CFG, ((read_reg(INT_CFG) & ~INTCFG_RAW_RDY_EN) | _State));
}

bool ITG3200::isITGReady() 
{
    return ((read_reg(INT_STATUS) & INTSTATUS_ITG_RDY) >> 2);
}

bool ITG3200::isRawDataReady() 
{
    return (read_reg(INT_STATUS) & INTSTATUS_RAW_DATA_RDY);
}

void ITG3200::readTemp(double *_Temp) 
{
	uint8_t buf[2];
    read_bytes(TEMP_OUT, buf, 2);
    *_Temp = 35 + ((buf[0] << 8 | buf[1]) + 13200) / 280.0;    // F=C*9/5+32
}

void ITG3200::readGyroRaw(int *_GyroX, int *_GyroY, int *_GyroZ) 
{
	int8_t _buff[6];
    read_bytes(GYRO_XOUT, (uint8_t*)_buff, 6);

	int16_t x, y, z;
    x = _buff[0] << 8 | _buff[1];
    y = _buff[2] << 8 | _buff[3];
    z = _buff[4] << 8 | _buff[5];

	*_GyroX = x;
	*_GyroY = y;
	*_GyroZ = z;
}

void ITG3200::readGyroRaw( int *_GyroXYZ) 
{
    readGyroRaw(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2);
}

void ITG3200::setOffsets(int _Xoffset, int _Yoffset, int _Zoffset) 
{
    offsets[0] = _Xoffset;
    offsets[1] = _Yoffset;
    offsets[2] = _Zoffset;
}

void ITG3200::zeroCalibrate(unsigned int totSamples, unsigned int sampleDelayMS) 
{
    double tmpOffsets[] = {0,0,0};
    int xyz[3];

    for (unsigned int i = 0; i < totSamples; i++) 
	{
        delay(sampleDelayMS);
        readGyroRaw(xyz);
        tmpOffsets[0] += xyz[0];
        tmpOffsets[1] += xyz[1];
        tmpOffsets[2] += xyz[2];
    }
    setOffsets(-tmpOffsets[0] / totSamples + 0.5, -tmpOffsets[1] / totSamples + 0.5, -tmpOffsets[2] / totSamples + 0.5);
}

void ITG3200::readGyroRawCal(int *_GyroX, int *_GyroY, int *_GyroZ) 
{
    readGyroRaw(_GyroX, _GyroY, _GyroZ);
    *_GyroX += offsets[0];
    *_GyroY += offsets[1];
    *_GyroZ += offsets[2];
}

void ITG3200::readGyroRawCal(int *_GyroXYZ) 
{
    readGyroRawCal(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2);
}

imu::Vector<3> ITG3200::read_gyro()
{
    int x, y, z;
    imu::Vector<3> ret;
    readGyroRawCal(&x, &y, &z);
    ret.x() =  x / 14.375;
    ret.y() =  y / 14.375;
    ret.z() =  z / 14.375;
	return ret;
}

void ITG3200::reset() 
{
    write_reg(PWR_MGM, PWRMGM_HRESET);
    delay(GYROSTART_UP_DELAY); //gyro startup
}

bool ITG3200::isLowPower() 
{
    return (read_reg(PWR_MGM) & PWRMGM_SLEEP) >> 6;
}

void ITG3200::setPowerMode(bool _State) 
{
    write_reg(PWR_MGM, ((read_reg(PWR_MGM) & ~PWRMGM_SLEEP) | _State << 6));
}

bool ITG3200::isXgyroStandby() 
{
    return (read_reg(PWR_MGM) & PWRMGM_STBY_XG) >> 5;
}

bool ITG3200::isYgyroStandby()
{
    return (read_reg(PWR_MGM) & PWRMGM_STBY_YG) >> 4;
}

bool ITG3200::isZgyroStandby()
{
    return (read_reg(PWR_MGM) & PWRMGM_STBY_ZG) >> 3;
}

void ITG3200::setXgyroStandby(bool _Status)
{
    write_reg(PWR_MGM, ((read_reg(PWR_MGM) & PWRMGM_STBY_XG) | _Status << 5));
}

void ITG3200::setYgyroStandby(bool _Status)
{
    write_reg(PWR_MGM, ((read_reg(PWR_MGM) & PWRMGM_STBY_YG) | _Status << 4));
}

void ITG3200::setZgyroStandby(bool _Status)
{
    write_reg(PWR_MGM, ((read_reg(PWR_MGM) & PWRMGM_STBY_ZG) | _Status << 3));
}

byte ITG3200::getClockSource()
{
    return (read_reg(PWR_MGM) & PWRMGM_CLK_SEL);
}

void ITG3200::setClockSource(byte _CLKsource)
{
    write_reg(PWR_MGM, ((read_reg(PWR_MGM) & ~PWRMGM_CLK_SEL) | _CLKsource));
}


