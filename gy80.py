#
# GY80 rp2040
# four on board chips
# 30 = 0x1e HMC5883L digital compass
# 83 0x53  ADXL 345 accelerometer
# 105 0x69 L3G4200D gyro
# 119 0x77 BMP085 Pressure Temp.
#
import machine
import math
import utime

OUTPUT_DATA_INTERVAL = 20 # ms
timestamp = 0

# the pin indexes are the gpio numbers so Pin is a gpio pin
sda=machine.Pin(0)
scl=machine.Pin(1)
i2c=machine.I2C(0,sda=sda, scl=scl, freq=400000)
print('i2c devices found {0}'.format(i2c.scan()))

class I2cDevice:
    # protected attributes
    _debug = False

    def __init__ (self, deviceAddress):
        self.__deviceAddress = deviceAddress

    @staticmethod
    def _unsig (bytes):
        return bytes[0]*256 + bytes[1]

    @staticmethod
    def _unsig_le (bytes):
        return bytes[1]*256 + bytes[0]

    @staticmethod
    def _twos_complement (bytes):
        unsigned = I2cDevice._unsig(bytes)
        if unsigned & 0x8000:
             return ((unsigned & 0x7fff) -32768)
        else:
             return unsigned

    # two comp little endian byte order
    @staticmethod
    def _twos_complement_le (bytes):
        unsigned = I2cDevice._unsig_le(bytes)
        if unsigned & 0x8000:
             return ((unsigned & 0x7fff) -32768)
        else:
             return unsigned
            
    # read from address        
    def _rfa (self, addr, len=2):
        return i2c.readfrom_mem(self.__deviceAddress, addr, len)

    # write to address
    def _wta (self, addr, value):
        i2c.writeto_mem(self.__deviceAddress, addr, bytes([value]))

#=============================================================================
#
# Pressure and temperature sensor
#
class Bmp085(I2cDevice):
    __registerBase=0xAA # __registerBase register address for BMP085
    __results = {}
    __registers = ['AC1', 'AC2', 'AC3', 'AC4', 'AC5', 'AC6', 'B1', 'B2', 'MB', 'MC', 'MD'] 
    __unsigned = ['AC4', 'AC5', 'AC6']
    
    CONTROL_REG = 0xF4
    MEASURE_TEMPERATURE = 0x2E
    MEASURE_PRESSURE = 0x34   # measure pressure (osrs=0)
    MSB = 0xF6


    # constructor taking optional address
    def __init__ (self, deviceAddress=0x77):
       # 0xEF read 0xEE write
        super().__init__(deviceAddress)
        id = self._rfa(0xD0)
        if self._debug:
            print('ID {0} {1}'.format(hex(id[0]),hex(id[1])))

        # populate the hash with values from the named registers
        for register in self.__registers:
            value= self._rfa(self.__registerBase)
            if register in self.__unsigned:
                self.__results[register] = I2cDevice._unsig(value)
            else:
                self.__results[register] = I2cDevice._twos_complement(value)
            self.__registerBase+=2
            if self._debug:
                print('{0} {1}'.format(register, self.__results[register]))
        
    def getTemperature(self):
        self._wta(self.CONTROL_REG, self.MEASURE_TEMPERATURE)
        # wait 4.5 ms
        utime.sleep_us(4500)
        # read 2 bytes msb f6, lsb f7
        UT=self._unsig(self._rfa(self.MSB))

        X1=((UT-self.__results['AC6']) * self.__results['AC5']) / float(1<<15)
        X2=((self.__results['MC'] * float(1<<11)))/(X1+self.__results['MD'])
        self.__B5=X1+X2
        if self._debug:
            print('UT {0}'.format(UT))
            print('X1 {0}'.format(X1))
            print('X2 {0}'.format(X2))
            print('B5 {0}'.format(self.__B5))
        self.__T=(self.__B5+8.0)/16.0
        return self.__T

    # requires temperature to be calculated before this can be calculated
    def getPressure(self):
        self._wta(self.CONTROL_REG, self.MEASURE_PRESSURE)
        # wait 4.5 ms
        utime.sleep_us(4500)
        if self._debug:
            print('B5 {0} T {1}'.format(self.__B5,self.__T))
   
        # read 3 bytes msb f6, lsb f7, xlsb f8
        UP0= self._rfa(self.MSB,3)
        if self._debug:
            print('MSB {0}'.format(UP0[0]))
            print('LSB {0}'.format(UP0[1]))
            print('XLSB {0}'.format(UP0[2]))
        # if oss is 0 then xlsb is totally ignored
        UP=((UP0[0]<<16) + (UP0[1]<<8) + UP0[2]) >> 8 

        B6=math.floor(self.__B5 - 4000)
        X1=math.floor((self.__results['B2'] * ((B6 * B6)/float(1<<12)))/float(1<<11))
        X2=math.floor(((self.__results['AC2'] * B6)/float(1<<11)))
        X3=X1+X2
        B3=math.floor(((self.__results['AC1']*4+X3)+2)/4)
        if self._debug:
            print('UP {0}'.format(UP))
            print('B6 {0}'.format(B6))
            print('X1 {0}'.format(X1))
            print('X2 {0}'.format(X2))
            print('X3 {0}'.format(X3))
            print('B3 {0}'.format(B3))

        X1=math.floor(((self.__results['AC3']*B6)/float(1<<13)))
        X2=math.floor((self.__results['B1']*((B6*B6)/float(1<<12))/float(1<<16)))
        X3=math.floor(((X1+X2)+2)/4)
        B4=math.floor((self.__results['AC4']*(X3+32768))/float(1<<15))
        B7=math.floor((UP-B3)*50000)

        if self._debug:
            print('X1 {0}'.format(X1))
            print('X2 {0}'.format(X2))
            print('X3 {0}'.format(X3))
            print('B4 {0}'.format(B4))
            print('B7 {0}'.format(B7))
            print('B7 {0}'.format(hex(B7)))

        if B7 < 0x80000000:
                p = math.floor((B7*2)/B4)
        else:
                p = math.floor((B7/B4)*2)
     
        X1=math.floor((p/float(1<<8))*(p/float(1<<8)))
        if self._debug:
            print('p {0}'.format(p))
            print('X1 {0}'.format(X1))
        X1=math.floor((X1*3038)/float(1<<16))
        X2=math.floor((-7357*p)/float(1<<16))
        if self._debug:
            print('X1 {0}'.format(X1))
            print('X2 {0}'.format(X2))
        p = math.floor(p + (X1+X2+3791)/16)
        return p
#=============================================================================
#
# Accelerometer
#
class Adxl345(I2cDevice):
    X=0x32
    Y=0x34
    Z=0x36
    POWER = 0x2D
    DATA_FORMAT = 0x31
    RATE = 0x2C
    
    __registerBase=0x00 # _registerBase register address for ADXL345
    
    __fXg = 0.0
    __fYg = 0.0
    __fZg = 0.0
    
    ALPHA = 0.5

    # constructor taking optional address
    def __init__ (self, deviceAddress=0x53):
       # 0xEF read 0xEE write
        super().__init__(deviceAddress)
        if self._debug:
            results = []
            for i in range(58):
                results.append(self._rfa(i,1))
                if int.from_bytes(results[i], "big") != 0:
                    print('{0} {1}'.format(i,hex(int.from_bytes(results[i], "big"))))

        self._wta(self.POWER, 0x08) # measurement mode
        utime.sleep_us(5000)
        self._wta(self.DATA_FORMAT, 0x08) # full res.
        utime.sleep_us(5000)
        self._wta(self.RATE, 0x09) # 50Hz Normal
        utime.sleep_us(5000)
    
    def getX(self):
        return self._twos_complement_le(self._rfa(self.X))
    
    def getY(self):
        return self._twos_complement_le(self._rfa(self.Y))
    
    def getZ(self):
        return self._twos_complement_le(self._rfa(self.Z))
    
    def getRoll(self):
        # see oscarlamng using gy80 https://oscarliang.com/use-gy80-arduino-adxl345-accelerometer/
        Xg = self.getX()
        Yg = self.getY()
        Zg = self.getZ()
        self.__fXg = Xg * self.ALPHA + (self.__fXg * ( 1.0 - self.ALPHA))       
        self.__fYg = Yg * self.ALPHA + (self.__fYg * ( 1.0 - self.ALPHA))       
        self.__fZg = Zg * self.ALPHA + (self.__fZg * ( 1.0 - self.ALPHA))
        return (math.atan2(- self.__fYg, self.__fZg) * 180.0)/ math.pi

    def getPitch(self):
        return (math.atan2(self.__fXg, math.sqrt(self.__fYg * self.__fYg + self.__fZg * self.__fZg)) * 180.0)/ math.pi
        
#=============================================================================
#
# Digital compass
#
class Hmc5883L(I2cDevice):
    X=0x03 
    Y=0x07  # NB the addresses are swapped
    Z=0x05  # NB the addresses are swapped
    
    # register addresses
    CRA = 0x00
    CRB = 0x01
    MODE = 0x02
    
    IDENT_REG_A = 0x0A

    # constructor taking optional address
    def __init__ (self, deviceAddress=0x1E):
       # 0xEF read 0xEE write
        super().__init__(deviceAddress)
        if self._debug:
            results = []
            for i in range(13):
                results.append(self._rfa(i,1))
                if int.from_bytes(results[i], "big") != 0:
                    print('{0} {1}'.format(i,hex(int.from_bytes(results[i], "big"))))
            id = self._rfa(self.IDENT_REG_A,3)
            print('id = {0}'.format(id))

        self._wta(self.CRA, 0x70) # 8 samples average 15hz
        self._wta(self.CRB, 0xA0) # gain = 5, 390 lsb/gauss resolution 2.56 
        self._wta(self.MODE, 0x00) # continuous

        # wait 6 ms
        utime.sleep_us(6000)

    def getX(self):
        return self._twos_complement(self._rfa(self.X))
    def getY(self):
        return self._twos_complement(self._rfa(self.Y))
    def getZ(self):
        return self._twos_complement(self._rfa(self.Z))
    
    def getBearing(self, roll, pitch):
        # convert to radians
        roll = roll*math.pi/180.0
        pitch = pitch*math.pi/180.0
        
        x=self.getX()
        y=self.getY()
        z=self.getZ()
        
        cos_roll = math.cos(roll)
        sin_roll = math.sin(roll)
        cos_pitch = math.cos(pitch)
        sin_pitch = math.sin(pitch)
        
        # Tilt compensated magnetic field X
        mag_x = x*cos_pitch + y*sin_roll*sin_pitch + z*cos_roll*sin_pitch
        # Tilt compensated magnetic field Y
        mag_y = y*cos_roll + z*sin_roll
        
        # bearing
        return math.atan2(-mag_y, mag_x)*180.0/math.pi
#=============================================================================
#
# Gyroscope
#
class L3g4200d(I2cDevice):
    X=0x28
    Y=0x2A
    Z=0x2C
    CONTROL_REG1 = 0x20
    CONTROL_REG4 = 0x23
    AUTO_INCREMENT_ADDRESS = 0x80            
    
    # constructor taking optional address
    def __init__ (self, deviceAddress=0x69):
       # 0xEF read 0xEE write
        super().__init__(deviceAddress)
        if self._debug:
            results = []
            for i in range(57):
                results.append(self._rfa(i,1))
                if int.from_bytes(results[i], "big") != 0:
                    print('{0} {1}'.format(hex(i),hex(int.from_bytes(results[i], "big"))))

        self._wta(self.CONTROL_REG1, 0x0F) # normal power mode, all axes enabled, 100Hz
        utime.sleep_us(5000)
        self._wta(self.CONTROL_REG4, 0x20) # Continuous update Little endian, 2000 dps full scale
        utime.sleep_us(5000)
    
    def getX(self):
        return self._twos_complement_le(self._rfa(self.X | self.AUTO_INCREMENT_ADDRESS))
    def getY(self):
        return self._twos_complement_le(self._rfa(self.Y | self.AUTO_INCREMENT_ADDRESS))
    def getZ(self):
        return self._twos_complement_le(self._rfa(self.Z | self.AUTO_INCREMENT_ADDRESS))

#=============================================================================
# main loop
verbose=False

bmp085=Bmp085()
adxl345=Adxl345()
hmc5883L=Hmc5883L()
l3g4200d=L3g4200d()

while True:
    if utime.ticks_ms() - timestamp > OUTPUT_DATA_INTERVAL:
        timestamp_old = timestamp
        timestamp = utime.ticks_ms()
        
        temp=bmp085.getTemperature()
        pressure=bmp085.getPressure()

        if verbose:
            print('Temperature {0} degrees C Pressure {1} hPa {2} mmHg'.format(temp/10.0, pressure/100.0, (pressure*7.50062)/1000.0))
            print('Accelerometer adxl345 roll {0} pitch {1}'.format(adxl345.getRoll(),adxl345.getPitch()))
            print('Compass hmc5883L X={0} Y={1} Z={2} bearing {3}'.format(hmc5883L.getX(),hmc5883L.getY(),hmc5883L.getZ(),hmc5883L.getBearing(adxl345.getRoll(),adxl345.getPitch())))
            print('Gyro l3g4200d X={0} Y={1} Z={2}'.format(l3g4200d.getX(),l3g4200d.getY(),l3g4200d.getZ()))
            print()
            utime.sleep(1)
        else:
            print('{0:.2f};{1:.2f};{2:.2f};{3:.2f};{4:.2f};{5:.2f}'.format(hmc5883L.getBearing(adxl345.getRoll(),adxl345.getPitch()),adxl345.getPitch(),adxl345.getRoll(),temp/10.0,pressure/100.0,0.0))

