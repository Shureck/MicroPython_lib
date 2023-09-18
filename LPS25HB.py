# boot.py -- run on boot-up
from machine import Pin
import machine
import math

GOST4401_MIN_PRESSURE = 6.69384
GOST4401_MAX_PRESSURE = 101325.00

GOST4401_G = 9.80665
GOST4401_R = 287.05287
GOST4401_E = 6356766

GOST4401_MIN_GPALT = 0.00
GOST4401_MAX_GPALT = 51000.00
GOST4401_LUT_RECORDS = 6

ag_table = [
[ 0, 288.15, -0.0065, 101325.00 ],  [ 11000, 216.65, 0.0, 22632.04 ],
[ 20000, 216.65, 0.0010, 5474.87 ], [ 32000, 228.65, 0.0028, 868.0146 ],
[ 47000, 270.65, 0.0, 110.9056 ],   [ 51000, 270.65, -0.0028, 6.69384 ]]

class LPS25HB:

    def __init__(self, sda=21, scl=22):
        self.i2c = machine.I2C(0, sda=machine.Pin(sda), scl=machine.Pin(scl))
        buf = bytearray(2)
        buf[0] = 0x20
        buf[1] = 0x90
        self.i2c.writeto(0x5C, buf)

    def twos_complement(self, hexstr, bits):
        value = int(hexstr, bits)
        if value & (1 << (bits - 1)):
            value -= 1 << bits
        return value

    def readTemperatureRaw(self):
        t_h = self.i2c.readfrom_mem(0x5C, 0x2B, 1)[0]
        t_l = self.i2c.readfrom_mem(0x5C, 0x2C, 1)[0]
        retval = t_l << 8 | t_h
        return self.twos_complement(hex(retval)[2:], 16)

    def readPressureRaw(self):
        data = self.i2c.readfrom_mem(0x5C, 0x28 | 0x80, 3)
        return data[2] * 65536 + data[1] * 256 + data[0]

    def readTemperatureC(self):
        return 42.5 + (self.readTemperatureRaw() / 480.0)

    def readPressureMillibars(self):
        return self.readPressureRaw() / 4096.0
    
    def readPressurePascals(self):
        return self.readPressureMillibars() * 100   #MILLIBARS_TO_PASCALS

    def readAltitude(self):
        R = 8.31432  # универсальная газовая постоянная, J/(mol*K)
        g = 9.80665  # ускорение свободного падения на поверхности Земли, m/s^2
        M = 0.0289644  # молярная масса сухого воздуха, kg/mol
        L = 0.0065  # температурная лапласова константа, K/m
        T0 = 288.15  # стандартная температура на уровне моря, K
        p0 = 101325  # стандартное давление на уровне моря, Па

        temperature_celsius = self.readTemperatureC()
        pressure_pascals = self.readPressurePascals()

        temperature_kelvin = temperature_celsius + 273.15
        temperature_h = T0 - L * 0.5
        pressure_h = p0 * (temperature_h / T0) ** (-g * M / (R * L))
        height = -((R * temperature_h) / (g * M)) * math.log(pressure_pascals / pressure_h)

        return height

    def getAltitude(self):
        pressurePa = self.readPressurePascals()

        if ((pressurePa <= GOST4401_MIN_PRESSURE) or (pressurePa > GOST4401_MAX_PRESSURE)):
            return None

        idx = 0
        for i in range(GOST4401_LUT_RECORDS - 1):
            if ((pressurePa <= ag_table[i][3]) or (pressurePa > ag_table[i + 1][3])):
                idx = i
                break

        Ps = ag_table[idx][3]
        Bm = ag_table[idx][2]
        Tm = ag_table[idx][1]
        Hb = ag_table[idx][0]
        geopotH = 0

        if Bm != 0.0:
            geopotH = ((Tm * pow(Ps / pressurePa, Bm * GOST4401_R / GOST4401_G) - Tm) / Bm)
        else:
            geopotH = log10(Ps / pressurePa) * (GOST4401_R * Tm) / (GOST4401_G * 0.434294)

        return (Hb + geopotH) * GOST4401_E / (GOST4401_E - (Hb + geopotH))
