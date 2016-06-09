/*
  Copyright (c) 2016 Arduino LLC.  All right reserved.
  author: Federico Vanzati (f.vanzati@arduino.cc)

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
package ina226

// INA226 library
// Datasheet: http://www.ti.com/lit/ds/symlink/ina226.pdf
import (
	"errors"
	"fmt"
	"github.com/saljam/i2c"
	"math"
)

// Registers address map
const (
	CONFIG_REG         byte = 0x00
	SHUNTVOLTAGE_REG   byte = 0x01
	BUSVOLTAGE_REG     byte = 0x02
	POWER_REG          byte = 0x03
	CURRENT_REG        byte = 0x04
	CALIBRATION_REG    byte = 0x05
	MASKENABLE_REG     byte = 0x06
	ALERTLIMIT_REG     byte = 0x07
	MANUFACTURERID_REG byte = 0xFE
	DIEID_REG          byte = 0xFF
)

// Configuration register helper constants
//
//   15   14  13  12    11     10     9       8       7       6      5      4      3      2     1     0
//  _____ ___ ___ ___ ______ ______ ______ _______ _______ _______ ______ ______ ______ _____ _____ _____
// |     |   |   |   |      |      |      |       |       |       |      |      |      |     |     |     |
// | RST | - | - | - | AVG2 | AVG1 | AVG0 |VBUSCT2|VBUSCT1|VBUSCT0|VSHCT2|VSHCT1|VSHCT0|MODE3|MODE2|MODE1|
// |_____|__ |___|___|______|______|______|_______|_______|_______|______|______|______|_____|_____|_____|

const (
	// MODE: operating mode (3-bit)
	INA226_MODE_POWER_DOWN     uint16 = 0x00
	INA226_MODE_SHUNT_TRIG     uint16 = 0x01
	INA226_MODE_BUS_TRIG       uint16 = 0x02
	INA226_MODE_SHUNT_BUS_TRIG uint16 = 0x03
	INA226_MODE_ADC_OFF        uint16 = 0x04
	INA226_MODE_SHUNT_CONT     uint16 = 0x05
	INA226_MODE_BUS_CONT       uint16 = 0x06
	INA226_MODE_SHUNT_BUS_CONT uint16 = 0x07

	// VSHCT: shunt voltage conversion time (3-bit)
	INA226_SHUNT_CONV_TIME_140US  uint16 = 0x00 << 3
	INA226_SHUNT_CONV_TIME_204US  uint16 = 0x01 << 3
	INA226_SHUNT_CONV_TIME_332US  uint16 = 0x02 << 3
	INA226_SHUNT_CONV_TIME_588US  uint16 = 0x03 << 3
	INA226_SHUNT_CONV_TIME_1100US uint16 = 0x04 << 3
	INA226_SHUNT_CONV_TIME_2116US uint16 = 0x05 << 3
	INA226_SHUNT_CONV_TIME_4156US uint16 = 0x06 << 3
	INA226_SHUNT_CONV_TIME_8244US uint16 = 0x07 << 3

	// VBUSCT: bus voltage conversion time (3-bit)
	INA226_BUS_CONV_TIME_140US  uint16 = 0x00 << 6
	INA226_BUS_CONV_TIME_204US  uint16 = 0x01 << 6
	INA226_BUS_CONV_TIME_332US  uint16 = 0x02 << 6
	INA226_BUS_CONV_TIME_588US  uint16 = 0x03 << 6
	INA226_BUS_CONV_TIME_1100US uint16 = 0x04 << 6
	INA226_BUS_CONV_TIME_2116US uint16 = 0x05 << 6
	INA226_BUS_CONV_TIME_4156US uint16 = 0x06 << 6
	INA226_BUS_CONV_TIME_8244US uint16 = 0x07 << 6

	// AVG: averaging mode (3-bit)
	INA226_AVERAGES_1    uint16 = 0x00 << 9
	INA226_AVERAGES_4    uint16 = 0x01 << 9
	INA226_AVERAGES_16   uint16 = 0x02 << 9
	INA226_AVERAGES_64   uint16 = 0x03 << 9
	INA226_AVERAGES_128  uint16 = 0x04 << 9
	INA226_AVERAGES_256  uint16 = 0x05 << 9
	INA226_AVERAGES_512  uint16 = 0x06 << 9
	INA226_AVERAGES_1024 uint16 = 0x07 << 9

	// RST bit
	INA226_RST uint16 = 0x01 << 15
)

type Ina226 struct {
	device *i2c.Device

	// calibration variables
	rShunt, iMax, vBusMax, vShuntMax float64
}

func New(bus int, addr byte) (*Ina226, error) {
	dev, err := i2c.NewDevice(bus, addr)
	if err != nil {
		return nil, err
	}
	ina226 := Ina226{device: dev}

	return &ina226, nil
}

func wordToByteArray(w uint16) []byte {
	buf := make([]byte, 2)
	buf[0] = byte(w >> 8)
	buf[1] = byte(w)
	return buf
}

func (ina226 *Ina226) Configure(confs ...uint16) error {
	var configuration uint16

	for _, conf := range confs {
		configuration |= conf
	}
	var buf []byte
	buf = append(buf, CONFIG_REG)
	buf = append(buf, wordToByteArray(configuration)...)

	_, err := ina226.device.Write(buf)
	if err != nil {
		return err
	}
	return nil
}

func (ina226 *Ina226) Calibrate(rShuntValue float64, iMaxValue float64) error {
	ina226.rShunt = rShuntValue
	ina226.iMax = iMaxValue

	currentLSB := ina226.iMax / 32768
	currentLSB *= 1000000 // transform to micro Ampere
	// As described in the datasheet to simplify calculation we should approximate the current LSB number
	// the method used is following described:
	// first extract from the currentLSB normalized notation only the mantissa
	currentLSB_mantissa := currentLSB / (math.Pow(10, math.Floor(math.Log10(currentLSB))))
	// then apply the ceiling function and multiply for the exponent
	currentLSB_approx := math.Ceil(currentLSB_mantissa) * math.Pow(10, math.Floor((math.Log10(currentLSB))))
	currentLSB = currentLSB_approx / 1000000 //transform back to Ampere

	calibrationValue := uint16((0.00512) / (currentLSB * ina226.rShunt))

	var buf []byte
	buf = append(buf, CALIBRATION_REG)
	buf = append(buf, wordToByteArray(calibrationValue)...)

	_, err := ina226.device.Write(buf)
	if err != nil {
		return err
	}
	return nil

}

func (ina226 *Ina226) Reset() error {
	// reset bit ins in the configure register
	var buf []byte
	buf = append(buf, CONFIG_REG)
	buf = append(buf, wordToByteArray(INA226_RST)...)
	_, err := ina226.device.Write(buf)
	if err != nil {
		return err
	}
	return nil
}

func (ina226 *Ina226) readRegister16(reg byte) (uint16, error) {
	// send request to register
	_, err := ina226.device.Write([]byte{reg})
	if err != nil {
		return 0, err
	}
	//read the 16 bit register content
	buf := make([]byte, 2)
	_, err = ina226.device.Read(buf)
	if err != nil {
		return 0, err
	}

	value := uint16(buf[0])<<8 | uint16(buf[1])
	return value, nil
}

func (ina226 *Ina226) readConfigurationRegister() (uint16, error) {
	confReg, err := ina226.readRegister16(CONFIG_REG)
	if err != nil {
		return 0, err
	}
	return confReg, nil
}

func (ina226 *Ina226) readCalibrationRegister() (uint16, error) {
	calReg, err := ina226.readRegister16(CALIBRATION_REG)
	if err != nil {
		return 0, err
	}
	return calReg, nil
}

func (ina226 *Ina226) CurrentResolution() (float64, error) {
	if ina226.rShunt <= 0.0 {
		return 0, errors.New(fmt.Sprintf("rShunt value: %f is not correct. Must be greater than 0", ina226.rShunt))
	}
	calibration, err := ina226.readCalibrationRegister()
	if err != nil {
		return 0, err
	}
	return 0.00512 / (float64(calibration) * ina226.rShunt), nil
}

func (ina226 *Ina226) ReadManufacturerRegister() (uint16, error) {
	confReg, err := ina226.readRegister16(MANUFACTURERID_REG)
	if err != nil {
		return 0, err
	}
	return confReg, nil
}

func (ina226 *Ina226) ReadBusVoltage() (float64, error) {
	voltage, err := ina226.readRegister16(BUSVOLTAGE_REG)
	if err != nil {
		return 0, err
	}
	return float64(voltage) * 1.25, nil
}

func (ina226 *Ina226) ReadShuntVoltage() (float64, error) {
	voltage, err := ina226.readRegister16(SHUNTVOLTAGE_REG)
	if err != nil {
		return 0, err
	}
	return float64(int16(voltage)) * 0.0025, nil
}

func (ina226 *Ina226) ReadShuntCurrentRegister() (int16, error) {
	currentRaw, err := ina226.readRegister16(CURRENT_REG)
	if err != nil {
		return 0, err
	}
	return int16(currentRaw), nil
}

func (ina226 *Ina226) ReadShuntCurrent() (float64, error) {
	currentRaw, err := ina226.ReadShuntCurrentRegister()
	if err != nil {
		return 0, err
	}

	currentResolution, err := ina226.CurrentResolution()
	if err != nil {
		return 0, err
	}

	return float64(currentRaw) * currentResolution, nil
}
