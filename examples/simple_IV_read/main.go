package main

import (
	"fmt"
	"github.com/Fede85/go-ina226"
	"log"
)

func checkError(err error) {
	if err != nil {
		log.Fatal(err)
	}
}

func main() {

	// INA226 A0 and A1 tied to GND, address is 0x40
	// device connected to I2C bus 1
	currentSensor, err := ina226.New(1, 0x40)
	checkError(err)

	err = currentSensor.Configure(ina226.INA226_SHUNT_CONV_TIME_1100US, ina226.INA226_BUS_CONV_TIME_1100US, ina226.INA226_AVERAGES_1, ina226.INA226_MODE_SHUNT_BUS_CONT)
	checkError(err)

	// measure the power rail voltage
	vBus, err := currentSensor.ReadBusVoltage()
	checkError(err)
	fmt.Printf("Bus voltage: %2.5f V\n", vBus)
	// measure the voltage drop across the shunt resistor
	vShunt, err := currentSensor.ReadShuntVoltage()
	checkError(err)
	fmt.Printf("Shunt voltage: %2.5f V\n", vShunt)

	// If you want to read the current directly you must calibrate the sensor first
	// providing the Shunt resistor value (expressed in ohm) and
	// the maximum Expected current (expressed in Ampere).
	// This values are required to set the resolution of the readings
	currentSensor.Calibrate(0.01, 0.5)

	// read back the resolution
	iResolution, err := currentSensor.CurrentResolution()
	checkError(err)
	fmt.Printf("Current resolution: %2.5f A/bit\n", iResolution)

	iRegister, err := currentSensor.ReadShuntCurrentRegister()
	checkError(err)
	fmt.Println("Shunt current register:", iRegister)

	iShunt, err := currentSensor.ReadShuntCurrent()
	checkError(err)
	fmt.Printf("Shunt current: %2.5f A\n", iShunt)

	// stop acquisition
	//err = currentSensor.Configure(ina226.INA226_MODE_POWER_DOWN)
	//checkError(err)
}
