#include "pico/stdlib.h"
#include <algorithm>
#include "hardware/i2c.h"
#include "balance_port.h"
#include "ads1x15.h"

bool BalancePort::initADC(i2c_inst_t* i2c_port) {
    // Initialize inputVoltagesADC, set gain, etc.

    inputVoltagesADC.setGain(ADSXGain_ONE);

    if (inputVoltagesADC.beginADSX(ADSX_ADDRESS_GND, i2c_port, 100, CONFIG::I2C_SDA_PIN, CONFIG::I2C_SCL_PIN)) {
      return true;
    } else {
      sleep_ms(5000);
      printf("ADS1x15 : Failed to initialize ADS.!\n");
      return false;
    }
}

adcVoltages BalancePort::getCellVoltages() {
    adcVoltages voltages;
    int16_t adc0, adc1, adc2, adc3;

    adc0 = inputVoltagesADC.readADC_SingleEnded(ADSX_AIN0);
    adc1 = inputVoltagesADC.readADC_SingleEnded(ADSX_AIN1);
    adc2 = inputVoltagesADC.readADC_SingleEnded(ADSX_AIN2);
    adc3 = inputVoltagesADC.readADC_SingleEnded(ADSX_AIN3);

    voltages.cell1 = 6 * (inputVoltagesADC.computeVolts(adc0) - inputVoltagesADC.computeVolts(adc1));
    voltages.cell2 = 6 * (inputVoltagesADC.computeVolts(adc1) - inputVoltagesADC.computeVolts(adc2));
    voltages.cell3 = 6 * inputVoltagesADC.computeVolts(adc2);
    voltages.psu = 6 * inputVoltagesADC.computeVolts(adc3);

    return voltages;
}

CellStatus BalancePort::checkVoltages(adcVoltages measuredVoltages) {
    CellStatus voltageStatus;
    voltageStatus.voltages =  measuredVoltages;
    float cell1 = measuredVoltages.cell1;
    float cell2 = measuredVoltages.cell2;
    float cell3 = measuredVoltages.cell3;
    float PSU = measuredVoltages.psu;

    float maxVoltage = std::max({cell1, cell2, cell3});
    float minVoltage = std::min({cell1, cell2, cell3});
    voltageStatus.allOk = true;
    if ((maxVoltage - minVoltage) > balanceThreshold){
        voltageStatus.outOfBalance = true;
        voltageStatus.allOk = false;
        voltageStatus.fault = "cells out of balance, ";
    }
    if ( minVoltage < minCellVoltage ){
        voltageStatus.lowCellVoltage = true;
        voltageStatus.allOk = false;
        voltageStatus.fault += "cell undervoltage, ";
    }
    if (maxVoltage > maxCellVoltage) {
        voltageStatus.highCellVoltage = true;
        voltageStatus.allOk = false;
        voltageStatus.fault += "cell overvoltage, ";
    }
    if (PSU > PSUConnectedThreshold && PSU < minPSU) {
        voltageStatus.psuUnderVoltage = true;
        voltageStatus.allOk = false;
        voltageStatus.fault += "PSU undervoltage, ";
    }
    return voltageStatus;
}

void BalancePort::raiseCellStatus() {
    const adcVoltages voltages = getCellVoltages(); // Assume this method exists and fetches voltages
    const CellStatus status = checkVoltages(voltages);
    if (!status.allOk) {
        // Increment failCount if the cell status is not okay for 10 consecutive times
        failCount++;
        if (failCount > failCountThreshold) {
            printf("input voltage error! %s Voltages: ", status.fault.c_str());
            printf("cell 1: %fV, cell 2: %fV, cell 3: %fV, PSU: %fV\n",
                   status.voltages.cell1, status.voltages.cell2,
                   status.voltages.cell3, status.voltages.psu);
        }
    } else {
        failCount = 0;
    }
}
