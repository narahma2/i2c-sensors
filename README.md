# i2c-sensors
Python/LabJack scripts implementing I<sup>2</sup>C communications for MS8607 and SHT3x
environmental sensors. These scripts were run with a LabJack T7-Pro device,
however any of the LabJack series should work.

# Quick-Start
1. Install the LJM library:
https://support.labjack.com/docs/ljm-software-installer-downloads-t4-t7-t8-digit
2. Install `numpy` and `labjack-ljm` packages into your python environment:
`pip install numpy labjack-ljm`
3. Run the `labjack_i2c.py` script

# Documentation
## MS8607 vs. SHT3x
Update line 382 to choose which sensor function to call

## LabJack Configuration
Update lines 385/390 to the correct IP address of your LabJack device
(communication can also be done over USB, will need to update the `ljm.openS()`
functions--`readSHT31()` has an example of connecting over IP, `readMS8607()`
connects over USB).

Additionally, the pinouts are assumed to be SDA -> FIO1, SCL -> FIO0. You can
update this in lines 31-33 and 122-124, depending on your preferred setup.

```python
# SDA: FIO1, SCL: FIO0
ljm.eWriteName(handle, 'I2C_SDA_DIONUM', 1)
ljm.eWriteName(handle, 'I2C_SCL_DIONUM', 0)
```
