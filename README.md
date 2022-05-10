Install Teensy Loader
Install Arduino CLI

```
sudo cp 00-teensy.rules /etc/udev/rules.d/
arduino-cli core install teensy:avr
arduino-cli core install adafruit:samd
```