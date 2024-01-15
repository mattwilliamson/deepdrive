# Deepdrive Hardware

## Motor Controller

### Install Requirements

```sh
pip install --upgrade Jetson.GPIO
```

### Enable Pin 32 / PWM0 + Enable Pin 33 / PWM2
```sh
sudo busybox devmem 0x700031fc 32 0x45
sudo busybox devmem 0x6000d504 32 0x2
sudo busybox devmem 0x70003248 32 0x46
sudo busybox devmem 0x6000d100 32 0x00
```

