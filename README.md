Driver for CCSI for something like TI LP589x on i.MX6 using SSI

- testing on VAR-SOM-SOLO/DUAL which is an i.MX6 

## Build

source the toolchain from vendor

    . /opt/fslc-xwayland/4.0/environment-setup-cortexa9t2hf-neon-fslc-linux-gnueabi


### test

    gcc -D USER_SPACE test.c ccsi.c