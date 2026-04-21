export EFFCC_DIR="/home/argus/effcc/"
rm -rf ../../build
mkdir ../../build
cd ../../build
# Route stdio to UART_4, which is the UART that main.c (and now
# main_spi0_smoke.c) explicitly initializes and pinmuxes. With
# EFF_STDIO_PORT=3 the stdio UART pins were never assigned, which is
# why nothing from printf/DBG_PRINTF ever reached /dev/ttyACM2.
cmake -G Ninja .. -DEFF_STDIO_PORT=4
ninja Avionics-Argus3-SDR
sudo /home/argus/effcc/bin/eff-flash apps/Avionics-Argus3-SDR/scalar/Avionics-Argus3-SDR.hex sram

minicom -b 115200 -D /dev/ttyACM2

cd apps/Avionics-Argus3-SDR
