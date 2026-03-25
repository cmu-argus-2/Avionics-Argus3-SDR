export EFFCC_DIR="/home/argus/effcc/"
mkdir ../../build
cd ../../build
cmake -G Ninja . -DEFF_STDIO_PORT=3
ninja
sudo /home/argus/effcc/bin/eff-flash apps/Avionics-Argus3-SDR/scalar/Avionics-Argus3-SDR.hex sram

minicom -b 115200 -D /dev/ttyACM2

cd apps/Avionics-Argus3-SDR
