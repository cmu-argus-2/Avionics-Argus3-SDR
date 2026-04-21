export EFFCC_DIR="/home/argus/effcc/"
rm -rf ../../build
mkdir ../../build
cd ../../build
cmake -G Ninja .. -DEFF_STDIO_PORT=3
ninja Avionics-Argus3-SDR
sudo /home/argus/effcc/bin/eff-flash apps/Avionics-Argus3-SDR/scalar/Avionics-Argus3-SDR.hex sram

cd apps/Avionics-Argus3-SDR
