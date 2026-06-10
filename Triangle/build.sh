cd ~/hack/trictrl/Triangle

make clean
make -j4

st-flash write build/Triangle.bin 0x08000000