source ../env/core-v-mcu.sh;

make;

cp cli_test /home/wangshun/bin/qemu-riscv/bin ;

cd /home/wangshun/bin/qemu-riscv/bin/ ;

./qemu-system-riscv32 -M core_v_mcu -bios none -kernel cli_test -nographic -monitor none -serial stdio


