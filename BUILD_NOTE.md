# 构建命令
make lainsoc_defconfig && make -j
loongarch32r-linux-gnusf-objdump -S vmlinux > vmlinux.S
loongarch32r-linux-gnusf-objcopy ./vmlinux -O binary Image
mkimage -A mips -O linux -T kernel -C none -a 80300000 -e 808b10d0 -n lain-linux -d Image uImage
