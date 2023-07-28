# 构建命令
make lainsoc_defconfig && make -j
loongarch32r-linux-gnusf-objdump -S vmlinux > vmlinux.S
loongarch32r-linux-gnusf-objcopy ./vmlinux -O binary Image
mkimage -A mips -O linux -T kernel -C none -a 80300000 -e 808b10d0 -n lain-linux -d Image uImage


export CROSS_COMPILE=loongarch32r-linux-gnusf-
export ARCH=loongarch
make la32_defconfig
make vmlinux -j

----
fatload mmc 0 0xa2000000 vmlinux
bootelf 0xa2000000 g console=ttyS0,230400 rdinit=/init 