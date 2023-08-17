# linux 构建命令
make lainsoc_defconfig && make -j
loongarch32r-linux-gnusf-objdump -S vmlinux > vmlinux.S
loongarch32r-linux-gnusf-objcopy ./vmlinux -O binary Image
mkimage -A mips -O linux -T kernel -C none -a 80300000 -e 808b10d0 -n lain-linux -d Image uImage

export CROSS_COMPILE=loongarch32r-linux-gnusf-
export ARCH=loongarch
make la32_defconfig
make vmlinux -j

# MOS 启动
fatload mmc 0 0xa0000000 mos.bin
fatload mmc 0 0xa3800000 fs.img
go 0xa0010b24

# LINUX 启动
fatload mmc 0 0xa2000000 vmlinux_fb
bootelf 0xa2000000 g console=ttyS0,230400 root=/dev/mmcblk0p2 rootfstype=ext4 rw rootwait loglevel=9

fatload mmc 0 0xa2000000 vmlinux
<!-- bootelf 0xa2000000 g console=ttyS0,230400 root=/dev/mmcblk0p2 rootfstype=ext4 rw rootwait loglevel=9 -->
bootelf 0xa2000000 g console=ttyS0,230400 root=/dev/mmcblk0p2 rootfstype=ext4 rw rootwait loglevel=9
bootelf 0xa2000000 g console=ttyS0,230400 root=/dev/mmcblk0p2 rootfstype=ext4 rw rootwait loglevel=9 uio_pdrv_genirq.of_id=generic-uio

# 备份 rootfs
sudo tar -czvf ~/c_test/rootfs_back.tar ./*

# alsa 相关构建命令
## lib

CC=loongarch32r-linux-gnusf-gcc ./configure --host=loongarch32x-linux --prefix=/home/buaa-nscscc/c_test/aplay/install
make
make install

## utils

CC=loongarch32r-linux-gnusf-gcc ./configure --host=loongarch32x-linux --prefix=/home/buaa-nscscc/c_test/aplay/install --with-alsa-inc-prefix=/home/buaa-nscscc/c_test/aplay/install/include --with-alsa-prefix=/home/buaa-nscscc/c_test/aplay/install/lib --disable-alsamixer --disable-xmlto --disable-nls
make
make install

# glibc 相关构建命令
## 构建命令
../configure \
--host=loongarch32r-linux-gnusf \
--prefix=/home/buaa-nscscc/c_test/la32r-toolchains/src/la32r_glibc-2.28/build/install \
--with-headers="/home/buaa-nscscc/lain/la32r-linux/usr/include" \
--enable-shared \
--disable-profile \
--disable-build-nscd \
--disable-werror \
--enable-obsolete-rpc \
CC="loongarch32r-linux-gnusf-gcc" \
CFLAGS="-O3" \
CXX="loongarch32r-linux-gnusf-g++" \
AR="loongarch32r-linux-gnusf-ar" \
AS="loongarch32r-linux-gnusf-as"

# ffmpeg 相关命令
ffmpeg -i vi.mp3 -acodec pcm_s24le -ar 48000 -f wav vi.wav
ffmpeg -i *.mp4 -acodec pcm_s24le -ar 48000 -f wav vi.wav

# BASH 相关构建命令
./configure --prefix=/home/buaa-nscscc/c_test/bash-5.2.15/build --target=loongarchx32-linux --host=loongarchx32-linux --build=x86_64-linux-gnu CC=loongarch32r-linux-gnusf-gcc AR=loongarch32r-linux-gnusf-ar
