# SPDX-License-Identifier: GPL-2.0
subdir-y	+= loongson
subdir-$(CONFIG_MACH_MEGASOC) += megasoc

obj-$(CONFIG_BUILTIN_DTB)	:= $(addsuffix /, $(subdir-y))
