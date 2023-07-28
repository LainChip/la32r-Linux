/*
 * NaiveMIPS interrupt controller setup
 *
 * Copyright (C) 2017 Tsinghua Univ.
 * Author: Yuxiang Zhang
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

// #include <linux/of_fdt.h>
// #include <linux/of_platform.h>
// #include <linux/clk-provider.h>
// #include <linux/irqchip.h>
// #include <linux/clk.h>
// #include <linux/clocksource.h>
// #include <asm/prom.h>
// #include <asm/irq_cpu.h>
// #include <asm/time.h>
// #include <asm/mips-cps.h>
// #include <asm/irq_cpu.h>

// void __init arch_init_irq(void)
// {
// 	unsigned int sr;
// 	struct device_node *intc_node;
// 	pr_cont("arch_init_irq with ebase: 0x%x\n", (u32) ebase);
// 	sr = set_c0_status(ST0_BEV);
// 	write_c0_ebase((u32)ebase);
// 	write_c0_status(sr);
// 	intc_node = of_find_compatible_node(NULL, NULL,
// 			"mti,cpu-interrupt-controller");
// 	of_node_put(intc_node);
// 	irqchip_init();
// }

// unsigned int get_c0_compare_int(void) {
// 	return CP0_LEGACY_COMPARE_IRQ;
// }

// IRQCHIP_DECLARE(mips_cpu_intc, "mti,cpu-interrupt-controller",
// 		mips_cpu_irq_of_init);

// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020-2021 Loongson Technology Corporation Limited
 */
#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/stddef.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/loongarchregs.h>
#include <loongson.h>

struct acpi_madt_lio_pic *acpi_liointc;
struct acpi_madt_eio_pic *acpi_eiointc;
struct acpi_madt_ht_pic *acpi_htintc;
struct acpi_madt_lpc_pic *acpi_pchlpc;
struct acpi_madt_msi_pic *acpi_pchmsi;
struct acpi_madt_bio_pic *acpi_pchpic[MAX_PCH_PICS];

struct fwnode_handle *acpi_liointc_handle;
struct fwnode_handle *acpi_msidomain_handle;
struct fwnode_handle *acpi_picdomain_handle[MAX_PCH_PICS];

void mach_irq_dispatch(unsigned int pending)
{
    if (pending & 0x800)
		do_IRQ(LOONGSON_TIMER_IRQ);
	if (pending & 0x20)
		do_IRQ(4);
	if (pending & 0x4)
		do_IRQ(LOONGSON_GMAC_IRQ) ; //in fact , it's for ehternet
	if (pending & 0x8)
		do_IRQ(LOONGSON_UART_IRQ);
}

asmlinkage void plat_irq_dispatch(void)
{
	unsigned int pending;
	pending = read_csr_estat() & read_csr_ecfg();
	/* machine-specific plat_irq_dispatch */
	mach_irq_dispatch(pending);
}

int find_pch_pic(u32 gsi)
{
	int i, start, end;

	/* Find the PCH_PIC that manages this GSI. */
	for (i = 0; i < loongson_sysconf.nr_pch_pics; i++) {
		struct acpi_madt_bio_pic *irq_cfg = acpi_pchpic[i];

		start = irq_cfg->gsi_base;
		end   = irq_cfg->gsi_base + irq_cfg->size;
		if (gsi >= start && gsi < end)
			return i;
	}

	pr_err("ERROR: Unable to locate PCH_PIC for GSI %d\n", gsi);
	return -1;
}

void __init setup_IRQ(void)
{
	irqchip_init();
}

void __init arch_init_irq(void)
{
	clear_csr_ecfg(ECFG0_IM);
	clear_csr_estat(ESTATF_IP);

	setup_IRQ();

	set_csr_ecfg(ECFGF_IP0 | ECFGF_IP1 |ECFGF_IP2 | ECFGF_IP3| ECFGF_IPI | ECFGF_PC);
}
