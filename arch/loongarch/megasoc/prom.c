#include <linux/of_fdt.h>
#include <asm/prom.h>

#define EARLY_PRINTK_UART_BASE 0xbfe40000
#define TRIVIALMIPS_UART_OUTB(addr, byte) writeb((byte), (uint8_t *)(EARLY_PRINTK_UART_BASE + addr))

void prom_putchar(char c)
{
    while( (readl((uint8_t *)(EARLY_PRINTK_UART_BASE + 0x5)) & 0x40) == 0 );
    TRIVIALMIPS_UART_OUTB(0x00, c & 0xFF);
}

void __init prom_init(void)
{
    // UART should have been initialized from U-Boot
}

void __init prom_free_prom_memory(void)
{
}
