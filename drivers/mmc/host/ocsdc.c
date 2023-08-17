/*
 * ocsdc.c
 *
 * Copyright (C) 2013 Marek Czerski
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Written by Marek Czerski <ma.czerski@gmail.com>
 */

// #define DEBUG
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mmc/host.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>

// Register space
#define OCSDC_ARGUMENT 0x00
#define OCSDC_COMMAND 0x04
#define OCSDC_RESPONSE_1 0x08
#define OCSDC_RESPONSE_2 0x0c
#define OCSDC_RESPONSE_3 0x10
#define OCSDC_RESPONSE_4 0x14
#define OCSDC_DATA_TIMEOUT 0x18
#define OCSDC_CONTROL 0x1C
#define OCSDC_CMD_TIMEOUT 0x20
#define OCSDC_CLOCK_DIVIDER 0x24
#define OCSDC_SOFTWARE_RESET 0x28
#define OCSDC_POWER_CONTROL 0x2C
#define OCSDC_CAPABILITY 0x30
#define OCSDC_CMD_INT_STATUS 0x34
#define OCSDC_CMD_INT_ENABLE 0x38
#define OCSDC_DAT_INT_STATUS 0x3C
#define OCSDC_DAT_INT_ENABLE 0x40
#define OCSDC_BLOCK_SIZE 0x44
#define OCSDC_BLOCK_COUNT 0x48
#define OCSDC_DST_SRC_ADDR 0x60

//command register
#define OCSDC_COMMAND_NO_RESP 0x0
#define OCSDC_COMMAND_RESP_48 0x1
#define OCSDC_COMMAND_RESP_136 0x2
#define OCSDC_COMMAND_BUSY_CHECK 0x4
#define OCSDC_COMMAND_CRC_CHECK 0x8
#define OCSDC_COMMAND_INDEX_CHECK 0x10
#define OCSDC_COMMAND_DATA_READ 0x20
#define OCSDC_COMMAND_DATA_WRITE 0x40
#define OCSDC_COMMAND_INDEX(x) ((x) << 8)

// OCSDC_CMD_INT_STATUS bits
#define OCSDC_CMD_INT_STATUS_CC 0x0001
#define OCSDC_CMD_INT_STATUS_EI 0x0002
#define OCSDC_CMD_INT_STATUS_CTE 0x0004
#define OCSDC_CMD_INT_STATUS_CCRC 0x0008
#define OCSDC_CMD_INT_STATUS_CIE 0x0010
#define OCSDC_CMD_INT_ENABLE_ALL 0x001F

// SDCMSC_DAT_INT_STATUS
#define OCSDC_DAT_INT_STATUS_CC 0x01
#define OCSDC_DAT_INT_STATUS_EI 0x02
#define OCSDC_DAT_INT_STATUS_CTE 0x04
#define OCSDC_DAT_INT_STATUS_CCRC 0x08
#define OCSDC_DAT_INT_STATUS_CFE 0x10
#define OCSDC_DAT_INT_ENABLE_ALL 0x1F

struct ocsdc_dev {
	void __iomem *iobase;
	int irq_cmd;
	int irq_data;
	unsigned int clk_freq;
	struct mmc_request *curr_mrq;
	struct mmc_command *curr_cmd;
	struct mmc_data *curr_data;
};

static inline uint32_t ocsdc_read(struct ocsdc_dev *dev, int offset)
{
#ifdef CONFIG_WISHBONE_BUS_BIG_ENDIAN
	return ioread32be(dev->iobase + offset);
#else
	return ioread32(dev->iobase + offset);
#endif
}

static inline void ocsdc_write(struct ocsdc_dev *dev, int offset, uint32_t data)
{
#ifdef CONFIG_WISHBONE_BUS_BIG_ENDIAN
	iowrite32be(data, dev->iobase + offset);
#else
	iowrite32(data, dev->iobase + offset);
#endif
}

static u32 ocsdc_get_voltage(struct ocsdc_dev *dev)
{
	u32 v = ocsdc_read(dev, OCSDC_POWER_CONTROL);
	u32 voltage = 0;
	if (v >= 1650 && v <= 1950)
		voltage |= MMC_VDD_165_195;
	if (v >= 2000 && v <= 2100)
		voltage |= MMC_VDD_20_21;
	if (v >= 2100 && v <= 2200)
		voltage |= MMC_VDD_21_22;
	if (v >= 2200 && v <= 2300)
		voltage |= MMC_VDD_22_23;
	if (v >= 2300 && v <= 2400)
		voltage |= MMC_VDD_23_24;
	if (v >= 2400 && v <= 2500)
		voltage |= MMC_VDD_24_25;
	if (v >= 2500 && v <= 2600)
		voltage |= MMC_VDD_25_26;
	if (v >= 2600 && v <= 2700)
		voltage |= MMC_VDD_26_27;
	if (v >= 2700 && v <= 2800)
		voltage |= MMC_VDD_27_28;
	if (v >= 2800 && v <= 2900)
		voltage |= MMC_VDD_28_29;
	if (v >= 2900 && v <= 3000)
		voltage |= MMC_VDD_29_30;
	if (v >= 3000 && v <= 3100)
		voltage |= MMC_VDD_30_31;
	if (v >= 3100 && v <= 3200)
		voltage |= MMC_VDD_31_32;
	if (v >= 3200 && v <= 3300)
		voltage |= MMC_VDD_32_33;
	if (v >= 3300 && v <= 3400)
		voltage |= MMC_VDD_33_34;
	if (v >= 3400 && v <= 3500)
		voltage |= MMC_VDD_34_35;
	if (v >= 3500 && v <= 3600)
		voltage |= MMC_VDD_35_36;
	return voltage;
}

/* Set clock divider value based on the required clock in HZ */
static void ocsdc_set_clock(struct ocsdc_dev *dev, unsigned int clock)
{
	int clk_div = DIV_ROUND_UP(dev->clk_freq, (2 * clock)) - 1;
	if (clk_div < 0)
		clk_div = 0;

	ocsdc_write(dev, OCSDC_SOFTWARE_RESET, 1);
	ocsdc_write(dev, OCSDC_CLOCK_DIVIDER, clk_div);
	ocsdc_write(dev, OCSDC_SOFTWARE_RESET, 0);
}

/* Initialize ocsdc controller */
static int ocsdc_init(struct ocsdc_dev *dev)
{
	ocsdc_write(dev, OCSDC_CMD_TIMEOUT, 0x7FFF);
	ocsdc_write(dev, OCSDC_DATA_TIMEOUT, 0xFFFFFF);
	ocsdc_write(dev, OCSDC_CMD_INT_ENABLE, 0);
	ocsdc_write(dev, OCSDC_DAT_INT_ENABLE, 0);
	ocsdc_write(dev, OCSDC_CMD_INT_STATUS, 0);
	ocsdc_write(dev, OCSDC_DAT_INT_STATUS, 0);
	/* set clock to maximum */
	ocsdc_set_clock(dev, dev->clk_freq / 2);

	return 0;
}

static void ocsdc_set_buswidth(struct ocsdc_dev *dev, unsigned char width)
{
	if (width == MMC_BUS_WIDTH_4)
		ocsdc_write(dev, OCSDC_CONTROL, 1);
	else if (width == MMC_BUS_WIDTH_1)
		ocsdc_write(dev, OCSDC_CONTROL, 0);
}

static uint32_t get_timeout_reg_val(struct ocsdc_dev *dev, unsigned int t_ns,
				    unsigned int t_clks)
{
	uint32_t clkd;
	uint32_t timeout_ms;
	uint32_t freq_kHz;
	uint32_t timeout_clks;

	clkd = 2 * (ocsdc_read(dev, OCSDC_CLOCK_DIVIDER) + 1);
	timeout_ms = 1 + t_ns / 1000000;
	freq_kHz = 1 + dev->clk_freq / (1000 * clkd);
	timeout_clks = t_clks + timeout_ms * freq_kHz;
	if (timeout_clks > 0xFFFFFF)
		timeout_clks = 0xFFFFFF;
	return timeout_clks;
}

static void ocsdc_setup_data_xfer(struct mmc_host *mmc, struct mmc_data *data)
{
	struct ocsdc_dev *dev = mmc_priv(mmc);

	uint32_t timeout;

	dev_dbg(mmc_dev(mmc), "sg_len %d, next sg %p, addr %x\n", data->sg_len,
		sg_next(data->sg), sg_dma_address(data->sg));
	dev_dbg(mmc_dev(mmc), "%d blksize %x, blocks %x\n",
		(data->flags & MMC_DATA_WRITE) ? 1 : 0, data->blksz,
		data->blocks);
	dma_map_sg(mmc_dev(mmc), data->sg, data->sg_len,
		   ((data->flags & MMC_DATA_WRITE) ? DMA_TO_DEVICE :
						     DMA_FROM_DEVICE));
	ocsdc_write(dev, OCSDC_DST_SRC_ADDR, sg_dma_address(data->sg));
	ocsdc_write(dev, OCSDC_BLOCK_SIZE, data->blksz - 1);
	ocsdc_write(dev, OCSDC_BLOCK_COUNT, data->blocks - 1);

	//setup timeout
	timeout =
		get_timeout_reg_val(dev, data->timeout_ns, data->timeout_clks);
	ocsdc_write(dev, OCSDC_DATA_TIMEOUT, timeout);
}

static uint32_t ocsdc_prepare_cmd(struct ocsdc_dev *dev,
				  struct mmc_command *cmd,
				  struct mmc_data *data)
{
	uint32_t command = OCSDC_COMMAND_INDEX(cmd->opcode);
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			command |= OCSDC_COMMAND_RESP_136;
		else {
			command |= OCSDC_COMMAND_RESP_48;
		}
	}
	if (cmd->flags & MMC_RSP_BUSY)
		command |= OCSDC_COMMAND_BUSY_CHECK;
	if (cmd->flags & MMC_RSP_CRC)
		command |= OCSDC_COMMAND_CRC_CHECK;
	if (cmd->flags & MMC_RSP_OPCODE)
		command |= OCSDC_COMMAND_INDEX_CHECK;

	if (data &&
	    ((data->flags & MMC_DATA_READ) ||
	     ((data->flags & MMC_DATA_WRITE))) &&
	    data->blocks) {
		if (data->flags & MMC_DATA_READ)
			command |= OCSDC_COMMAND_DATA_READ;
		if (data->flags & MMC_DATA_WRITE)
			command |= OCSDC_COMMAND_DATA_WRITE;
	}
	return command;
}

static void ocsdc_start_cmd(struct ocsdc_dev *dev, struct mmc_command *cmd,
			    struct mmc_data *data)
{
	unsigned int command = ocsdc_prepare_cmd(dev, cmd, data);
	dev->curr_cmd = cmd;
	dev->curr_data = data;
	ocsdc_write(dev, OCSDC_COMMAND, command);
	ocsdc_write(dev, OCSDC_ARGUMENT, cmd->arg);
	ocsdc_write(dev, OCSDC_CMD_INT_ENABLE, OCSDC_CMD_INT_ENABLE_ALL);
}

static void ocsdc_end_request(struct mmc_host *mmc)
{
	struct ocsdc_dev *dev = mmc_priv(mmc);
	mmc_request_done(mmc, dev->curr_mrq);
	dev->curr_mrq = NULL;
}

static irqreturn_t ocsdc_irq_cmd(int irq, void *devid)
{
	struct mmc_host *mmc = devid;
	struct ocsdc_dev *dev = mmc_priv(mmc);
	struct mmc_command *cmd = dev->curr_cmd;
	struct mmc_data *data = dev->curr_data;

	uint32_t status = ocsdc_read(dev, OCSDC_CMD_INT_STATUS);
	if (status & OCSDC_CMD_INT_STATUS_EI) {
		if (status & OCSDC_CMD_INT_STATUS_CTE) {
			cmd->error = -ETIMEDOUT;
			dev_warn(mmc_dev(mmc), "Timeout error!\n");
		} else if (status & OCSDC_CMD_INT_STATUS_CCRC) {
			cmd->error = -EILSEQ;
			dev_warn(mmc_dev(mmc), "CRC error!\n");
		} else if (status & OCSDC_CMD_INT_STATUS_CIE) {
			cmd->error = -EILSEQ;
			dev_warn(mmc_dev(mmc), "Index error error!\n");
		}
	} else if (status & OCSDC_CMD_INT_STATUS_CC) {
		dev_dbg(mmc_dev(mmc), "Transfer complete!\n");
		cmd->resp[0] = ocsdc_read(dev, OCSDC_RESPONSE_1);
		if (cmd->flags & MMC_RSP_136) {
			cmd->resp[1] = ocsdc_read(dev, OCSDC_RESPONSE_2);
			cmd->resp[2] = ocsdc_read(dev, OCSDC_RESPONSE_3);
			cmd->resp[3] = ocsdc_read(dev, OCSDC_RESPONSE_4);
		}
	} else if (!status) {
		dev_warn(mmc_dev(mmc), "Spurious interrupt\n");
		return IRQ_NONE;
	} else {
		dev_err(mmc_dev(mmc), "Wrong cmd interrupt status 0x%x\n",
			status);
		return IRQ_NONE;
	}

	ocsdc_write(dev, OCSDC_CMD_INT_ENABLE, 0);
	ocsdc_write(dev, OCSDC_CMD_INT_STATUS, 0);

	dev->curr_cmd = NULL;

	if (cmd->error || !data || !data->blocks) {
		ocsdc_end_request(mmc);
		dev_dbg(mmc_dev(mmc), "End of req\n");
	} else {
		/* waiting for data interrupt */
		ocsdc_write(dev, OCSDC_DAT_INT_ENABLE,
			    OCSDC_DAT_INT_ENABLE_ALL);
		dev_dbg(mmc_dev(mmc), "Waiting for data int\n");
	}
	return IRQ_HANDLED;
}

static irqreturn_t ocsdc_irq_data(int irq, void *devid)
{
	struct mmc_host *mmc = devid;
	struct ocsdc_dev *dev = mmc_priv(mmc);
	struct mmc_data *data = dev->curr_data;

	uint32_t status = ocsdc_read(dev, OCSDC_DAT_INT_STATUS);

	if (status & OCSDC_DAT_INT_STATUS_CC) {
		data->bytes_xfered = data->blocks * data->blksz;
	} else {
		if (status & OCSDC_DAT_INT_STATUS_CTE) {
			data->error = -ETIMEDOUT;
			dev_warn(mmc_dev(mmc), "Data timeout error!\n");
		} else if (status & OCSDC_DAT_INT_STATUS_CCRC) {
			data->error = -EILSEQ;
			dev_warn(mmc_dev(mmc), "Data CRC error!\n");
		} else if (status & OCSDC_DAT_INT_STATUS_CFE) {
			data->error = -EILSEQ;
			dev_warn(mmc_dev(mmc), "Data FIFO error!\n");
		} else {
			dev_err(mmc_dev(mmc),
				"Wrong data interrupt status 0x%x\n", status);
			return IRQ_NONE;
		}
	}

	ocsdc_write(dev, OCSDC_DAT_INT_ENABLE, 0);
	ocsdc_write(dev, OCSDC_DAT_INT_STATUS, 0);

	dma_unmap_sg(mmc_dev(mmc), data->sg, data->sg_len,
		     (data->flags & MMC_DATA_WRITE) ? DMA_TO_DEVICE :
						      DMA_FROM_DEVICE);
	dev->curr_data = NULL;

	if (data->error || !dev->curr_mrq->stop) {
		ocsdc_end_request(mmc);
	} else {
		ocsdc_start_cmd(dev, dev->curr_mrq->stop, NULL);
	}
	return IRQ_HANDLED;
}

static void ocsdc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct ocsdc_dev *dev = mmc_priv(mmc);

	if (dev->curr_mrq)
		goto ERROR;
	if (!mrq->cmd)
		goto ERROR;
	if (mrq->stop && !mrq->data)
		goto ERROR;
	if (mrq->sbc)
		goto ERROR;

	if (mrq->data && mrq->data->blocks && mrq->data->blksz)
		ocsdc_setup_data_xfer(mmc, mrq->data);

	dev->curr_mrq = mrq;

	ocsdc_start_cmd(dev, mrq->cmd, mrq->data);

	return;

ERROR:
	mrq->cmd->error = -EIO;
	mmc_request_done(mmc, mrq);
}

static void ocsdc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct ocsdc_dev *dev = mmc_priv(mmc);

	if (ios->power_mode != MMC_POWER_ON)
		return;

	if (ios->clock)
		ocsdc_set_clock(dev, ios->clock);

	ocsdc_set_buswidth(dev, ios->bus_width);
}

static void ocsdc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
}

static const struct mmc_host_ops ocsdc_ops = { .request = ocsdc_request,
					       .set_ios = ocsdc_set_ios,
					       .get_cd = NULL,
					       .get_ro = NULL,
					       .enable_sdio_irq =
						       ocsdc_enable_sdio_irq };

static int ocsdc_probe(struct platform_device *pdev)
{
	int ret;
	struct mmc_host *mmc;
	struct ocsdc_dev *dev;
	struct resource *res;
	struct clk *clk;

	mmc = mmc_alloc_host(sizeof(struct ocsdc_dev), &pdev->dev);
	if (!mmc) {
		dev_err(&pdev->dev, "Failed to allocate mmc_host");
		return -ENOMEM;
	}

	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "Failed to get clock for ocsdc");
		ret = PTR_ERR(clk);
		goto ERROR;
	}
	clk_prepare_enable(clk);

	dev = mmc_priv(mmc);
	dev->clk_freq = clk_get_rate(clk);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot obtain I/O memory space\n");
		ret = -ENXIO;
		goto ERROR;
	}

	dev->iobase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dev->iobase)) {
		dev_err(&pdev->dev, "cannot request I/O memory space\n");
		ret = PTR_ERR(dev->iobase);
		goto ERROR;
	}

	dev->irq_cmd = platform_get_irq(pdev, 0);
	dev->irq_data = platform_get_irq(pdev, 1);
	if (dev->irq_cmd < 0 || dev->irq_data < 0) {
		ret = -ENXIO;
		goto ERROR;
	}

	ocsdc_init(dev);

	ret = devm_request_irq(&pdev->dev, dev->irq_cmd, ocsdc_irq_cmd, 0,
			       mmc_hostname(mmc), mmc);
	if (ret)
		goto ERROR;

	ret = devm_request_irq(&pdev->dev, dev->irq_data, ocsdc_irq_data, 0,
			       mmc_hostname(mmc), mmc);
	if (ret)
		goto ERROR;

	mmc->ops = &ocsdc_ops;
	mmc->f_min = dev->clk_freq / 8;
	mmc->f_max = dev->clk_freq / 6;// limit clock rate to prevent fifo underrun
	mmc->caps = MMC_CAP_4_BIT_DATA | MMC_CAP_NONREMOVABLE;
	mmc->caps2 = 0;
	mmc->max_segs = 1;
	mmc->max_blk_size = 1 << 12;
	mmc->max_blk_count = 1 << 16;
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_seg_size = mmc->max_req_size;
	mmc->ocr_avail = ocsdc_get_voltage(dev);

	ret = mmc_add_host(mmc);
	if (ret < 0)
		goto ERROR;

	platform_set_drvdata(pdev, mmc);
	printk("ocsdc probe ok!\n");
	dev_dbg(&pdev->dev, "ocsdc probe ok\n");
	return 0;

ERROR:
	mmc_free_host(mmc);
	return ret;
}

static int ocsdc_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	if (mmc) {
		mmc_remove_host(mmc);
		mmc_free_host(mmc);
	}
	return 0;
}

#ifdef CONFIG_PM
static int ocsdc_suspend(struct platform_device *pdev, pm_message_t state)
{
	return -ENOSYS;
}

static int ocsdc_resume(struct platform_device *pdev)
{
	return -ENOSYS;
}
#else
#define ocsdc_suspend NULL
#define ocsdc_resume NULL
#endif

static struct of_device_id ocsdc_match[] = {
	{ .compatible = "opencores,ocsdc" },
	{},
};
MODULE_DEVICE_TABLE(of, ocsdc_match);

static struct platform_driver ocsdc_driver = {
	.probe   = ocsdc_probe,
	.remove  = ocsdc_remove,
	.suspend = ocsdc_suspend,
	.resume  = ocsdc_resume,
	.driver  = {
		.name = "ocsdc",
		.owner = THIS_MODULE,
		.of_match_table = ocsdc_match,
	},
};

module_platform_driver(ocsdc_driver);

MODULE_AUTHOR("Marek Czerski");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("OpenCores SD Card Controller IP Core driver");
