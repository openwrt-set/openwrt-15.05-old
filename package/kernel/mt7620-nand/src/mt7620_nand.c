#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define CTRL0	0x10
#define CMD1 	0x18 
#define CMD2 	0x1C
#define ADDR 	0x24
#define CFG  	0x14
#define DATA	0x28
#define STATUS	0x30
#define INT_STA	0x38
#define CTRL1	0x3C
#define ADDR2	0x60

struct mt7620_nand_platform_data {
	struct mtd_partition *parts;
	unsigned int num_parts;
};

struct mt7620_nand_info {
	struct mtd_info			mtd;
	struct nand_chip		chip;
	struct platform_device	*pdev;
	struct mt7620_nand_platform_data plat;

	void* __iomem			io_base;
	uint32_t 				databuf;
};

static int mt7620_nand_wait_ready(struct mtd_info *mtd, uint32_t timeout) {
	struct nand_chip *chip = mtd->priv;
	int retry = 10;

	while (retry--) {
		uint32_t status = readl(chip->IO_ADDR_R + STATUS);

		if( (status & 0x1) == 0 ) {
			schedule_timeout(timeout * HZ/1000);
			continue;
		}
		return 0;	
	}

	return NAND_STATUS_FAIL;
}

void mt7620_read_buf(struct mtd_info *mtd, uint8_t *buf, int len){
	struct nand_chip *chip = mtd->priv;

	int retry = 10, i,j;
	uint32_t data, status;
	pr_info("mt7620_nand_read_buf called with params: buf:0x%p len:0x%d\n", buf, len );
	data = readl(chip->IO_ADDR_R + CFG);
	data |= 0x1;
	writel(data, (chip->IO_ADDR_R + CFG));


	while(retry--) {
		status = readl(chip->IO_ADDR_R + INT_STA);
		if( status & 0x4 )
			break;

		schedule_timeout(10 * HZ/1000);
	}
	
	if(!retry)
		goto fail;

	j=0;
	for(i=0; i < len/4; i+=4) {
		data = readl(chip->IO_ADDR_R + DATA);
		buf[j++] = data & 0xff;
		buf[j++] = (data >> 8)  & 0xff;
		buf[j++] = (data >> 16) & 0xff;
		buf[j++] = (data >> 24) & 0xff;
	}

	data = readl(chip->IO_ADDR_R + DATA);
	for(i = 0; i < len % 4; i++) {
		buf[j++] = (data >> (i*8)) & 0xff;
	}

fail:
	return;
}

uint8_t mt7620_read_byte(struct mtd_info *mtd) {
	struct nand_chip *chip = mtd->priv;
	struct mt7620_nand_info *info = chip->priv;

	int retry = 10;
	uint32_t data, status;

	if( info->databuf == 0 ) {
		pr_info("read next word\n");
		status = readl(chip->IO_ADDR_R + INT_STA);

		while(retry--) {
			if( status & 0x4 )
				break;

			schedule_timeout(10 * HZ/1000);
		}

		if(!retry)
			goto fail;
		
		info->databuf = readl(chip->IO_ADDR_R + DATA);
	}

	data = info->databuf & 0xff;
	info->databuf = (info->databuf >> 8);

	pr_info("read DATA register: 0x%0X\n", data);

	return data;
fail:
	pr_err("read_byte_timeout\n");
	return 0;
}

static void mt7620_nand_dump_registers(struct mtd_info *mtd) {
	struct nand_chip *chip = mtd->priv;

	pr_info("---DUMP NFC REGISTERS---\n");
	pr_info("\tCTRL0:\t\t0x%08X\n", readl(chip->IO_ADDR_R + 0x10));
	pr_info("\tTRANS_CFG:\t0x%08X\n", readl(chip->IO_ADDR_R + 0x14));
	pr_info("\tCMD1:\t\t0x%08X\n", readl(chip->IO_ADDR_R + 0x18));
	pr_info("\tCMD2:\t\t0x%08X\n", readl(chip->IO_ADDR_R + 0x1C));
	pr_info("\tCMD3:\t\t0x%08X\n", readl(chip->IO_ADDR_R + 0x20));
	pr_info("\tADDR:\t\t0x%08X\n", readl(chip->IO_ADDR_R + 0x24));
	pr_info("\tDATA:\t\t0x%08X\n", readl(chip->IO_ADDR_R + 0x28));
	pr_info("\tSTATUS:\t\t0x%08X\n", readl(chip->IO_ADDR_R + 0x30));
	pr_info("\tINT_ENA:\t0x%08X\n", readl(chip->IO_ADDR_R + 0x34));
	pr_info("\tINT_STA:\t0x%08X\n", readl(chip->IO_ADDR_R + 0x38));
	pr_info("\tCTRL1:\t\t0x%08X\n", readl(chip->IO_ADDR_R + 0x3C));
	pr_info("\tECC_PAGE1:\t0x%08X\n", readl(chip->IO_ADDR_R + 0x40));
	pr_info("\tECC_PAGE2:\t0x%08X\n", readl(chip->IO_ADDR_R + 0x44));
	pr_info("\tECC_PAGE3:\t0x%08X\n", readl(chip->IO_ADDR_R + 0x48));
	pr_info("\tECC_PAGE4:\t0x%08X\n", readl(chip->IO_ADDR_R + 0x4C));
	pr_info("\tECC_ERR_PAGE1:\t0x%08X\n", readl(chip->IO_ADDR_R + 0x50));
	pr_info("\tECC_ERR_PAGE2:\t0x%08X\n", readl(chip->IO_ADDR_R + 0x54));
	pr_info("\tECC_ERR_PAGE3:\t0x%08X\n", readl(chip->IO_ADDR_R + 0x58));
	pr_info("\tECC_ERR_PAGE4:\t0x%08X\n", readl(chip->IO_ADDR_R + 0x5C));
	pr_info("\tADDR2:\t\t0x%08X\n", readl(chip->IO_ADDR_R + 0x60));
	pr_info("---DUMP END---\n");
}


static void mt7620_nand_cmdfunc(struct mtd_info *mtd, unsigned int command,
						int column, int page_addr)
{
	struct nand_chip *chip = mtd->priv;
	struct mt7620_nand_info *info = chip->priv;
	pr_info("mt7620_nand_cmdfunc called with params: CMD:0x%X COLUMN:0x%X PAGE_ADDR:0x%X\n", command, column, page_addr);
	info->databuf = 0;
//	mt7620_nand_dump_registers(mtd);
	switch(command) {
		case NAND_CMD_RESET:
			writel(0x00, 			(chip->IO_ADDR_R + CMD1));
			writel(NAND_CMD_RESET, 	(chip->IO_ADDR_R + CMD2));
			writel(0x00, 			(chip->IO_ADDR_R + ADDR));
			writel(0x0411, 			(chip->IO_ADDR_R + CFG ));
		break;
		case NAND_CMD_READID:
			writel(NAND_CMD_READID, 	(chip->IO_ADDR_R + CMD1));
			writel(0x00, 				(chip->IO_ADDR_R + CMD2));
			writel(column, 				(chip->IO_ADDR_R + ADDR));
			writel(0x0, 				(chip->IO_ADDR_R + ADDR2));
			writel(
					(1 << 0) 
					| (1 << 6)
					| (1 << 8)
					| (1 << 16)
					| (8 << 20)
					, (chip->IO_ADDR_R + CFG));			
		break;
		case NAND_CMD_READOOB:
			column += 0x800;
		case NAND_CMD_READ0:
			writel(NAND_CMD_READ0,		(chip->IO_ADDR_R + CMD1));
			writel(NAND_CMD_READSTART, 	(chip->IO_ADDR_R + CMD2));
			writel(page_addr << 16 | column, (chip->IO_ADDR_R + ADDR));
			writel(0x0, 	(chip->IO_ADDR_R + ADDR2));
			writel(
					(1 << 4)
					| (1 << 8)
					| (1 << 10)
					| (4 << 16)
					, (chip->IO_ADDR_R + CFG));			
			
		default:
		break;
	}

	if( mt7620_nand_wait_ready(mtd,100) ) {
		pr_err("nand cmdfunc timeout\n");
	}
//	mt7620_nand_dump_registers(mtd);
}

static void mt7620_nand_cmd_ctrl(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	pr_info("mt7620_nand_cmd_ctrl called with params: CMD:0x%X CTRL:0x%X\n", cmd, ctrl);
	return;
}

static int mt7620_nand_probe(struct platform_device *pdev)
{
	struct mt7620_nand_info *info;
	struct resource *res;
	struct mtd_part_parser_data ppdata = {};
	int ret;

	info = devm_kzalloc(&pdev->dev, sizeof(struct mt7620_nand_info), GFP_KERNEL);
	
	if (!info)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	info->io_base = devm_ioremap_resource(&pdev->dev, res);

	if(IS_ERR(info->io_base)) {
		ret = PTR_ERR(info->io_base);
		goto fail;
	}
	info->chip.priv = info;
	info->mtd.priv 	= &info->chip;
	info->mtd.owner = THIS_MODULE;
	info->mtd.name  = dev_name(&pdev->dev);

	info->chip.IO_ADDR_R	= info->io_base;
	info->chip.IO_ADDR_W	= info->io_base;
	info->chip.ecc.mode		= NAND_ECC_SOFT;
	info->chip.options 		= 0; //FIXIT
	info->chip.chip_delay	= 40;
	info->chip.cmd_ctrl 	= mt7620_nand_cmd_ctrl;
	info->chip.cmdfunc		= mt7620_nand_cmdfunc;
	info->chip.read_byte	= mt7620_read_byte;
	info->chip.read_buf		= mt7620_read_buf;
	info->databuf			= 0;
	writel(readl(info->io_base + CTRL1) | 0x1, info->io_base + CTRL1);
	writel(readl(info->io_base + CTRL0) | 0x2, info->io_base + CTRL0);
	writel(readl(info->io_base + CTRL0) & ~(0x2), info->io_base + CTRL0);
	writel(0x00000000, info->io_base + INT_STA);

	platform_set_drvdata(pdev, info);

	if( nand_scan(&info->mtd, 1) ) {
		ret = -ENXIO;
		goto fail;
	}

	ppdata.of_node = pdev->dev.of_node;
	ret = mtd_device_parse_register(&info->mtd, NULL, &ppdata,
				info->plat.parts,
				info->plat.num_parts);

	if( ret )
		goto fail;

	return 0;
fail:
	return ret;
}

static int mt7620_nand_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id mt7620_nand_id_table[] = {
		{ .compatible = "mt7620-nand" },
		{}
};
MODULE_DEVICE_TABLE(of, mt7620_nand_id_table);

static struct platform_driver mt7620_nand_driver = {
		.probe		= mt7620_nand_probe,
		.remove		= mt7620_nand_remove,
		.driver		= {
			.name	= "mt7620-nand",
			.owner	= THIS_MODULE,
			.of_match_table = of_match_ptr(mt7620_nand_id_table),
		},
};

module_platform_driver(mt7620_nand_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nikita Nazarenko <nnazarenko@radiofid.com>");
MODULE_DESCRIPTION("MT7620 NAND Driver");
