/*
 * aic3212_spi.c
 *
 *  Created on: Jan 15, 2019
 *      Author: mehrabian
 */


#include "msfw.h"
#include LAYER(lld)
#include "../inc/aic3212.h"

static void aic3212_goto(struct aic3212_dev *dev, uint8_t book,uint8_t page);
static uint8_t aic3212_read_addr(struct aic3212_dev *dev, aic_addr addr);
static void aic3212_write_addr(struct aic3212_dev *dev, aic_addr addr,uint8_t data);

static void aic3212_goto(struct aic3212_dev *dev, uint8_t book,uint8_t page){
     if(dev->current_address.page != page){
        uint16_t data = AIC3212_SELECT_PAGE_REG << 1;
        data |= page << 8;
        dev->aic3212_spi_transceiver(dev, (uint8_t *)&data, (uint8_t *)&data, 2);
    }
    if(dev->current_address.book != book){
        uint16_t data = AIC3212_SELECT_BOOK_REG << 1;
        data |= book << 8;
        dev->aic3212_spi_transceiver(dev, (uint8_t *)&data, (uint8_t *)&data, 2);
    }
    dev->current_address.book = book;
    dev->current_address.page = page;
}

static uint8_t aic3212_read_addr(struct aic3212_dev *dev, aic_addr addr)
{
    uint16_t tx_rx_data = (addr.reg << 1) |  1;

    aic3212_goto(dev, addr.book,addr.page);
    dev->aic3212_spi_transceiver(dev, (uint8_t *)&tx_rx_data, (uint8_t *)&tx_rx_data, 2);
    return (uint8_t)((tx_rx_data >> 8) & 0xFF);
}

static void aic3212_write_addr(struct aic3212_dev *dev, aic_addr addr,uint8_t data)
{
    uint16_t tx_data = addr.reg << 1;
    tx_data |= data << 8;

    aic3212_goto(dev, addr.book,addr.page);
    dev->aic3212_spi_transceiver(dev, (uint8_t *)&tx_data, (uint8_t *)&tx_data, 2);
}

/***************************************************************************//**
 * @brief Initializes the device.
 *
 * @param device - Data structure for SPI functions (Depends to your machine)
 * @param init_param - The other parameters of SPI
 * @return status - Result of the initialization procedure.
 *                  Example: 0x0 - SPI peripheral not initialized.
 *                           0x1 - SPI peripheral initialized.
*******************************************************************************/
uint32_t aic3212_setup(struct aic3212_dev **device,
                          struct aic3212_init_param * init_param)
{
    uint8_t status = 0;
    struct aic3212_dev * dev;
    dev = (struct aic3212_dev *)malloc(sizeof(struct aic3212_dev));

    if(dev == NULL)
        return ~0UL;
    dev->ins = init_param->ins;
    dev->aic3212_spi_transceiver = init_param->aic3212_spi_transceiver;
    dev->aic3212_spi_remove = init_param->aic3212_spi_remove;
    *device = dev;
    init_param->aic3212_spi_init(dev);
    dev->current_address.book = 0xFF;
    dev->current_address.page = 0xFF;

    return status;
}

uint8_t aic3212_read(struct aic3212_dev *dev, uint32_t packed_addr)
{
    return aic3212_read_addr(dev, AIC_TO_ADDR(packed_addr));
}

void aic3212_write(struct aic3212_dev *dev, uint32_t packed_addr,uint8_t data)
{
    aic3212_write_addr(dev, AIC_TO_ADDR(packed_addr),data);
}

