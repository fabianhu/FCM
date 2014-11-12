/*
 * spi.c General SPI initialisation
 *
 * Created: 21.02.2012 20:47:48
 *
 * (c) 2012-2014 by Fabian Huslik
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */  

#include <asf.h>
#include "types.h"
#include "spi.h"
#include "../FabOS_config.h"
#include "../FabOS/FabOS.h"

void spi_pdca_config(void);
void SPI_SetupDevice(uint8_t csel, uint32_t baud, uint8_t bitWidth, uint8_t mode, uint8_t CSHighBetweenTransfers);
__attribute__((__interrupt__))
static void dma_handler_TransferComplete0(void);

u_avr32_spi_tdr_t				SPI_cmdQ[SPI_QLEN]; // 32 bit wide and contains chip select data.
uint16_t /*vti_spiRX_ctrl_t*/	SPI_resQ[SPI_QLEN]; // 16 bit wide, contains only the received data.


uint32_t qfill=0;


//************************************
// Method:    SPI_AddQ8
// Returns:   uint32_t position in queue for next Element
// Parameter: uint8_t data
// Parameter: uint8_t cs
// Parameter: uint8_t last
//************************************
uint32_t SPI_AddQ8(uint8_t data, uint8_t cs, uint8_t last)
{
	SPI_cmdQ[qfill].b.td = data;
	SPI_cmdQ[qfill].b.lastxfer = last;
	SPI_cmdQ[qfill].b.pcs = cs;
	qfill++;
	return qfill;
}

//************************************
// Method:    SPI_AddQ16
// Returns:   uint32_t position in queue for next Element
// Parameter: uint8_t data
// Parameter: uint8_t cs
// Parameter: uint8_t last
//************************************
uint32_t SPI_AddQ16(uint16_t data, uint8_t cs, uint8_t last)
{
	SPI_cmdQ[qfill].b.td = data;
	SPI_cmdQ[qfill].b.lastxfer = last;
	SPI_cmdQ[qfill].b.pcs = cs;
	qfill++;
	return qfill;
}

void SPI_ResetQ(void)
{
	qfill=0;
}

void SPI_EndQ(void)
{
	if(qfill == 0) 
	{
		asm("breakpoint");
		OS_ShutdownHook(3); // no better idea
	}
	SPI_cmdQ[qfill-1].b.lastxfer=1;
}

void SPI_SetupDevice(uint8_t csel, uint32_t baud, uint8_t bitWidth, uint8_t mode, uint8_t CSHighBetweenTransfers)
{
	
	spi_set_chipselect_delay_bct(&AVR32_SPI,csel,CONFIG_SPI_MASTER_DELAY_BCT);
	spi_set_chipselect_delay_bs(&AVR32_SPI,csel,CONFIG_SPI_MASTER_DELAY_BS);
	spi_set_bits_per_transfer(&AVR32_SPI,csel,bitWidth);
	spi_set_baudrate_register(&AVR32_SPI,csel,getBaudDiv(baud, BOARD_SYS_HZ));
//	spi_disable_active_mode(&AVR32_SPI,csel);
	if(CSHighBetweenTransfers)
	{
		// only for CMR3000!
		//spi_enable_cs_high_between_trnsf(&AVR32_SPI,csel);
	}
	else
	{
		switch(csel) 
		{
			case 0:
			  AVR32_SPI.CSR0.csnaat = 0;
			  break;
			case 1:
			  AVR32_SPI.CSR1.csnaat = 0;
			  break;
			case 2:
			  AVR32_SPI.CSR2.csnaat = 0;
			  break;
			case 3:
			  AVR32_SPI.CSR3.csnaat = 0;
			  break;
		}
	}
	
	spi_set_mode(&AVR32_SPI, csel, mode);
}

void SPI_ReInit(void) 
{
		// ***************************************
	// init SPI	
	sysclk_enable_pba_module(SYSCLK_SPI);	
	spi_reset(&AVR32_SPI);
	spi_set_master_mode(&AVR32_SPI);
	spi_disable_modfault(&AVR32_SPI); // required by errata.
	spi_disable_loopback(&AVR32_SPI);
	spi_set_chipselect(&AVR32_SPI,(1 << AVR32_SPI_MR_PCS_SIZE) - 1);
	spi_enable_variable_chipselect(&AVR32_SPI); // activate chipsel control via tdr, which is transferred via DMA
	spi_disable_chipselect_decoding(&AVR32_SPI);
	spi_set_delay(&AVR32_SPI,CONFIG_SPI_MASTER_DELAY_BCS);
	
// configure devices HERE!!
//**********************************************	
	SPI_SetupDevice(CHIPSEL_L3GD20, 400000,8,3,0); //  400kHz for ST L3GD20 (max 10MHz) todo  // have to get 10 bytes at 400kHz
	// -> 40 kHz per byte; 5kHz with 8 bytes -> should be possible to keep up with the 760 Hz.
//**********************************************	

	spi_enable(&AVR32_SPI);	
}


void SPI_init(void)
{
	SPI_ReInit();

	
	spi_pdca_config();	
	
	pdca_enable(0);
	pdca_enable(1);
	
	INTC_register_interrupt(&dma_handler_TransferComplete0, AVR32_PDCA_IRQ_0, AVR32_INTC_INTLEVEL_INT2);
	pdca_enable_interrupt_transfer_complete(0);
}

void spi_pdca_config(void) 
{
	static const pdca_channel_options_t txopt =
	  {
		.addr = (void *)&SPI_cmdQ,                  // memory address
		.pid = AVR32_PDCA_PID_SPI_TX,             // select peripheral
		.size = 0,//sizeof(vti_cmdQ)/sizeof(vti_cmdQ[0]),                   // transfer counter
		.r_addr = (void *)&SPI_cmdQ,                           // next memory address
		.r_size = 0,                              // next transfer counter
		.transfer_size = PDCA_TRANSFER_SIZE_WORD  // select size of the transfer
	  };

	static const pdca_channel_options_t rxopt =
	  {
		.addr = (void *)&SPI_resQ,                  // memory address
		.pid = AVR32_PDCA_PID_SPI_RX,             // select peripheral
		.size = 0,//sizeof(vti_resQ)/ sizeof(vti_resQ[0]),                   // transfer counter
		.r_addr = (void *)&SPI_resQ,                           // next memory address
		.r_size = 0,								// next transfer counter
		.transfer_size = PDCA_TRANSFER_SIZE_HALF_WORD  // select size of the transfer
	  };

	  pdca_init_channel(1, &txopt); // init PDCA channel with options.
	  pdca_init_channel(0, &rxopt); // init PDCA channel with options.
	  

}

uint32_t debug_SPI_ovres;

void SPI_startqueue(void)
{
	avr32_spi_sr_t sr = AVR32_SPI.SR;
	
	if(sr.ovres)
		debug_SPI_ovres++;
	
	// start the DMA transfer	
	pdca_load_channel(1,SPI_cmdQ,qfill);//sizeof(SPI_cmdQ)/sizeof(SPI_cmdQ[0]));
	pdca_load_channel(0,SPI_resQ,qfill);//sizeof(SPI_resQ)/sizeof(SPI_resQ[0])); // fixme andersrum sinnvoller?
	
	AVR32_PDCA.channel[0].IER.trc=1; // enable ISR
	
}

vfpv_t spi_ready_fp;

void SPI_RegisterReadyHandler(vfpv_t fp)
{
	spi_ready_fp = fp;
}

__attribute__((__interrupt__))
static void dma_handler_TransferComplete0(void)
{
	AVR32_PDCA.channel[0].IDR.trc=1; // disable ISR
	// ISR if dma transfer is ready
	if(spi_ready_fp != NULL)
	(*spi_ready_fp)();
}