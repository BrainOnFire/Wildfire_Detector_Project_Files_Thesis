#include "rfm95.h"
#include "main.h"

#include <assert.h>
#include <string.h>

#define RFM9x_VER 0x12

/**
 * Registers addresses.
 */
typedef enum
{
	RFM95_REGISTER_FIFO_ACCESS = 0x00,
	RFM95_REGISTER_OP_MODE = 0x01,
	RFM95_REGISTER_FR_MSB = 0x06,
	RFM95_REGISTER_FR_MID = 0x07,
	RFM95_REGISTER_FR_LSB = 0x08,
	RFM95_REGISTER_PA_CONFIG = 0x09,
	RFM95_REGISTER_LNA = 0x0C,
	RFM95_REGISTER_FIFO_ADDR_PTR = 0x0D,
	RFM95_REGISTER_FIFO_TX_BASE_ADDR = 0x0E,
	RFM95_REGISTER_FIFO_RX_BASE_ADDR = 0x0F,
	RFM95_REGISTER_IRQ_FLAGS_MASK = 0x11,
	RFM95_REGISTER_IRQ_FLAGS = 0x12,
	RFM95_REGISTER_MODEM_CONFIG_1 = 0x1D,
	RFM95_REGISTER_MODEM_CONFIG_2 = 0x1E,
	RFM95_REGISTER_PREAMBLE_MSB = 0x20,
	RFM95_REGISTER_PREAMBLE_LSB = 0x21,
	RFM95_REGISTER_PAYLOAD_LENGTH = 0x22,
	RFM95_REGISTER_MAX_PAYLOAD_LENGTH = 0x23,
	RFM95_REGISTER_MODEM_CONFIG_3 = 0x26,
	RFM95_REGISTER_INVERT_IQ_1 = 0x33,
	RFM95_REGISTER_SYNC_WORD = 0x39,
	RFM95_REGISTER_INVERT_IQ_2 = 0x3B,
	RFM95_REGISTER_DIO_MAPPING_1 = 0x40,
	RFM95_REGISTER_VERSION = 0x42,
	RFM95_REGISTER_PA_DAC = 0x4D
} rfm95_register_t;

typedef struct
{
	union {
		struct {
			uint8_t output_power : 4;
			uint8_t max_power : 3;
			uint8_t pa_select : 1;
		};
		uint8_t buffer;
	};
} rfm95_register_pa_config_t;

#define RFM95_REGISTER_OP_MODE_SLEEP                            0x00
#define RFM95_REGISTER_OP_MODE_LORA_SLEEP                       0x80
#define RFM95_REGISTER_OP_MODE_LORA_STANDBY                     0x81
#define RFM95_REGISTER_OP_MODE_LORA_TX                          0x83

#define RFM95_REGISTER_PA_DAC_LOW_POWER                         0x84
#define RFM95_REGISTER_PA_DAC_HIGH_POWER                        0x87

#define RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_TXDONE             0x40

#define RFM95_REGISTER_INVERT_IQ_1_TX                    		0x27
#define RFM95_REGISTER_INVERT_IQ_2_TX							0x1D

bool rfm95_read_register(rfm95_handle_t *handle, rfm95_register_t reg, uint8_t *buffer)
{
	HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_RESET);

	uint8_t transmit_buffer = (uint8_t)reg & 0x7Fu;

	if (HAL_SPI_Transmit(handle->spi_handle, &transmit_buffer, 1, RFM95_SPI_TIMEOUT) != HAL_OK) {
		return false;
	}

	if (HAL_SPI_Receive(handle->spi_handle, buffer, 1, RFM95_SPI_TIMEOUT) != HAL_OK) {
		return false;
	}

	HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_SET);

	return true;
}

bool rfm95_write_register(rfm95_handle_t *handle, rfm95_register_t reg, uint8_t value)
{
	HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_RESET);

	uint8_t transmit_buffer[2] = {((uint8_t)reg | 0x80u), value};

	if (HAL_SPI_Transmit(handle->spi_handle, transmit_buffer, 2, RFM95_SPI_TIMEOUT) != HAL_OK) {
		return false;
	}

	HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_SET);

	return true;
}

bool rfm95_init(rfm95_handle_t *handle)
{
	// Frequency to 915 MHz
	uint8_t version;
	uint32_t frequency = 915000000;
	// FQ = (FRF * 32 Mhz) / (2 ^ 19)
	uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

	assert(handle->spi_handle->Init.Mode == SPI_MODE_MASTER);
	assert(handle->spi_handle->Init.Direction == SPI_DIRECTION_2LINES);
	assert(handle->spi_handle->Init.DataSize == SPI_DATASIZE_8BIT);
	assert(handle->spi_handle->Init.CLKPolarity == SPI_POLARITY_LOW);
	assert(handle->spi_handle->Init.CLKPhase == SPI_PHASE_1EDGE);
	rfm95_reset(handle);

	// Check for correct version.
	if (!rfm95_read_register(handle, RFM95_REGISTER_VERSION, &version)) return false;
	if (version != RFM9x_VER) return false;

	// Module must be placed in sleep mode before switching to lora.
	if (!rfm95_write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_SLEEP)) return false;
	if (!rfm95_write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;

	// Frequency configuration (only in sleep mode)
	if (!rfm95_write_register(handle, RFM95_REGISTER_FR_MSB, (uint8_t)(frf >> 16))) return false;
	if (!rfm95_write_register(handle, RFM95_REGISTER_FR_MID, (uint8_t)(frf >> 8))) return false;
	if (!rfm95_write_register(handle, RFM95_REGISTER_FR_LSB, (uint8_t)(frf >> 0))) return false;

	// Default interrupt configuration, must be done to prevent DIO5 clock interrupts at 1Mhz
	// if (!rfm95_write_register(handle, RFM95_REGISTER_DIO_MAPPING_1, RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_RXDONE)) return false;

	// Set module power to 7dbm.
	if (!rfm95_set_power(handle, 7)) return false;

	// Set LNA to the highest gain and LNABoost 150%.
	if (!rfm95_write_register(handle, RFM95_REGISTER_LNA, 0x23)) return false;

	// Set IRQ Flag Mask for TX Done
	if (!rfm95_write_register(handle, RFM95_REGISTER_IRQ_FLAGS_MASK, 0x08)) return false;

	// Preamble set to 8 + 4.25 = 12.25 symbols.
	if (!rfm95_write_register(handle, RFM95_REGISTER_PREAMBLE_MSB, 0x00)) return false;
	if (!rfm95_write_register(handle, RFM95_REGISTER_PREAMBLE_LSB, 0x08)) return false;

	// Set TTN sync word 0x34.
	if (!rfm95_write_register(handle, RFM95_REGISTER_SYNC_WORD, 0x34)) return false;

	// Set up TX and RX FIFO base addresses.
	if (!rfm95_write_register(handle, RFM95_REGISTER_FIFO_TX_BASE_ADDR, 0x80)) return false;
	if (!rfm95_write_register(handle, RFM95_REGISTER_FIFO_RX_BASE_ADDR, 0x00)) return false;

	// Maximum payload length of the RFM95 is 64.
	if (!rfm95_write_register(handle, RFM95_REGISTER_MAX_PAYLOAD_LENGTH, 64)) return false;

	// Let module sleep after initialisation.
	if (!rfm95_write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;

	return true;
}

void rfm95_reset(rfm95_handle_t *handle)
{
	HAL_GPIO_WritePin(handle->nrst_port, handle->nrst_pin, GPIO_PIN_RESET);
	HAL_Delay(1); // 0.1ms would theoretically be enough
	HAL_GPIO_WritePin(handle->nrst_port, handle->nrst_pin, GPIO_PIN_SET);
	HAL_Delay(5);
}

bool rfm95_set_power(rfm95_handle_t *handle, int8_t power)
{
	assert((power >= 2 && power <= 17) || power == 20);

	rfm95_register_pa_config_t pa_config = {0};
	uint8_t pa_dac_config = 0;

	if (power >= 2 && power <= 17) {
		pa_config.max_power = 7;
		pa_config.pa_select = 1;
		pa_config.output_power = (power - 2);
		pa_dac_config = RFM95_REGISTER_PA_DAC_LOW_POWER;

	} else if (power == 20) {
		pa_config.max_power = 7;
		pa_config.pa_select = 1;
		pa_config.output_power = 15;
		pa_dac_config = RFM95_REGISTER_PA_DAC_HIGH_POWER;
	}

	if (!rfm95_write_register(handle, RFM95_REGISTER_PA_CONFIG, pa_config.buffer)) return false;
	if (!rfm95_write_register(handle, RFM95_REGISTER_PA_DAC, pa_dac_config)) return false;

	return true;
}

bool rfm95_send_package(rfm95_handle_t *handle, uint8_t *data, size_t length)
{
	// Configure channel for transmission (channel 0)
	//uint8_t channel = 0;

	// Check if packet length is shorter than 64
	assert(length <= 64);

	// TODO Configure modem (62kHz afecta la velocidad de transmision , 4/5 error coding rate, implicit header; SF7 *revisar SF* (not close to gateway, therefore, should be higher SF), single packet, CRC enable; LowDataRateOptimize = Disable, AgcAutoOn -> LNA Gain set by register LnaGain)
	if (!rfm95_write_register(handle, RFM95_REGISTER_MODEM_CONFIG_1, 0x63)) return false;
	if (!rfm95_write_register(handle, RFM95_REGISTER_MODEM_CONFIG_2, 0x75)) return false;
	if (!rfm95_write_register(handle, RFM95_REGISTER_MODEM_CONFIG_3, 0x00)) return false;

	// Set IQ registers to normal values according to AN1200.24.
	if (!rfm95_write_register(handle, RFM95_REGISTER_INVERT_IQ_1, RFM95_REGISTER_INVERT_IQ_1_TX)) return false;
	if (!rfm95_write_register(handle, RFM95_REGISTER_INVERT_IQ_2, RFM95_REGISTER_INVERT_IQ_2_TX)) return false;

	// Set the payload length
	if (!rfm95_write_register(handle, RFM95_REGISTER_PAYLOAD_LENGTH, length)) return false;

	// Enable tx-done interrupt, clear interrupt flags
	if (!rfm95_write_register(handle, RFM95_REGISTER_DIO_MAPPING_1, RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_TXDONE)) return false;
	if (!rfm95_write_register(handle, RFM95_REGISTER_IRQ_FLAGS, 0xFF)) return false;

	// TODO Wait for the modem to activate IRQ Tx Done bit
	//wait_for_irq(handle, RFM95_INTERRUPT_DIO5, RFM95_WAKEUP_TIMEOUT);

	// Set SPI pointer to start of TX section in FIFO (antes 0x80) verificar si esta correcto
	if (!rfm95_write_register(handle, RFM95_REGISTER_FIFO_ADDR_PTR, 0x80)) return false;

	// Write payload to FIFO.
	for (size_t i = 0; i < length; i++) {
		rfm95_write_register(handle, RFM95_REGISTER_FIFO_ACCESS, data[i]);
	}

	// TODO Reset IRQ Register


	// Mode Request TX
	if (!rfm95_write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_TX)) return false;

	// Wait for the transfer complete interrupt.
	while(G0_LORA_INT_Pin == 0)
	{}

	// Return modem to sleep
	if (!rfm95_write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;

	return true;
}
