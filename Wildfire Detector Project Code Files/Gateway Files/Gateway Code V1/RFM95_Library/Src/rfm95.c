#include "rfm95.h"
#include "main.h"

#include <assert.h>
#include <string.h>

#define RFM9x_VER 0x12

uint8_t debug;
uint8_t irqFlags;

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
	RFM95_REGISTER_OCP = 0x0B,
	RFM95_REGISTER_FIFO_ADDR_PTR = 0x0D,
	RFM95_REGISTER_FIFO_TX_BASE_ADDR = 0x0E,
	RFM95_REGISTER_FIFO_RX_BASE_ADDR = 0x0F,
	RFM95_REGISTER_FIFO_RX_CURRENT_ADDR = 0x10,
	RFM95_REGISTER_IRQ_FLAGS_MASK = 0x11,
	RFM95_REGISTER_IRQ_FLAGS = 0x12,
	RFM95_REGISTER_RX_NB_BYTES = 0x13,
	RFM95_REGISTER_PKT_SNR_VALUE = 0x19,
	RFM95_REGISTER_PKT_RSSI_VALUE = 0x1A,
	RFM95_REGISTER_MODEM_CONFIG_1 = 0x1D,
	RFM95_REGISTER_MODEM_CONFIG_2 = 0x1E,
	RFM95_REGISTER_PREAMBLE_MSB = 0x20,
	RFM95_REGISTER_PREAMBLE_LSB = 0x21,
	RFM95_REGISTER_PAYLOAD_LENGTH = 0x22,
	RFM95_REGISTER_MAX_PAYLOAD_LENGTH = 0x23,
	RFM95_REGISTER_MODEM_CONFIG_3 = 0x26,
	RFM95_REGISTER_RSSI_WIDEBAND = 0x2C,
	REFM95_REGISTER_DETECTION_OPTIMIZE = 0x31,
	RFM95_REGISTER_INVERT_IQ_1 = 0x33,
	REFM95_REGISTER_DETECTION_THRESHOLD = 0x37,
	RFM95_REGISTER_SYNC_WORD = 0x39,
	RFM95_REGISTER_INVERT_IQ_2 = 0x3B,
	RFM95_REGISTER_DIO_MAPPING_1 = 0x40,
	RFM95_REGISTER_DIO_MAPPING_2 = 0x41,
	RFM95_REGISTER_VERSION = 0x42,
	RFM95_REGISTER_PA_DAC = 0x4D
} rfm95_register_t;

/*
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
*/

// Modes
#define RFM95_REGISTER_OP_MODE_SLEEP                            0x00
#define RFM95_REGISTER_OP_MODE_LORA_SLEEP                       0x80
#define RFM95_REGISTER_OP_MODE_LORA_STANDBY                     0x81
#define RFM95_REGISTER_OP_MODE_LORA_TX                          0x83
#define RFM95_REGISTER_OP_MODE_RX_CONTINUOS						0x85
#define RFM95_REGISTER_OP_MODE_RX_SINGLE						0x86

// PA Config
#define PA_BOOST												0x80

// DAC Power
//#define RFM95_REGISTER_PA_DAC_LOW_POWER                         0x84
//#define RFM95_REGISTER_PA_DAC_HIGH_POWER                        0x87

// IRQ Masks
//#define RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_TXDONE             	0x40
//#define RFM95_REGISTER_DIO_MAPPING_1_IRQ_PAYLOAD_CRC_ERROR_MASK	0x20
//#define RFM95_REGISTER_DIO_MAPPING_1_IRQ_RX_DONE_MASK				0x40

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
	uint8_t lna_gain;
	//uint8_t modemConfig1;
	//uint8_t modemConfig2;
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
	//rfm95_read_register(handle, RFM95_REGISTER_OP_MODE, &debug);
	if (!rfm95_write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;
	//rfm95_read_register(handle, RFM95_REGISTER_OP_MODE, &debug);

	// Frequency configuration (only in sleep mode)
	if (!rfm95_write_register(handle, RFM95_REGISTER_FR_MSB, (uint8_t)(frf >> 16))) return false;
	//rfm95_read_register(handle, RFM95_REGISTER_FR_MSB, &debug);
	if (!rfm95_write_register(handle, RFM95_REGISTER_FR_MID, (uint8_t)(frf >> 8))) return false;
	//rfm95_read_register(handle, RFM95_REGISTER_FR_MID, &debug);
	if (!rfm95_write_register(handle, RFM95_REGISTER_FR_LSB, (uint8_t)(frf >> 0))) return false;
	//rfm95_read_register(handle, RFM95_REGISTER_FR_LSB, &debug);

	// Set up TX and RX FIFO base addresses.
	//if (!rfm95_write_register(handle, RFM95_REGISTER_FIFO_TX_BASE_ADDR, 0x00)) return false;
	if (!rfm95_write_register(handle, RFM95_REGISTER_FIFO_RX_BASE_ADDR, 0x00)) return false;
	//rfm95_read_register(handle, RFM95_REGISTER_FIFO_RX_BASE_ADDR, &debug);

	// Set LNA to the highest gain and LNABoost 150%.
	if (!rfm95_read_register(handle, RFM95_REGISTER_LNA, &lna_gain)) return false;
	if (!rfm95_write_register(handle, RFM95_REGISTER_LNA, lna_gain | 0x03)) return false; //Verificar por si no manda
	//rfm95_read_register(handle, RFM95_REGISTER_LNA, &debug);

	// Configure modem 2 LowDataRateOptimize = Disable, AgcAutoOn -> LNA Gain set by register LnaGain)
	// if (!rfm95_write_register(handle, RFM95_REGISTER_MODEM_CONFIG_1, 0x63)) return false; // (62kHz afecta la velocidad de transmision , 4/5 error coding rate, implicit header)
	// {Set implicitheader -> codigo temporal, ideal mejorar para optimizar;
	// if (rfm95_read_register(handle, RFM95_REGISTER_MODEM_CONFIG_1, &modemConfig1));
	if (!rfm95_write_register(handle, RFM95_REGISTER_MODEM_CONFIG_1, 0x72)) return false; // 125kHz BW, 4/5 CR, Explicit Header)
	//rfm95_read_register(handle, RFM95_REGISTER_MODEM_CONFIG_1, &debug);
	//if (!rfm95_write_register(handle, RFM95_REGISTER_MODEM_CONFIG_1, 0x62)) return false; // (62kHz afecta la velocidad de transmision , 4/5 error coding rate, explicit header)
	if (!rfm95_write_register(handle, RFM95_REGISTER_MODEM_CONFIG_2, 0x70)) return false; // SF7, TxContinuous Mode = False, RxPayloadCrcOn = Disable, SymbTimeout = 0
	//}

	// if (!rfm95_write_register(handle, RFM95_REGISTER_MODEM_CONFIG_2, 0x75)) return false; // SF7 *revisar SF* (not close to gateway, therefore, should be higher SF), single packet, CRC enable;
	if (!rfm95_write_register(handle, RFM95_REGISTER_MODEM_CONFIG_3, 0x04)) return false; // LowDataRateOptimize = Disabled, AgcAutoOn =  LNA gain set by internal AGC loop
	//rfm95_read_register(handle, RFM95_REGISTER_MODEM_CONFIG_3, &debug);

	// Set module power to 20dbm.
	// if (!rfm95_set_power(handle, 20)) return false;
	if (!rfm95_write_register(handle, RFM95_REGISTER_PA_CONFIG, 0x92)) return false; // Corregir en caso este mal
	//rfm95_read_register(handle, RFM95_REGISTER_PA_CONFIG, &debug);
	// if (!rfm95_write_register(handle, RFM95_REGISTER_PA_DAC, 0x87)) return false; // Segun datasheet
	// rfm95_read_register(handle, RFM95_REGISTER_PA_DAC, &debug);

	// Ovefcurrent protection (OcpO; OcpTrim)
	// if (!rfm95_write_register(handle, RFM95_REGISTER_OCP, 0x1C)) return false;

	// (Optional IRQ Mask) Set IRQ Flag Mask for TX Done
	// if (!rfm95_write_register(handle, RFM95_REGISTER_IRQ_FLAGS_MASK, 0x50)) return false;
	//rfm95_read_register(handle, RFM95_REGISTER_IRQ_FLAGS_MASK, &debug);

	//RX INIT
	// Read IRQs
	if (!rfm95_read_register(handle, RFM95_REGISTER_IRQ_FLAGS, &irqFlags)) return false;

	//Clear IRQs
	if (!rfm95_write_register(handle, RFM95_REGISTER_IRQ_FLAGS, 0xFF)) return false; // Retorna 0x00 lo cual es correcto porque al enviar 0xFF se limpian las banderas
	//rfm95_read_register(handle, RFM95_REGISTER_IRQ_FLAGS, &debug);

	// Let module stand-by after initialization.
	if (!rfm95_write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_STANDBY)) return false;
	//rfm95_read_register(handle, RFM95_REGISTER_OP_MODE, &debug);

	return true;
}

void rfm95_reset(rfm95_handle_t *handle)
{
	HAL_GPIO_WritePin(handle->nrst_port, handle->nrst_pin, GPIO_PIN_RESET);
	HAL_Delay(1); // 0.1ms would theoretically be enough
	HAL_GPIO_WritePin(handle->nrst_port, handle->nrst_pin, GPIO_PIN_SET);
	HAL_Delay(5);
}

/*
bool rfm95_set_power(rfm95_handle_t *handle, int8_t power) // TODO revisar
{
	assert((power >= 2 && power <= 17) || power == 20);

	rfm95_register_pa_config_t pa_config = {0};
	uint8_t pa_dac_config = 0;

	if (power >= 2 && power <= 17) {
		pa_config.max_power = 7;
		pa_config.pa_select = 1;
		pa_config.output_power = (power - 2);
		pa_dac_config = RFM95_REGISTER_PA_DAC_HIGH_POWER;

	} else if (power == 20) {
		pa_config.max_power = 7;
		pa_config.pa_select = 1;
		pa_config.output_power = 15;
		pa_dac_config = RFM95_REGISTER_PA_DAC_HIGH_POWER;
	}

	if (!rfm95_write_register(handle, RFM95_REGISTER_PA_CONFIG, 0x92)) return false; // Corregir en caso este mal
	rfm95_read_register(handle, RFM95_REGISTER_PA_CONFIG, &debug);
	//if (!rfm95_write_register(handle, RFM95_REGISTER_PA_DAC, 0x87)) return false; // Segun datasheet
	//rfm95_read_register(handle, RFM95_REGISTER_PA_DAC, &debug);

	return true;
}
*/

bool rfm95_receive_package(rfm95_handle_t *handle, uint8_t *data, uint8_t *data2, uint8_t *data3, uint8_t *data4, int RSSI, float SNR){

	int RSSI_temp = 0;
	int SNR_temp = 0;
	uint8_t packetLength = 0;
	uint8_t address;

	//Stand-by mode previous to Continuous Mode
	if (!rfm95_write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_STANDBY)) return false;

	// Clear flags
	if (!rfm95_write_register(handle, RFM95_REGISTER_IRQ_FLAGS, 0xFF)) return false;
	rfm95_read_register(handle, RFM95_REGISTER_IRQ_FLAGS, &irqFlags);

	// Continuous Mode
	if (!rfm95_write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_RX_CONTINUOS)) return false;
	// Check if RX Mode
	while (irqFlags == 0x00){
		rfm95_read_register(handle, RFM95_REGISTER_IRQ_FLAGS, &irqFlags);
		HAL_Delay(500);
	}


	// Wait for IRQ RxDone
	if (irqFlags == 0x50){ // TODO Verificar que se puede usar el DIO0 para RXDone

		// Clear flags
		if (!rfm95_write_register(handle, RFM95_REGISTER_IRQ_FLAGS, 0xFF)) return false;
		rfm95_read_register(handle, RFM95_REGISTER_IRQ_FLAGS, &irqFlags);

		// Read Packet Length
		rfm95_read_register(handle, RFM95_REGISTER_RX_NB_BYTES, &packetLength);

		// Set FifoPtrAddr to FifoRxCurrentAddr
		rfm95_read_register(handle, RFM95_REGISTER_FIFO_RX_CURRENT_ADDR, &address);
		if (!rfm95_write_register(handle, RFM95_REGISTER_FIFO_ADDR_PTR, address)) return false;

		// Read FIFO
		//for(uint8_t i = 0; i < packetLength ; i++){
		rfm95_read_register(handle, RFM95_REGISTER_FIFO_ACCESS, data);			// Temperatura Address Inicial
		rfm95_write_register(handle, RFM95_REGISTER_FIFO_ADDR_PTR, address+1);
		rfm95_read_register(handle, RFM95_REGISTER_FIFO_ACCESS, data2);			// Humedad Address +1
		rfm95_write_register(handle, RFM95_REGISTER_FIFO_ADDR_PTR, address+2);
		rfm95_read_register(handle, RFM95_REGISTER_FIFO_ACCESS, data3);			// CO Upper Byte Address +2
		rfm95_write_register(handle, RFM95_REGISTER_FIFO_ADDR_PTR, address+3);
		rfm95_read_register(handle, RFM95_REGISTER_FIFO_ACCESS, data4);			// CO Lower Byte Address +3
		//rfm95_write_register(handle, RFM95_REGISTER_FIFO_ADDR_PTR, address+4);
		//rfm95_read_register(handle, RFM95_REGISTER_FIFO_ACCESS, data3);
		//rfm95_write_register(handle, RFM95_REGISTER_FIFO_ADDR_PTR, address+5);
		//rfm95_read_register(handle, RFM95_REGISTER_FIFO_ACCESS, data3);
		//rfm95_write_register(handle, RFM95_REGISTER_FIFO_ADDR_PTR, address+6);
		//rfm95_read_register(handle, RFM95_REGISTER_FIFO_ACCESS, data3);
		//}

		if (!rfm95_write_register(handle, RFM95_REGISTER_FIFO_RX_BASE_ADDR, 0x00)) return false;

		// Read RSSI & SNR values
		rfm95_read_register(handle, RFM95_REGISTER_PKT_SNR_VALUE,(uint8_t *) &SNR_temp);
		SNR = SNR_temp * 0.25;

		rfm95_read_register(handle, RFM95_REGISTER_PKT_RSSI_VALUE, (uint8_t *) &RSSI_temp);
		RSSI = RSSI_temp - 157;

		// Stand-by mode
		if (!rfm95_write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_STANDBY)) return false;
	}

	return true;
}
