#include "rfm95.h"
#include "main.h"

#include <assert.h>
#include <string.h>

#define RFM9x_VER 0x12

//uint8_t debug;

/*
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
#define RFM95_REGISTER_PA_DAC_LOW_POWER                         0x84
#define RFM95_REGISTER_PA_DAC_HIGH_POWER                        0x87

// IRQ Masks
#define RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_TXDONE             0x01

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
	uint32_t frequency = 915000000;
	// FQ = (FRF * 32 Mhz) / (2 ^ 19)
	// (32000000/524288) = 61.03515625
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

	// Frequency configuration (only in sleep mode) 915MHz
	if (!rfm95_write_register(handle, RFM95_REGISTER_FR_MSB, (uint8_t)(frf >> 16))) return false;
	//rfm95_read_register(handle, RFM95_REGISTER_FR_MSB, &debug);
	if (!rfm95_write_register(handle, RFM95_REGISTER_FR_MID, (uint8_t)(frf >> 8))) return false;
	//rfm95_read_register(handle, RFM95_REGISTER_FR_MID, &debug);
	if (!rfm95_write_register(handle, RFM95_REGISTER_FR_LSB, (uint8_t)(frf >> 0))) return false;
	//rfm95_read_register(handle, RFM95_REGISTER_FR_LSB, &debug);

	// Set up TX and RX FIFO base addresses.
	if (!rfm95_write_register(handle, RFM95_REGISTER_FIFO_TX_BASE_ADDR, 0x00)) return false;
	//rfm95_read_register(handle, RFM95_REGISTER_FIFO_TX_BASE_ADDR, &debug);

	// Set LNA to the highest gain and LNABoost 150%.
	if (!rfm95_read_register(handle, RFM95_REGISTER_LNA, &lna_gain)) return false;
	if (!rfm95_write_register(handle, RFM95_REGISTER_LNA, lna_gain | 0x03)) return false;
	//rfm95_read_register(handle, RFM95_REGISTER_LNA, &debug);

	//Configure modem LowDataRateOptimize = Disable, AgcAutoOn -> LNA Gain set by register LnaGain)
	//if (!rfm95_write_register(handle, RFM95_REGISTER_MODEM_CONFIG_1, 0x63)) return false; // (62kHz afecta la velocidad de transmision , 4/5 error coding rate, implicit header)
	//if (!rfm95_write_register(handle, RFM95_REGISTER_MODEM_CONFIG_2, 0x75)) return false; // SF7 *revisar SF* (not close to gateway, therefore, should be higher SF), single packet, CRC enable;
	if (!rfm95_write_register(handle, RFM95_REGISTER_MODEM_CONFIG_3, 0x04)) return false; // LNA gain set by internal AGC loop
	//rfm95_read_register(handle, RFM95_REGISTER_MODEM_CONFIG_3, &debug);

	// Set module power to 20dbm.
	if (!rfm95_write_register(handle, RFM95_REGISTER_PA_CONFIG, 0x9F)) return false; // PaSelect = 0x01 -> PA_BOOST; MaxPower = 0x01 -> Pmax = 11.4 dbm; OutputPower = 0x20 -> Pout = 17 - (15 - 2) = 4dBm
	//rfm95_read_register(handle, RFM95_REGISTER_PA_CONFIG, &debug);
	//if (!rfm95_write_register(handle, RFM95_REGISTER_PA_DAC, 0x87)) return false; // Segun datasheet
	//rfm95_read_register(handle, RFM95_REGISTER_PA_DAC, &debug);

	// Ovefcurrent protection (OcpO; OcpTrim)
	//if (!rfm95_write_register(handle, RFM95_REGISTER_OCP, 0x1C)) return false;

	// (Optional IRQ Mask) Set IRQ Flag Mask for TX Done
	//if (!rfm95_write_register(handle, RFM95_REGISTER_IRQ_FLAGS_MASK, 0xF7)) return false; // TODO Revisar
	//rfm95_read_register(handle, RFM95_REGISTER_IRQ_FLAGS_MASK, &debug);

	// Preamble set to 8 + 4.25 = 12.25 symbols.
	//if (!rfm95_write_register(handle, RFM95_REGISTER_PREAMBLE_MSB, 0x00)) return false;
	//if (!rfm95_write_register(handle, RFM95_REGISTER_PREAMBLE_LSB, 0x08)) return false;

	// Set TTN sync word 0x34.
	//if (!rfm95_write_register(handle, RFM95_REGISTER_SYNC_WORD, 0x12)) return false;

	// Maximum payload length of the RFM95 is 6 bytes.
	if (!rfm95_write_register(handle, RFM95_REGISTER_PAYLOAD_LENGTH, 6)) return false;

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
bool rfm95_send_package(rfm95_handle_t *handle, uint8_t *data, size_t length, uint8_t enviado)
{
	//uint8_t registro_flags;
	//uint8_t modemConfig1;
	//uint8_t modemConfig2;

	//Mode Request STAND-BY
	if (!rfm95_write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_STANDBY)) return false;
	//if (rfm95_read_register(handle, RFM95_REGISTER_OP_MODE, &debug));

	// Modem Config 1
	if (!rfm95_write_register(handle, RFM95_REGISTER_MODEM_CONFIG_1, 0x72)) return false; // (125kHz afecta la velocidad de transmision , 4/5 error coding rate, explicit header)
	//if (!rfm95_write_register(handle, RFM95_REGISTER_MODEM_CONFIG_1, 0x62)) return false; // (62kHz afecta la velocidad de transmision , 4/5 error coding rate, explicit header)

	// Modem Config 2
	if (!rfm95_write_register(handle, RFM95_REGISTER_MODEM_CONFIG_2, 0x74)) return false; // SF7, TxContinuous Mode = False, RxPayloadCrcOn = Enable, SymbTimeout = 0
	//if (!rfm95_write_register(handle, RFM95_REGISTER_MODEM_CONFIG_2, 0xC4)) return false; // SF12, TxContinuous Mode = False, RxPayloadCrcOn = Enable, SymbTimeout = 0

	// Set SPI pointer to start of TX section in FIFO 0x00
	if (!rfm95_write_register(handle, RFM95_REGISTER_FIFO_ADDR_PTR, 0x00)) return false;
	//if (rfm95_read_register(handle, RFM95_REGISTER_FIFO_ADDR_PTR, &debug));

	// Set the payload length
	if (!rfm95_write_register(handle, RFM95_REGISTER_PAYLOAD_LENGTH, length)) return false;
	//if (rfm95_read_register(handle, RFM95_REGISTER_PAYLOAD_LENGTH, &debug));

	// Config Mapping 1, clear flags
	if (!rfm95_write_register(handle, RFM95_REGISTER_DIO_MAPPING_1, RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_TXDONE)) return false; //Revisar
	//if (rfm95_read_register(handle, RFM95_REGISTER_DIO_MAPPING_1, &debug));
	if (!rfm95_write_register(handle, RFM95_REGISTER_IRQ_FLAGS, 0xFF)) return false; // Retorna 0x00 lo cual es correcto porque al enviar 0xFF se limpian las banderas
	//if (rfm95_read_register(handle, RFM95_REGISTER_IRQ_FLAGS, &debug));

	// Write payload to FIFO.
	for (size_t i = 0; i < length; i++) {
	rfm95_write_register(handle, RFM95_REGISTER_FIFO_ACCESS, data[i]);
	}

	// Mode Request TX
	if (!rfm95_write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_TX)) return false;
	//if (rfm95_read_register(handle, RFM95_REGISTER_OP_MODE, &debug));

	// Wait for the transfer complete interrupt.
	while(enviado == 0){
		rfm95_read_register(handle, RFM95_REGISTER_IRQ_FLAGS, &enviado);
	}

	// Return modem to sleep
	if (!rfm95_write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;

	return true;
}
