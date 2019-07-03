#include "xmodem.h"

#include <stdio.h>

/* Local Struct */
typedef struct {
	uint8_t first_packet;         /**< First packet or not. */
	uint8_t packet_number;        /**< Packet number counter. */
	uint32_t dst_addr; /**< Address where we have to write. */
	uint32_t cur_addr;
	uint32_t maxsize;             /**< Maximum size of the file */
	size_t size;
} xmodem_file_t;

/* Local functions. */
static uint16_t xmodem_calc_crc(uint8_t *data, uint16_t length);

static size_t xmodem_receive_file(uint32_t flash_location, uint32_t maxsize);
static xmodem_status xmodem_handle_packet(xmodem_file_t* file, uint8_t header);
static xmodem_status xmodem_error_handler(uint8_t *error_number, uint8_t max_error_number);


/**
 * @brief   This function download the signature and the app in the cache
 * @param   void
 * @return  void
 */
bool xmodem_receive(void)
{
	//Erase cache sector (Erase sector4 only)
	if (flash_erase(FLASH_APP2, 1u) != FLASH_OK) {
		return false;
	}
	app_t app;
	uart_tx_str((uint8_t*)"\n\rSend signature file\n\r");
	app.sign_size = xmodem_receive_file(FLASH_APP2_SIGN, FLASH_SIGN_SIZE);
	if (!app.sign_size)
		return false;

	//uart_tx_str((uint8_t*)"\n\rSend update file\n\r");

	char b[128];
	sprintf(b, "Downloaded signature: %dByte and app: %dByte\n", app.sign_size, app.app_size);
	uart_tx_str((uint8_t*)b);

	uart_tx_str((uint8_t*)"\n\rApplication downloaded!\n\r");
	uart_tx_str((uint8_t*)"Starting verification...\n\r");
	//flash_jump_to_app();
	return true;
}



/**
 * @brief   This function is the base of the Xmodem protocol.
 *          When we receive a header from UART, it decides what action it shall take.
 * @param   void
 * @return  void
 */
size_t xmodem_receive_file(uint32_t flash_location, uint32_t maxsize)
{
	xmodem_file_t file;
	file.first_packet = false;
	file.packet_number = 1u;
	file.dst_addr = flash_location;
	file.cur_addr = flash_location;
	file.maxsize = maxsize;
	file.size = 0;

	uint8_t error_number = 0u;

	volatile xmodem_status status = X_OK;
	/* Loop until there isn't any error (or until we jump to the user application). */
	while (status == X_OK)
	{
		uint8_t header = 0x00u;

		/* Get the header from UART. */
		uart_status comm_status = uart_rx(&header, 1u);

		/* Spam the host (until we receive something) with ACSII "C", to notify it, we want to use CRC-16. */
		if (comm_status != UART_OK && !file.first_packet)
		{
			uart_tx_ch(X_C);
		}
		/* Uart timeout or any other errors. */
		else if (comm_status != UART_OK && file.first_packet)
		{
			status = xmodem_error_handler(&error_number, X_MAX_ERRORS);
		}

		xmodem_status packet_status = X_ERROR;
		/* The header can be: SOH, STX, EOT and CAN. */
		switch(header)
		{
			/* 128 or 1024 bytes of data. */
		case X_SOH:
		case X_STX:
			/* If the handling was successful, then send an ACK. */
			packet_status = xmodem_handle_packet(&file, header);
			if (packet_status == X_OK)
			{
				uart_tx_ch(X_ACK);
			}
			/* If the error was flash related, then immediately set the error counter to max (graceful abort). */
			else if (packet_status == X_ERROR_FLASH)
			{
				error_number = X_MAX_ERRORS;
				status = xmodem_error_handler(&error_number, X_MAX_ERRORS);
			}
			/* Error while processing the packet, either send a NAK or do graceful abort. */
			else
			{
				status = xmodem_error_handler(&error_number, X_MAX_ERRORS);
			}

			break;
			/* End of Transmission. */
		case X_EOT:
			/* ACK and leave*/
			uart_tx_ch(X_ACK);
			return file.size;
			break;
			/* Abort from host. */
		case X_CAN:
			status = X_ERROR;
			break;
		default:
			/* Wrong header. */
			if (comm_status == UART_OK)
			{
				status = xmodem_error_handler(&error_number, X_MAX_ERRORS);
			}
			break;
		}
	}

	return file.size;
}

/**
 * @brief   This function handles the data packet we get from the xmodem protocol.
 * @param   header: SOH or STX.
 * @return  status: Report about the packet.
 */
static xmodem_status xmodem_handle_packet(xmodem_file_t* file, uint8_t header)
{
	xmodem_status status = X_OK;

	uint16_t size = 0u;
	if (header == X_SOH) {
		size = X_PACKET_128_SIZE;
	}
	else if (header == X_STX) {
		size = X_PACKET_1024_SIZE;
	}
	else {
		/* Wrong header type. */
		status = X_ERROR;
	}

	uint16_t length = size + X_PACKET_DATA_INDEX + X_PACKET_CRC_SIZE;
	uint8_t received_data[X_PACKET_1024_SIZE + X_PACKET_DATA_INDEX + X_PACKET_CRC_SIZE];

	/* Get the packet (except for the header) from UART. */
	uart_status comm_status = uart_rx(received_data, length);
	/* The last two bytes are the CRC from the host. */
	uint16_t crc_received = ((uint16_t)received_data[length-2u] << 8u) | ((uint16_t)received_data[length-1u]);
	/* We calculate it too. */
	uint16_t crc_calculated = xmodem_calc_crc(&received_data[X_PACKET_DATA_INDEX], size);

	if (!file->first_packet) {
		file->first_packet = true;
	}

	/* Error handling and flashing. */
	if (status == X_OK) {
		if (comm_status != UART_OK) {
			/* UART error. */
			status |= X_ERROR_UART;
		}
		if (received_data[X_PACKET_NUMBER_INDEX] != file->packet_number) {
			/* Packet number counter mismatch. */
			status |= X_ERROR_NUMBER;
		}
		if ((received_data[X_PACKET_NUMBER_INDEX] + received_data[X_PACKET_NUMBER_COMPLEMENT_INDEX]) != 255u) {
			/* The sum of the packet number and packet number complement aren't 255. */
			/* The sum always has to be 255. */
			status |= X_ERROR_NUMBER;
		}
		if (crc_calculated != crc_received) {
			/* The calculated and received CRC are different. */
			status |= X_ERROR_CRC;
		}
		//if (file->cur_addr + size > file->dst_addr + file->dst_addr) {
			//Not enough place dedicated in the flash.
		if (flash_write(file->cur_addr,
		           (uint32_t*)&received_data[X_PACKET_DATA_INDEX],
				   (uint32_t)size/4u)
				   != FLASH_OK) {
			/* Flashing error. */
			status |= X_ERROR_FLASH;
		}
	}

	/* Raise the packet number and the address counters (if there wasn't any error). */
	if (status == X_OK)
	{
		file->packet_number++;
		file->cur_addr += size;
		file->size += size;
	}

	return status;
}

/**
 * @brief   Calculates the CRC-16 for the input package.
 * @param   *data:  Array of the data which we want to calculate.
 * @param   length: Size of the data, either 128 or 1024 bytes.
 * @return  status: The calculated CRC.
 */
static uint16_t xmodem_calc_crc(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0u;
    while (length)
    {
        length--;
        crc = crc ^ ((uint16_t)*data++ << 8u);
        for (uint8_t i = 0u; i < 8u; i++)
        {
            if (crc & 0x8000u)
            {
                crc = (crc << 1u) ^ 0x1021u;
            }
            else
            {
                crc = crc << 1u;
            }
        }
    }
    return crc;
}

/**
 * @brief   Handles the xmodem error.
 *          Raises the error counter, then if the number of the errors reached critical, do a graceful abort, otherwise send a NAK.
 * @param   *error_number:    Number of current errors (passed as a pointer).
 * @param   max_error_number: Maximal allowed number of errors.
 * @return  status: X_ERROR in case of too many errors, X_OK otherwise.
 */
static xmodem_status xmodem_error_handler(uint8_t *error_number, uint8_t max_error_number)
{
  xmodem_status status = X_OK;
  /* Raise the error counter. */
  (*error_number)++;
  /* If the counter reached the max value, then abort. */
  if ((*error_number) >= max_error_number)
  {
    /* Graceful abort. */
    (void)uart_tx_ch(X_CAN);
    (void)uart_tx_ch(X_CAN);
    status = X_ERROR;
  }
  /* Otherwise send a NAK for a repeat. */
  else
  {
    (void)uart_tx_ch(X_NAK);
    status = X_OK;
  }
  return status;
}
