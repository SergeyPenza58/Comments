#include "main.h"

//define boot flag address
#define BOOT_FLAG_ADDRESS           0x08004000U

//define main application address
#define APP_ADDRESS                 0x08008000U

//define timeout value for received UART function (ms)
#define TIMEOUT_VALUE               100

//define ACK value
#define ACK     0x06U
//define NACK value
#define NACK    0x16U

CRC_HandleTypeDef hcrc;
static UART_HandleTypeDef huart;
static uint8_t RX_Buffer[32];

// define the value for commands
typedef enum
{
	ERASE = 0x43,    // Erase data
	WRITE = 0x31,	 // write data
	CHECK = 0x51,	 // check data
	JUMP = 0xA1,	 // do jump on memory address
} COMMANDS;

static void Jump2App(void);
static void Boot_Init(void);
static void MX_CRC_Init(void);
static void Transmit_ACK(UART_HandleTypeDef *huart);
static void Transmit_NACK(UART_HandleTypeDef *huart);
static uint8_t Check_Checksum(uint8_t *pBuffer, uint32_t len);
static void Erase(void);
static void Write(void);
static void Check(void);

int main(void)
{
	Clk_Update(); 	// Init peripheral clock
	Boot_Init(); 	// Init UART

	Transmit_ACK(&huart); // Send on line UART data ACK
	// Wait 2 bytes of data
	if (HAL_UART_Rx(&huart, RX_Buffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT)
	{
		// Data not received
		// Send on line UART data NACK
		Transmit_NACK(&huart);
		// Jump to main application
		Jump2App();
	}

	// Data was received, need check checksum
	if (Check_Checksum(RX_Buffer, 2) != 1 || RX_Buffer[0] != ACK)
	{
		// Checksum is not OK or we received first byte not ACK
		// Send on line UART data NACK
		Transmit_NACK(&huart);
		// Jump to main application
		Jump2App();
	}

	for (;;)
	{
		//wait 2 bytes on UART line
		while (HAL_UART_Rx(&huart, RX_Buffer, 2, TIMEOUT_VALUE)
				== HAL_UART_TIMEOUT)
			;

		//2 byte successfully received

		if (Check_Checksum(RX_Buffer, 2) != 1)
		{
			// Checksum is not OK
			// Send on line UART data NACK
			Transmit_NACK(&huart);
		}
		else
		{
			// According to commands
			switch (RX_Buffer[0])
			{
			case ERASE:
				// war received ERASE command
				// Send on line UART data ACK.
				Transmit_ACK(&huart);
				// do erase algorithm
				Erase();
				break;
			case WRITE:
				// war received Write command
				// Send on line UART data ACK.
				Transmit_ACK(&huart);
				// do write algorithm
				Write();
				break;
			case CHECK:
				// war received Check command
				// Send on line UART data ACK.
				Transmit_ACK(&huart);
				// do check algorithm
				Check();
				break;
			case JUMP:
				// war received Jump command
				// Send on line UART data ACK.
				Transmit_ACK(&huart);
				// do jump algorithm
				Jump2App();
				break;
			default:
				// war received not defined command
				// Send on line UART data NACK.
				Transmit_NACK(&huart);
				break;
			}
		}
	}

	for (;;)
		;
	return 0;
}

/* @brief Move from the boot area to the main program memory area
 * @param None
 * @retval none
 */
static void Jump2App(void)
{
	//  Test if user code is programmed starting from address
	if (((*(__IO uint32_t*) APP_ADDRESS) & 0x2FFE0000) == 0x20000000)
	{
		__disable_irq(); // disable all interrupts
		/* Set the jump address to the main program */
		/* The transition is made by executing the function, the address of which is specified manually */
		/* +4 bytes because there is a pointer to the interrupt vector at the very beginning */
		uint32_t jump_address = *(__IO uint32_t*) (APP_ADDRESS + 4);
		/* Move stack address */
		__set_MSP(*(__IO uint32_t*) APP_ADDRESS);
		/* Go to the main program */
		void (*pmain_app)(void) = (void (*)(void))(jump_address);
		pmain_app();
	}

}

/* @brief Initialize inreface UART2
 * @param None
 * @retval none
 */
static void Boot_Init(void)
{
	GPIO_InitTypeDef gpio_uart;
	// Configured PA2 and PA3 how alternative function UART interface
	gpio_uart.Pin = GPIO_PIN_2 | GPIO_PIN_3;
	gpio_uart.Mode = GPIO_MODE_AF_PP;
	gpio_uart.Pull = GPIO_PULL_NONE;
	gpio_uart.Speed = GPIO_SPEED_LOW;
	gpio_uart.Alternate = GPIO_AF7_USART2;

	// Enable Clock for GPIOA
	HAL_RCC_GPIOA_CLK_ENABLE();

	// initialize the ports according to our struct
	HAL_GPIO_Init(GPIOA, &gpio_uart);

	// Configured UART2 interface with next settings.
	huart.Init.BaudRate = 115200;
	huart.Init.Mode = HAL_UART_MODE_TX_RX;
	huart.Init.OverSampling = HAL_UART_OVERSAMPLING_16;
	huart.Init.Parity = HAL_UART_PARITY_NONE;
	huart.Init.StopBits = HAL_UART_STOP_1;
	huart.Init.WordLength = HAL_UART_WORD8;
	huart.Instance = USART2;

	// Enable Clock for UART2
	HAL_RCC_USART2_CLK_ENABLE();
	// initialize the UART according to our struct
	HAL_UART_Init(&huart);
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{
	hcrc.Instance = CRC;
	HAL_CRC_Init(&hcrc);
}

/* @brief Transmit package with ACK
 *
 * @param UART_HandleTypeDef *handle - pointer in uart struct @UART_HandleTypeDef
 *
 *
 * @retval none
 */
static void Transmit_ACK(UART_HandleTypeDef *handle)
{
	// define array with 2 bytes ACK
	uint8_t msg[2] =
	{ ACK, ACK };

	// Send 2 bytes
	HAL_UART_Tx(handle, msg, 2);
}

/* @brief Transmit package with NACK
 *
 * @param UART_HandleTypeDef *handle - pointer in uart struct @UART_HandleTypeDef
 *
 *
 * @retval none
 */
static void Transmit_NACK(UART_HandleTypeDef *handle)
{
	// define array with 2 bytes NACK
	uint8_t msg[2] =
	{ NACK, NACK };

	// Send 2 bytes
	HAL_UART_Tx(handle, msg, 2);
}

/* @brief Check Checksum in received package
 *
 * @param uint8_t *pBuffer - pointer to received buffer
 *
 * @param uint32_t len - length of received data
 *
 * @retval uint8_t - "1" if checksum is OK
 */
static uint8_t Check_Checksum(uint8_t *pBuffer, uint32_t len)
{
	uint8_t initial = 0xFF;
	uint8_t result = 0x7F;

	// Do XOR operation with all received bytes
	result = initial ^ *pBuffer++;
	len--;
	while (len--)
	{
		result ^= *pBuffer++; // XOR and next byte
	}

	result ^= 0xFF;

	// Checksum is OK
	if (result == 0x00)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/* @brief Do Flash Sector Erase according to received data from UART
 *  @note ACK will be tx, if erase is OK.
 *		 NACK will be tx, if error is detected
 *
 * @retval none
 */
static void Erase(void)
{
	Flash_EraseInitTypeDef flashEraseConfig;
	uint32_t sectorError;

	// Received 3 byte of data
	while (HAL_UART_Rx(&huart, RX_Buffer, 3, TIMEOUT_VALUE) == HAL_UART_TIMEOUT)
		;

	// Do check checksum from received buffer
	if (Check_Checksum(RX_Buffer, 3) != 1)
	{
		// Checksum is incorrect, send NACK
		Transmit_NACK(&huart);
		return;
	}

	if (RX_Buffer[0] == 0xFF)
	{
		// first byte is incorrect, send NACK
		Transmit_NACK(&huart);
	}
	else
	{
		// Received data is correct
		// Fill in the fields of the struct
		flashEraseConfig.TypeErase = HAL_FLASH_TYPEERASE_SECTOR;
		flashEraseConfig.NbSectors = RX_Buffer[0];
		flashEraseConfig.Sector = RX_Buffer[1];

		//Do unlock, erase, lock Flash memory
		HAL_Flash_Unlock();
		HAL_Flash_Erase(&flashEraseConfig, &sectorError);
		HAL_Flash_Lock();

		// Flash was erased, send ACK
		Transmit_ACK(&huart);
	}
}

/* @brief Write new received flash value
 *  @note ACK will be tx, if write is OK.
 *		 NACK will be tx, if error is detected
 *
 * @retval none
 */
static void Write(void)
{
	uint8_t numBytes;
	uint32_t startingAddress = 0;
	uint8_t i;

	// Received 5 byte of data flash address
	while (HAL_UART_Rx(&huart, RX_Buffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT)
		;

	// Do check checksum from received buffer
	if (Check_Checksum(RX_Buffer, 5) != 1)
	{
		// Checksum is incorrect, send NACK
		Transmit_NACK(&huart);
		return;
	}
	else
	{
		// Checksum is correct, send ACK
		Transmit_ACK(&huart);
	}

	// According to received data defined address where will be written new value
	startingAddress = RX_Buffer[0] + (RX_Buffer[1] << 8) + (RX_Buffer[2] << 16)
			+ (RX_Buffer[3] << 24);

	// Received 2 byte of data Number bytes which need write in flash
	while (HAL_UART_Rx(&huart, RX_Buffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT)
		;
	numBytes = RX_Buffer[0];

	// Received n byte of new flash value
	while (HAL_UART_Rx(&huart, RX_Buffer, numBytes + 1, TIMEOUT_VALUE)
			== HAL_UART_TIMEOUT)
		;

	// Do check checksum from received buffer
	if (Check_Checksum(RX_Buffer, numBytes + 1) != 1)
	{
		// Checksum is incorrect, send NACK
		Transmit_NACK(&huart);
		return;
	}

	i = 0;
	HAL_Flash_Unlock(); // Unlock flash before write value
	while (numBytes--)
	{
		// write new received value in flash in @startingAddress
		HAL_Flash_Program(FLASH_TYPEPROGRAM_BYTE, startingAddress,
				RX_Buffer[i]);
		startingAddress++; 		// increment flash address
		i++; 					// next value of data
	}
	HAL_Flash_Lock();			// Lock flash after write value
	Transmit_ACK(&huart); 		// Send ACK
}

/* @brief Check Flash memory according to received starting and ending address,
 *  @note ACK will be tx, if Checksum is OK.
 *		 NACK will be tx, if error is detected.
 *		 Will jump to the main program.
 *
 *
 * @retval none
 */
static void Check(void)
{
	uint32_t startingAddress = 0;
	uint32_t endingAddress = 0;
	uint32_t address;
	uint32_t *data;
	uint32_t crcResult;

	// wait 5 bytes of starting address
	while (HAL_UART_Rx(&huart, RX_Buffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT)
		;

	if (Check_Checksum(RX_Buffer, 5) != 1)
	{
		// Checksum is incorrect, send NACK
		Transmit_NACK(&huart);
		return;
	}
	else
	{
		// Checksum received data is correct, send ACK
		Transmit_ACK(&huart);
	}

	// was received starting address
	startingAddress = RX_Buffer[0] + (RX_Buffer[1] << 8) + (RX_Buffer[2] << 16)
			+ (RX_Buffer[3] << 24);

	// wait 5 bytes of ending address
	while (HAL_UART_Rx(&huart, RX_Buffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT)
		;

	if (Check_Checksum(RX_Buffer, 5) != 1)
	{
		// Checksum is incorrect, send NACK
		Transmit_NACK(&huart);
		return;
	}
	else
	{
		// Checksum received data is correct, send ACK
		Transmit_ACK(&huart);
	}

	// was received ending address
	endingAddress = RX_Buffer[0] + (RX_Buffer[1] << 8) + (RX_Buffer[2] << 16)
			+ (RX_Buffer[3] << 24);

	// enable clock signal from CRC module
	HAL_RCC_CRC_CLK_ENABLE();
	// Read data in flash from @startingAddress to @endingAddress and calculate checksum
	data = (uint32_t*) ((__IO uint32_t*) startingAddress);
	for (address = startingAddress; address < endingAddress; address += 4)
	{
		data = (uint32_t*) ((__IO uint32_t*) address);
		crcResult = HAL_CRC_Accumulate(&hcrc, data, 1); // starting with the previously computed CRC as initialization value.
	}
	// Disable clock signal from CRC module
	HAL_RCC_CRC_CLK_DISABLE();

	if (crcResult == 0x00)
	{
		Transmit_ACK(&huart);
	}
	else
	{
		// Checksum FLASH is incorrect, send NACK
		Transmit_NACK(&huart);
	}

	// Jump to main program
	Jump2App();
}
