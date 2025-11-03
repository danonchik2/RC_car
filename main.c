#include "RccConfig.h"
#include <stdlib.h>
#include <string.h>

#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0XC8
#define CRSF_FRAME_LENGTH 24
#define CRSF_PACKET_SIZE 26
#define CRSF_MAX_CHANNEL 16
#define CRSF_PACKETTYPE_RC_CHANNELS_DATA 0X16

uint8_t* inBuffer = NULL;
uint8_t crsfData[CRSF_PACKET_SIZE];
uint16_t m_channels[CRSF_MAX_CHANNEL];

uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len);


void GPIO_Config() {
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	
	//AF MODE FOR PA 8, 9, 10, 11 FOR TIMER
	GPIOA->MODER |= (2 << GPIO_MODER_MODER8_Pos) | (2 << GPIO_MODER_MODER9_Pos) | (2 << GPIO_MODER_MODER10_Pos) | (2 << GPIO_MODER_MODER11_Pos);
	
	//AF MODE FOR PB7 (USART1_RX)
	GPIOB->MODER |= (2 << GPIO_MODER_MODER7_Pos);
	
	// PD0-PD7 - 8 OUTPUTS FOR MOTORS (0-1 2-3 4-5 6-7)
	GPIOD->MODER |= (1 << GPIO_MODER_MODER0_Pos) | (1 << GPIO_MODER_MODER1_Pos) | (1 << GPIO_MODER_MODER2_Pos) | (1 << GPIO_MODER_MODER3_Pos) |
	(1 << GPIO_MODER_MODER4_Pos) | (1 << GPIO_MODER_MODER5_Pos) | (1 << GPIO_MODER_MODER6_Pos) | (1 << GPIO_MODER_MODER7_Pos);
	
	// AF TIM1 
	GPIOA->AFR[1] |= (1 << GPIO_AFRH_AFSEL8_Pos) | (1 << GPIO_AFRH_AFSEL9_Pos) | (1 << GPIO_AFRH_AFSEL10_Pos) | (1 << GPIO_AFRH_AFSEL11_Pos);

	//AF 7 FOR USART1_RX (PB7)
	GPIOB->AFR[0] |= (7 << GPIO_AFRL_AFSEL7_Pos);
	
	GPIOD->OSPEEDR |= 0x5555;
}

void TIMERS_Config() {
	
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // en clock
	
	TIM1->CR1 |= TIM_CR1_ARPE; // the auto-reload preload register ENABLE 
	
	TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos); // ‘110’ (PWM mode 1) for ch 1
	TIM1->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos); // ‘110’ (PWM mode 1) for ch 2
	TIM1->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos); // ‘110’ (PWM mode 1) for ch 3
	TIM1->CCMR2 |= (6 << TIM_CCMR2_OC4M_Pos); // ‘110’ (PWM mode 1) for ch 4
	
	TIM1->PSC = 168 - 1;
	TIM1->ARR = 49;
	TIM1->CCR1 = 25;
	TIM1->CCR2 = 25;
	TIM1->CCR3 = 25;
	TIM1->CCR4 = 25;
	
	
	TIM1->CCMR1 |= TIM_CCMR1_OC1PE; //  CCR1 (for ch 1) preload register. preload value will be in queue and will not be taken in account immediately
	TIM1->CCMR1 |= TIM_CCMR1_OC2PE;
	TIM1->CCMR2 |= TIM_CCMR2_OC3PE;
	TIM1->CCMR2 |= TIM_CCMR2_OC4PE;
	
	// output enable FOR PWM REACH PINS
	TIM1->CCER |= TIM_CCER_CC1E;
TIM1->CCER |= TIM_CCER_CC2E;
TIM1->CCER |= TIM_CCER_CC3E;
TIM1->CCER |= TIM_CCER_CC4E;

TIM1->BDTR |= TIM_BDTR_MOE;
TIM1->EGR |= TIM_EGR_UG; // As the preload registers are transferred to the shadow registers the user must initialize all the registers by setting the UG bit in the TIMx_EGR register.


	TIM1->CR1 |= TIM_CR1_CEN; // en timer

	while (!(TIM1->SR & TIM_SR_UIF)); // wait for flag to set

}

void USART1_Config() {
	RCC->APB2ENR |= (1 << RCC_APB2ENR_USART1EN_Pos);
	
	USART1->BRR = 0x2D9; 
	
	USART1->CR1 |= (1 << USART_CR1_RE_Pos); // receiver en
	
	USART1->CR1 |= (1 << USART_CR1_UE_Pos); // EN USART

}

uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len)
{
  static const uint8_t crsf_crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++)
  {
    crc = crsf_crc8tab[crc ^ *ptr++];
  }
  return crc;
}

void updateChannels()
{

  size_t bitOffset = 0;
  
  for (size_t i = 0; i < 16; ++i) {
    size_t byteOffset = bitOffset / 8;
    size_t bitStart = bitOffset % 8;

    // unpack the 11-bit channel value
    m_channels[i] = (
        (crsfData[3 + byteOffset] >> bitStart) |
        (crsfData[4 + byteOffset] << (8 - bitStart))
    ) & 0x07FF;

    bitOffset += 11;
  }
}

void handleCar() {
    if (m_channels[1] > 1024) {
        GPIOD->BSRR = (1 << GPIO_BSRR_BS0_Pos); 
        GPIOD->BSRR = (1 << GPIO_BSRR_BR1_Pos);
    } else if (m_channels[1] < 1024) {
        GPIOD->BSRR = (1 << GPIO_BSRR_BR0_Pos); 
        GPIOD->BSRR = (1 << GPIO_BSRR_BS1_Pos);
    } else {
        GPIOD->BSRR = (1 << GPIO_BSRR_BR0_Pos) | (1 << GPIO_BSRR_BR1_Pos);
    }
}

int main () {
	
	SysClockConfig ();
	GPIO_Config();
	USART1_Config();
	TIMERS_Config();
	
while (1){
	
	uint8_t packetLength, inData, bufferIndex = 0;
	while (USART1->SR & USART_SR_RXNE){
	inData = USART1->DR;
		if (bufferIndex == 0) {
			if (inData == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
				uint8_t addressByte = inData;
					if (USART1->SR & USART_SR_RXNE){
						inData = USART1->DR;
						packetLength = inData;
						if (USART1->SR & USART_SR_RXNE){
							inData = USART1->DR;
							if (inData != CRSF_PACKETTYPE_RC_CHANNELS_DATA) {
							bufferIndex = 0;
							continue;
							}
							uint8_t inBuffer[CRSF_PACKET_SIZE + 2];
							inBuffer[bufferIndex++] = addressByte;
							inBuffer[bufferIndex++] = packetLength;
							inBuffer[bufferIndex++] = inData;
						}
					}
			} else { // skip
        uint8_t length = USART1->DR;
        for (int i = 0; i < length - 1; i++) {
          USART1->DR;
        }
        bufferIndex = 0;
        continue;
      }
			
		} else if (bufferIndex > 1 && bufferIndex < packetLength + 1) {
      inBuffer[bufferIndex++] = inData;
			}
			else if (bufferIndex == packetLength + 1) {
      inBuffer[bufferIndex++] = inData;
      uint8_t crc = crsf_crc8(&inBuffer[2], inBuffer[1] - 1);
      if (crc == inBuffer[packetLength + 1]) {
        memcpy(crsfData, inBuffer, packetLength + 2);
        updateChannels();
        handleCar();
      }
      inBuffer = NULL;
      bufferIndex = 0;
      packetLength = 0;
    }
}

}
}
