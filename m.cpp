//nucleof401 stm32f401

#include "stm32f4xx.h"

//整数から文字列への変換
void itoa(char* str, int i){
	int n = 0;
	bool neg = false;
	if (i < 0){
		neg = true;
		i = -i;
	}
	do{
		str[n] = i % 10 + '0';
		i /= 10;
		n++;
	} while (i);
	if (neg) str[n++] = '-';
	str[n] = '\0';

	for (int j = 0; j != n / 2; j++){
		char tmp = str[j];
		str[j] = str[n - j - 1];
		str[n - j - 1] = tmp;
	}
}

//SysTickを使おう！
void Delay(volatile uint32_t nCount) {
	for (; nCount != 0; nCount--)
		;
}



//グローバルなコンストラクタを自分で呼ぶ
void initialize(void){
	extern void(*__preinit_array_start[]) (void)__attribute__((weak));
	extern void(*__preinit_array_end[]) (void)__attribute__((weak));
	extern void(*__init_array_start[]) (void)__attribute__((weak));
	extern void(*__init_array_end[]) (void)__attribute__((weak));
	// This is basically libc_init_array -- handles global constructors

	for (int i = 0; i < __preinit_array_end - __preinit_array_start; i++)
		__preinit_array_start[i]();

	for (int i = 0; i < __init_array_end - __init_array_start; i++)
		__init_array_start[i]();
}

namespace GPIO{
	class AssignmentHolder{
		//		friend int16_t operator=(int16_t& v, AssignmentHolder& ah);
		//		friend int16_t operator=(AssignmentHolder& ah, int16_t&);
	public:
		AssignmentHolder(GPIO_TypeDef* gpio, uint16_t pin);//GPIO_Pin_ALL is not avairable.
		GPIO_TypeDef* getGPIO(void);
		uint16_t getPin(void);
		uint16_t getPinSource(void);
	private:
		AssignmentHolder();
		//AssignmentHolder(const AssignmentHolder&){gpio_ = nullptr; pin_ = 0;  }
		//const AssignmentHolder& operator=(const AssignmentHolder&){ return *this; }

		GPIO_TypeDef* gpio_;
		uint16_t pin_;
	};

	AssignmentHolder::AssignmentHolder(GPIO_TypeDef* gpio, uint16_t pin)
		:gpio_(gpio), pin_(pin)
	{}

	GPIO_TypeDef* AssignmentHolder::getGPIO(void){
		return gpio_;
	}

	uint16_t AssignmentHolder::getPin(void){
		return pin_;
	}

	uint16_t AssignmentHolder::getPinSource(void){
		uint16_t mask = 0x01;
		uint16_t i = 0;
		while (!(pin_ & mask)){
			i++;
			mask <<= 1;
		}
		return i;
	}

}

namespace IO{

	//nucleo interface
	GPIO::AssignmentHolder user_led(GPIOA, GPIO_Pin_5);//CN10.11
	GPIO::AssignmentHolder user_switch(GPIOC, GPIO_Pin_13);//CN7.23
	GPIO::AssignmentHolder spi1_cs(GPIOB, GPIO_Pin_10);//cn10 25

	void setting(void){
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

		GPIO_InitTypeDef GPIO_InitStructure = {};

		//nucleo LED
		GPIO_InitStructure.GPIO_Pin = user_led.getPin();
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(user_led.getGPIO(), &GPIO_InitStructure);


		//nuclleo Pushswitch
		GPIO_InitStructure.GPIO_Pin = user_switch.getPin();//ここはハード的にPULLUPされてる
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(user_switch.getGPIO(), &GPIO_InitStructure);


		GPIO_InitStructure.GPIO_Pin = spi1_cs.getPin();
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(spi1_cs.getGPIO(), &GPIO_InitStructure);
	}

	//書き込み補助
	//可変長引数への対応->ポート違うとだめだから保留
	void Set(GPIO::AssignmentHolder& ah){
		GPIO_SetBits(ah.getGPIO(), ah.getPin());
	}

	void Reset(GPIO::AssignmentHolder& ah){
		GPIO_ResetBits(ah.getGPIO(), ah.getPin());
	}

	void Write(GPIO::AssignmentHolder& ah, int16_t v){
		GPIO_WriteBit(ah.getGPIO(), ah.getPin(), v ? Bit_SET : Bit_RESET);
	}
	/*
	int operator=(AssignmentHolder& ah, int v){
	Write(ah, v);
	return v;
	}
	*/
	//読み込み補助
	int16_t Read(GPIO::AssignmentHolder& ah){
		return (int16_t)GPIO_ReadInputDataBit(ah.getGPIO(), ah.getPin());
	}
	/*
	int16_t operator=(int16_t& v, AssignmentHolder& ah){
	v = Read(ah);
	return v;
	}

	bool operator==(int v, GPIO::AssignmentHolder& ah){
	return v == Read(ah);
	}

	bool operator==(GPIO::AssignmentHolder& ah, int v){
	return v == Read(ah);
	}*/
}

namespace USART{

	//USART2 module
	GPIO::AssignmentHolder usart_tx(GPIOA, GPIO_Pin_2);//CN10.35
	GPIO::AssignmentHolder usart_rx(GPIOA, GPIO_Pin_3);//CN10.37


	void setting(void){
		//クロック供給
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

		GPIO_InitTypeDef GPIO_InitStructure = {};
		GPIO_InitStructure.GPIO_Pin = usart_tx.getPin() | usart_rx.getPin();
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(usart_tx.getGPIO(), &GPIO_InitStructure);

		/* Connect USART pins to AF */
		GPIO_PinAFConfig(usart_rx.getGPIO(), usart_rx.getPinSource(), GPIO_AF_USART2);
		GPIO_PinAFConfig(usart_tx.getGPIO(), usart_tx.getPinSource(), GPIO_AF_USART2);

		USART_InitTypeDef USART_InitStructure;

		/* USARTx configuration ------------------------------------------------------*/
		/* USARTx configured as follow:
		- BaudRate = X baud
		- Word Length = 8 Bits
		- One Stop Bit
		- No parity
		- Hardware flow control disabled (RTS and CTS signals)
		- Receive and transmit enabled
		*/
		USART_InitStructure.USART_BaudRate = 9600;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

		USART_Init(USART2, &USART_InitStructure);

		USART_Cmd(USART2, ENABLE);

	}

	void output(uint8_t data){
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // Wait for Empty

		USART_SendData(USART2, data);
	}

	void outputString(char* str){
		while (*str)
		{
			while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // Wait for Empty

			USART_SendData(USART2, *str++); // Send Char
		}
	}

	class Stream{
	public:
		Stream(){}
		~Stream(){}

		Stream& operator<<(char* s){
			outputString(s);
			return *this;
		}

		Stream& operator<<(int n){
			char str[16];
			itoa(str, n);
			outputString(str);
			return *this;
		}

		Stream& operator<<(Stream& (*manipulator)(Stream&)){
			return manipulator(*this);
		}
	};

	Stream& endl(Stream& st){
		outputString("\n\r");
		return st;
	}


	Stream out;
}

namespace SPI{

	//SPI1 module for moter driving
	//TODO : change pins for ps2
	GPIO::AssignmentHolder spi1_clk(GPIOB, GPIO_Pin_3);//CN10.31
	GPIO::AssignmentHolder spi1_miso(GPIOB, GPIO_Pin_4);//CN10.27
	GPIO::AssignmentHolder spi1_mosi(GPIOB, GPIO_Pin_5);//CN10.29


	void setting(void){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
		//RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

		GPIO_InitTypeDef GPIO_InitStructure = {};

		GPIO_InitStructure.GPIO_Pin = spi1_clk.getPin() | spi1_miso.getPin() | spi1_mosi.getPin();
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(spi1_clk.getGPIO(), &GPIO_InitStructure);

		GPIO_PinAFConfig(spi1_clk.getGPIO(), spi1_clk.getPinSource(), GPIO_AF_SPI1);
		GPIO_PinAFConfig(spi1_miso.getGPIO(), spi1_miso.getPinSource(), GPIO_AF_SPI1);
		GPIO_PinAFConfig(spi1_mosi.getGPIO(), spi1_mosi.getPinSource(), GPIO_AF_SPI1);




		//SPI setting
		/* SPI1 and SPI2 configuration */
		SPI_InitTypeDef SPI_InitStructure = { 0 };
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;//通常時のHigh,Low
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//ラッチ
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;

		SPI_Init(SPI1, &SPI_InitStructure);

		SPI_Cmd(SPI1, ENABLE);
	}

	uint8_t transaction1(uint8_t data){
		uint16_t RxData;
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);		// wait until transmit complete
		SPI_I2S_SendData(SPI1, data);										// write data to be transmitted to the SPI data register
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);	// wait until receive complete
		RxData = SPI_I2S_ReceiveData(SPI1);								// return received data from SPI data register
		return (uint8_t)RxData;
	}


}

namespace Timer{
	void setting(void){
		//クロック供給
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //タイマ2有効化

		//割り込み優先順位設定
		NVIC_InitTypeDef NVIC_InitStructure = { 0 };
		//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
		// Enable the TIM2 gloabal Interrupt 
		NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		//	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

		//タイマ設定
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = { 0 };
		// 42MHz*2=84MHz? -> 10kHz -> 10kHz -> 1Hz
		TIM_TimeBaseStructure.TIM_Period = 100 - 1;//何カウントしたら割り込みするか10kHz-->100Hz ==> PIDかけられる?
		TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;//一カウントするのに何クロック要求するか84MHz-->10kHz
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//84MHz / 1 = 84MHz
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	}

	void begin2(void){
		TIM_Cmd(TIM2, ENABLE);
	}
}

namespace STEP{

	//SPI1 module for moter driving
	GPIO::AssignmentHolder stp1_clk(GPIOA, GPIO_Pin_7);//CN10.15
	GPIO::AssignmentHolder stp1_dir(GPIOA, GPIO_Pin_11);//CN10.14
	GPIO::AssignmentHolder stp1_rst(GPIOA, GPIO_Pin_12);//CN10.12

	volatile unsigned long stp1_cnt = 0;
	typedef enum {
		CW,		//Set
		CCW,	//Reset.tabun
	} dir_e;
	dir_e nowdir = CW;

	void setting(void){
		GPIO_InitTypeDef GPIO_InitStructure = {};

		GPIO_InitStructure.GPIO_Pin = stp1_clk.getPin() | stp1_dir.getPin() | stp1_rst.getPin();
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(stp1_clk.getGPIO(), &GPIO_InitStructure);

	}
	
	void reset_stp1(void) {
		IO::Reset(stp1_rst);
		IO::Reset(stp1_dir);
		IO::Reset(stp1_clk);
		Delay(0xffff);//1ms待ちたい
		IO::Set(stp1_rst);
		Delay(0xffff);//1ms待ちたい
		IO::Reset(stp1_rst);
		Delay(0xffff);//1ms待ちたい
	}

	void reset_stp(void) {
		reset_stp1();
	}

	void stp1Pulse(dir_e dir){
		/**/ if(dir == CW  && nowdir != CW ) IO::Set(stp1_dir);
		else if(dir == CCW && nowdir != CCW) IO::Reset(stp1_dir);
		static bool p = true;
		if (p){
			IO::Reset(stp1_clk);
			stp1_cnt++;
		}
		else{
			IO::Set(stp1_clk);
		}
		p = !p;
	}

}

class PS2Controller{
public:
	void update(void){
		//spi transaction

		IO::Reset(IO::spi1_cs);
		Delay(100);

		SPI::transaction1(0x01);
		Delay(100);
		SPI::transaction1(0x42);
		Delay(100);
		SPI::transaction1(0x00);
		Delay(100);
		button[0] = ~SPI::transaction1(0x00);
		Delay(100);
		button[1] = ~SPI::transaction1(0x00);
		Delay(100);
		stick[0][0] = SPI::transaction1(0x00);
		Delay(100);
		stick[0][1] = SPI::transaction1(0x00);
		Delay(100);
		stick[1][0] = SPI::transaction1(0x00);
		Delay(100);
		stick[1][1] = SPI::transaction1(0x00);
		Delay(100);

		IO::Set(IO::spi1_cs);

	}
	//TODO : constexpr
	unsigned char get_button(int n){ return button[n]; }
	unsigned char get_stick(int n, int m){ return stick[n][m]; }
	//char puressure(int n){ return puressure[n]; }
#	define pushed_def(btn)int pushed_##btn(void) {return get_button((mask_##btn >> 8) & 1) & mask_##btn;}
	pushed_def(select);
	pushed_def(lst   );
	pushed_def(rst   );
	pushed_def(start );
	pushed_def(up    );
	pushed_def(right );
	pushed_def(down  );
	pushed_def(left  );
	pushed_def(l2    );
	pushed_def(r2    );
	pushed_def(l1    );
	pushed_def(r1    );
	pushed_def(tri   );
	pushed_def(cir   );
	pushed_def(cr    );
	pushed_def(sq    );
#	undef pushed_def
#	define pushed(btn) pushed_##btn()
private:
	unsigned char button[2];
	unsigned char stick[2][2];//[0:R 1:L][0:s 1:v]
	//char puressure[12];
	enum {
		mask_select = 0b000000001,
		mask_lst    = 0b000000010,
		mask_rst    = 0b000000100,
		mask_start  = 0b000001000,
		mask_up     = 0b000010000,
		mask_right  = 0b000100000,
		mask_down   = 0b001000000,
		mask_left   = 0b010000000,
		mask_l2     = 0b100000001,
		mask_r2     = 0b100000010,
		mask_l1     = 0b100000100,
		mask_r1     = 0b100001000,
		mask_tri    = 0b100010000,
		mask_cir    = 0b100100000,
		mask_cr     = 0b101000000,
		mask_sq     = 0b110000000
	};

};


int main(void)
{
	initialize();

	SystemInit();

	IO::setting();
	USART::setting();
	SPI::setting();
	Timer::setting();
	STEP::setting();

	Delay(0xff);


	USART::out << "Connected to Nucleo F401RE." << USART::endl;

	//クロック周波数の確認
	RCC_ClocksTypeDef RCC_ClockFreq;
	RCC_GetClocksFreq(&RCC_ClockFreq);
	//printf("Frequency\r\nSYSCLK:%d\r\nHCLK:%d\r\nPCLK1:%d\r\nPCLK2:%d\r\n", RCC_ClockFreq.SYSCLK_Frequency, RCC_ClockFreq.HCLK_Frequency, RCC_ClockFreq.PCLK1_Frequency, RCC_ClockFreq.PCLK2_Frequency);
	USART::out << "SYSCLK:" << RCC_ClockFreq.SYSCLK_Frequency << USART::endl
		<< "HCLK:" << RCC_ClockFreq.HCLK_Frequency << USART::endl;


//	Timer::begin2();//モータ用割り込み発生開始
	
	PS2Controller c;
	STEP::reset_stp();
	bool led = false;

	while (1){
		c.update(); 
		//USART::out << (uint8_t)c.get_button(0) << "," << (uint8_t)c.get_button(1) <<" "
		//<< "Lv:" << c.get_stick(1, 1) << "Ls:" << c.get_stick(1, 0) 
		//<< "Rv:" << c.get_stick(0, 1) << "Rs:" << c.get_stick(0, 0) << USART::endl;
		if(c.pushed(tri)) {
			STEP::stp1Pulse(STEP::CW);
			USART::out << "tri pushed" << USART::endl;
		} else if (c.pushed(cr)) {
			STEP::stp1Pulse(STEP::CCW);
			USART::out << "cr pushed" << USART::endl;
		}
		if (c.pushed(tri) || c.pushed(cr)) {
			if(led){
				IO::Set(IO::user_led);
				led = false;
			} else {
				IO::Reset(IO::user_led);
				led = true;
			}
		} else {
			IO::Reset(IO::user_led);
			led = false;
		}
		
		USART::out << "$" << USART::endl;
		Delay(100000);
	}
	
	USART::out << "Exit!";//ありえない
}


void motor_control(void){

}


//Interrupt Function for TIM2
//モーターの速度調整
extern "C" void TIM2_IRQHandler(void)
{
	// グローバル割り込みから個別要因へジャンプするためにフラグチェックする
	// 今回は要因が1つしかないけど、通常はこういうスタイルで分岐する
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		// 割り込み保留ビット(=割り込み要因フラグ)をクリア
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		motor_control();
	}
}

/**************************************************************************************/

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{
	while (1)
	{
	}
}
#endif

static __IO uint32_t uwTimingDelay;
void TimingDelay_Decrement(void)
{
	if (uwTimingDelay != 0x00)
	{
		uwTimingDelay--;
	}
}


