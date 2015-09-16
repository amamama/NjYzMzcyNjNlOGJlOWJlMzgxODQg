//nucleof401 stm32f401
//自動機プログラム

#include"stm32f4xx.h"
#include"Util.h"
#include"GPIO.h"
#include"IO.h"
#include"USART.h"
#include"SPI.h"
#include"Timer.h"
#include"ADConverter.h"
#include"PWM.h"

#include<vector>
#include<math.h>
//------------------------初期化----------------------------------

volatile void init_module(void){
	//モジュールの初期化
	IO::setting();
	USART::setting();
	SPI::setting();
	ADConverter::setting();
	Timer::setting();
	PWM::setting();
}

volatile void print_init_message(void){
	//環境出力
	USART::out << "Connected to Nucleo F401RE." << USART::endl;
	//クロック周波数の確認
	RCC_ClocksTypeDef RCC_ClockFreq;
	RCC_GetClocksFreq(&RCC_ClockFreq);
	//printf("Frequency\r\nSYSCLK:%d\r\nHCLK:%d\r\nPCLK1:%d\r\nPCLK2:%d\r\n", RCC_ClockFreq.SYSCLK_Frequency, RCC_ClockFreq.HCLK_Frequency, RCC_ClockFreq.PCLK1_Frequency, RCC_ClockFreq.PCLK2_Frequency);
	USART::out << "SYSCLK:" << RCC_ClockFreq.SYSCLK_Frequency << USART::endl
		<< "HCLK:" << RCC_ClockFreq.HCLK_Frequency << USART::endl
		<< "PCLK1:" << RCC_ClockFreq.PCLK1_Frequency << USART::endl
		<< "PCLK2:" << RCC_ClockFreq.PCLK2_Frequency << USART::endl;
}

volatile void init_motor_driver(void){
	//待つ
	Delay(0xfffff);
	//モータSPI初期化
	if ((char)SPI::transaction3(0) == -128) USART::out << "get initial value L\n\r";
	for (int i = 0; i < 20; i++) SPI::transaction3(0);
	if ((char)SPI::transaction2(0) == -128) USART::out << "get initial value R\n\r";
	for (int i = 0; i < 20; i++) SPI::transaction2(0);
}

//--------------------各種パラメータ-------------------------------
enum class Mode{
	Stop,
	Straight,
	Back,
	Linetrace,
	LinetraceTurn,
	Accell,
	Turn,
	Move,
	tenp,
	tenn,
};

const float sens_pos[8] = { -114.0f, -81.0f,-51.0f, -18.0f, +18.0f, +51.0f, +81.5f, +114.0f };//センサー位置(右に向かって正になる軸)mm単位

volatile const char speed_straight = 100;//通常時の直進スピード
volatile const char speed_back = -60;
volatile char back_acc = 0;
volatile char inf_spd = 10;
volatile char front_acc = 0;


//------------------------変数------------------------------------
volatile int time_count = 0;
volatile Mode exec_mode = Mode::Stop;

volatile uint16_t line_green[8] = { 1053, 678, 685, 939, 1085, 574, 904, 721};//{5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000};
volatile uint16_t line_white[8] = { 3581, 2225, 2392, 3328, 3521, 1918, 2888, 1931 };//{ 0, 0, 0, 0, 0, 0, 0, 0 };
volatile uint16_t line_sensor[8];
volatile float normalized_sensor[8];

volatile bool prev_is_onL = false;//初期値注意!
volatile bool prev_is_onR = false;

volatile int count_lineL_in, count_lineR_in;
volatile int count_lineL_out, count_lineR_out;

volatile float prev_pos = 0.0f;//前回の白線位置

volatile float Kp = 0.20;	//P制御の回転における比例ゲイン217-210
volatile float Kd = 0.10f;	//D制御の回転における微分ゲイン


volatile float move_straight = 0;
volatile float move_turn = 0;

//------------------------関数------------------------------------

char clamp_duty(float duty){
	return (char)clamp(duty, -120.0f, +120.0f);
}

volatile void wait_x0ms(int n){
	time_count = n;
	while (time_count);
}

//センサーのキャリブレーション
void calibulation(void){

	//白と緑の値を決める
	uint32_t tmp[8];
	for (auto& i : tmp) i = 0;

	for (int j = 0; j < 20; j++)for (int i = 0; i < 8; i++){ //無駄
		ADConverter::get(i); wait_x0ms(1);
	}
	USART::out << "calibulation green start!" << USART::endl;


	while (IO::Read(user_switch));//緑
	for (int c = 0; c < 10; c++){
		for (int i = 0; i < 8; i++){
//			tmp[i] += ADConverter::get(i);
			int t = ADConverter::get(i);
			line_green[i] = line_green[i] > t ? t : line_green[i];
			wait_x0ms(1);
		}
	}
//	for (int i = 0; i < 8; i++){
//		line_green[i] = tmp[i] / 100;
//	}
	USART::out << "green end" << USART::endl;

	//wait_x0ms(100);
	IO::Set(user_led);
	USART::out << "white start" << USART::endl;

	for (auto& i : tmp) i = 0;
		
	while (IO::Read(user_switch));//白
	for (int c = 0; c < 10; c++){
		for (int i = 0; i < 8; i++){
//			tmp[i] += ADConverter::get(i);
			int t = ADConverter::get(i);
			line_white[i] = line_white[i] < t ? t : line_white[i];
			wait_x0ms(1);
		}
	}
//	for (int i = 0; i < 8; i++){
//		line_white[i] = tmp[i] / 100;
//	}
	USART::out << "white end" << USART::endl;

	IO::Reset(user_led);
}

void update_sensor(void){
	for (int i = 0; i < 8; i++){
		line_sensor[i] = ADConverter::get(i);
	}
}

int is_on_line(int n){
	return (line_sensor[n] >= ((float)(line_white[n] - line_green[n])*0.5f + line_green[n]))?1:0;
}

void count_line(void){
	bool is_onL = is_on_line(0);
	bool is_onR = is_on_line(7);

	if (prev_is_onL && !is_onL) count_lineL_out++;
	if (prev_is_onR && !is_onR) count_lineR_out++;
	if (!prev_is_onL && is_onL) count_lineL_in++;
	if (!prev_is_onR && is_onR) count_lineR_in++;

	prev_is_onL = is_onL;
	prev_is_onR = is_onR;

}

volatile void wait_line_over(int k){
	count_lineL_out = count_lineR_out = 0;
	/*while (!((count_lineL_out + count_lineR_out) % 2 == 0 && (count_lineL_out + count_lineR_out) / 2 == k)){
//		USART::out << count_lineL_out << "," << count_lineR_out << USART::endl;
	}*/
	while((count_lineL_out + count_lineR_out)  < (k*2) )
		USART::out <<count_lineL_out<<", "<<count_lineR_out<<USART::endl;
}

void normalize_sensor(void){
	for (int i = 0; i < 8; i++){
		//センサー値の正規化
		normalized_sensor[i] = clamp((float)(line_sensor[i] - line_green[i]) / (float)(line_white[i] - line_green[i]), 0.0f, 1.0f);
	}
}

std::vector<float> search_convex(void){//normarized_sensor分布の凸を調べる,凸の位置を返す
	std::vector<float> pos_array;
	
/*
	float sum = 0.0f;
	for (int i = 0; i < 8; i++){ sum += normalized_sensor[i]; }

	float v = 0.0f;
	for (int i = 0; i < 8; i++){
		v += normalized_sensor[i] * sens_pos[i];
	}

	pos_array.push_back(v / sum);
*/
	//5%以下はゴミとする
	for (auto &x : normalized_sensor) x = (x > 0.05) ? x : 0.0f;

	for (int i = 0; i < 8; i++){
		if (normalized_sensor[i] > 0.5){
			float sum, v;
			if (i == 0){
				sum = normalized_sensor[0] + normalized_sensor[1];
				v =   normalized_sensor[0] * sens_pos[0] + normalized_sensor[1] * sens_pos[1];
			}
			else if (i == 7){
				sum = normalized_sensor[6] + normalized_sensor[7];
				v = normalized_sensor[6] * sens_pos[6] + normalized_sensor[7] * sens_pos[7];
			}
			else{
				sum = normalized_sensor[i - 1] + normalized_sensor[i] + normalized_sensor[i + 1];
				v = normalized_sensor[i - 1] * sens_pos[i-1] + normalized_sensor[i] * sens_pos[i] + normalized_sensor[i + 1] * sens_pos[i+1];
			}
			pos_array.push_back(v/sum);
		}
	}
	return pos_array;
	


}


//現在のセンサー値から白線の位置を計算する
float calc_pos(void){
	normalize_sensor();

	auto v = search_convex();
	for (auto a : v)
		//USART::out << "[" << a * 100 << "]";
	//USART::out << USART::endl;

	if (v.size() == 0){
		return prev_pos;
	}

	if (v.size() == 1){
		return v[0];
	}

	//prev_posと近いものを白線の位置とする
	float x = 0.0f,d = 1000.0f;
	for (auto y : v){
		float h = fabs(y - prev_pos);
		if (h < d){
			d = h;
			x = y;
		}
	}

	return x;
}

volatile void stop(int n){
	exec_mode = Mode::Stop;
	wait_x0ms(n);
}

volatile void linetrace(int k){
	exec_mode = Mode::Linetrace;
	prev_pos = calc_pos();
	count_lineL_out = count_lineR_out = 0;
//	count_lineL_in = count_lineR_in = 0;
	volatile char prev_spd = speed_straight;
	int flag = 0;
	while (count_lineL_out + count_lineR_out < k*2){
		//USART::out <<count_lineL_out<<", "<<count_lineR_out<<USART::endl;
		if(flag == 0 && (k-2)*2 <= count_lineL_out + count_lineR_out) {
			flag = 1,front_acc = -1;
			USART::out <<count_lineL_out<<", "<<count_lineR_out<<USART::endl;
		}

/*
		if (count_lineL_in || count_lineR_in){
			//線に侵入するときの角度をそろえるルーチン
			exec_mode = Mode::Move;
			bool Lin = true, Rin = true;
			while (Lin || Rin){
				if (Lin&&count_lineL_in){
					//左が侵入
					//左を止めて右を動かす
					move_straight = 30; move_turn = 30;
					Lin = false;
					count_lineL_in = 0;
				}
				if (Rin&&count_lineR_in){
					//右が侵入
					//右を止めて左を動かす
					move_straight = 30; move_turn = -30;
					Rin = false;
					count_lineR_in = 0;
				}
			}
			exec_mode = Mode::Linetrace;
			count_lineL_in = count_lineR_in = 0;
		}
*/
	}

}

volatile void straight(int k){
	exec_mode = Mode::Straight;
	wait_line_over(k);
}

volatile void back(int k){
	exec_mode = Mode::Back;
	back_acc = 50;
	const int table[] = {5, 5, 4, 2, 3, 3, 3, 4};
	wait_line_over(k);
}

volatile void turn(int n){
	exec_mode = Mode::Turn;
	wait_x0ms(n);
}

volatile void turnL(void){
	//両端センサが入るまで減速->低速度
	//左を止めて右を動かす
	//二本見るまで動かす
	//今までより遠いほうを目標にして直進成分0でpidしてみる
	exec_mode = Mode::Move;
	count_lineL_in = count_lineR_in = 0;
	move_straight = 40; move_turn = 0;
	//線に侵入するときの角度をそろえる
	bool Lin = true, Rin = true;
	while (Lin || Rin){
		if (Lin&&count_lineL_in){
			//左が侵入
			//左を止めて右を動かす
			move_straight = 20; move_turn = 20;
			Lin = false;
		}
		if (Rin&&count_lineR_in){
			//右が侵入
			//右を止めて左を動かす
			move_straight = 20; move_turn = -20;
			Rin = false;
		}
	}
	
	move_straight = -20; move_turn = 20;
	wait_x0ms(20);


	std::vector<float> v;
	volatile int count = 0;
	do{
		normalize_sensor();
		v = search_convex();
		if (v.size() == 2) count++;
		wait_x0ms(2);
	} while (count<2);

	USART::out << "B";


	//v[0] < v[1]のはずだからv[0]が0に来るようにpidする
	prev_pos = min(v[0],v[1]);
	exec_mode = Mode::LinetraceTurn;
	while (prev_pos < 0){//真ん中超えたらおわり
//		v = search_convex();
//		while (v.size() == 0){
//			exec_mode = Mode::Turn;
//		}
//		exec_mode = Mode::LinetraceTurn;
	}
	
	
	exec_mode = Mode::Stop;
}

void hold(void) {
	PWM::duty(3, 0.075);//right
	PWM::duty(2, 0.075);//left
}

void unhold(void) {
	PWM::duty(3, 0.04);//right
	PWM::duty(2, 0.11);//left
}
void move(float straight, float turn); //進行方向速度と回転方向速度を指定、ターンの正は反時計回り

int main(void){
	//一連の初期化
	call_constractor();
	SystemInit();
	init_module();

	Delay(0x100000);
	
	print_init_message();
	init_motor_driver();
	
	USART::out << "initializing success!" << USART::endl;
	
	USART::out << "and begin Timer2" << USART::endl;
	Timer::begin2();//モータ用割り込み発生開始
	
	//calibulation();//センサーキャリブレーション
	USART::out << line_green[0] << "," << line_green[1] << "," << line_green[2] << "," << line_green[3] << "," << line_green[4] << "," << line_green[5] << "," << line_green[6] << "," << line_green[7] << USART::endl;
	USART::out << line_white[0] << "," << line_white[1] << "," << line_white[2] << "," << line_white[3] << "," << line_white[4] << "," << line_white[5] << "," << line_white[6] << "," << line_white[7] << USART::endl;
	//入力待ち
	unhold();
	while (IO::Read(user_switch));
	USART::out << "start!" << USART::endl;

	linetrace(8);
	USART::out << "示談" << USART::endl;
	move(10,0);
	wait_x0ms(10);
	stop(10);
	while (IO::Read(user_switch));
	back(8);
	stop(10);
	while(1);

	straight(8);
	hold();
	back(8);
	stop(100);
	exec_mode = Mode::tenp;
	wait_x0ms(80);
	exec_mode = Mode::tenn;
	wait_x0ms(80);
	while(1);
	//turnL();/*linetrace(3);*/ stop(100); while (1);//back(3); stop(100);
	//straight(6);stop(100);
	//linetrace(6); stop(100); back(6); stop(100); while (1);
	//turn(1000);
	
	
	while (1){
		//update_sensor();
		//USART::out << line_sensor[0] << "," << line_sensor[1] << "," << line_sensor[2] << "," << line_sensor[3] << "," << line_sensor[4] << "," << line_sensor[5] << "," << line_sensor[6] << "," << line_sensor[7] << USART::endl;
		float pos = calc_pos();
		USART::out << normalized_sensor[0] * 1000 << "," << normalized_sensor[1] * 1000 << "," << normalized_sensor[2] * 1000 << "," << normalized_sensor[3] * 1000 << "," << normalized_sensor[4] * 1000 << "," << normalized_sensor[5] * 1000 << "," << normalized_sensor[6] * 1000 << "," << normalized_sensor[7] * 1000 << USART::endl;
		USART::out << "pos: " << pos * 100 << USART::endl;
		USART::out<< USART::endl;
		wait_x0ms(50);
	}
	

	while (1);

	USART::out << "Exit!";//ありえない
}

//-----------------------------割り込み関数-----------------------------------

void transaction_duty(char dutyL,char dutyR){

	static char prev_dutyL = 0;//エラー検出用
	static char prev_dutyR = 0;//エラー検出用
	
	//データ転送
	if ((char)SPI::transaction3((uint8_t)dutyL) != prev_dutyL){
		USART::out << "errL" << USART::endl;//ありえない
	}

	if ((char)SPI::transaction2((uint8_t)dutyR) != prev_dutyR){
		USART::out << "errR" << USART::endl;//ありえない
	}

	//過去値の更新
	prev_dutyL = dutyL;
	prev_dutyR = dutyR;
}

void move(float straight, float turn){ //進行方向速度と回転方向速度を指定、ターンの正は反時計回り
	char L = clamp_duty(straight - turn);
	char R = clamp_duty(straight + turn);
//	USART::out <<L<< "," <<R<< USART::endl;
	transaction_duty(L,R);
}

void mode_linetrace(float speed,float p,float d){
	int count = is_on_line(0) + is_on_line(1) + is_on_line(2)
		+ is_on_line(3) + is_on_line(4) + is_on_line(5) + is_on_line(6);
	//if (count==0){
	//	move(0, 0);
	//	return;
	//}
	/*if (count>=4){
		move(speed, 0);
		return;
	}*/
/*
	if (!is_on_line(0) && !is_on_line(1) && !is_on_line(2)
		&& !is_on_line(3) && !is_on_line(4) && !is_on_line(5) && !is_on_line(6)){//すべて緑上または黒線上
		move(speed_straight, 0);
	}
*/
	float pos = calc_pos();//中心が0、車体が右にあるとき負、左にあるとき正
	//PD制御もどき　目標値は0
	//USART::out << -Kp*pos - Kd*(pos - prev_pos) << USART::endl;
	move(speed, -p*pos - d*(pos - prev_pos));//-Kp*(pos-0)-Kd((pos-0) - (prev_pos-0))
	prev_pos = pos;
}
/*
void mode_linetrace_straight(float speed, float p, float d){
	bool line[8] = { false };
	for (int i = 0; i < 8; i++) line[i] = is_on_line(i);
	

	if (){
		return;
	}
	else if (){
		return;
	}
	else{

		float pos = calc_pos();//中心が0、車体が右にあるとき負、左にあるとき正
		//PD制御もどき　目標値は0
		//USART::out << -Kp*pos - Kd*(pos - prev_pos) << USART::endl;
		move(speed, -p*pos - d*(pos - prev_pos));//-Kp*(pos-0)-Kd((pos-0) - (prev_pos-0))
		prev_pos = pos;

	}

	//線に侵入するときの角度をそろえる
	bool Lin = true, Rin = true;
	while (Lin || Rin){
		if (Lin&&count_lineL_in){
			//左が侵入
			//左を止めて右を動かす
			move_straight = 20; move_turn = 20;
			Lin = false;
		}
		if (Rin&&count_lineR_in){
			//右が侵入
			//右を止めて左を動かす
			move_straight = 20; move_turn = -20;
			Rin = false;
		}
	}




}
*/
void motor_control(void){

	//タイマーをカウントする
	if (time_count>0) time_count--;

	//センサ関係
	update_sensor();
	count_line();

	switch (exec_mode){
	case Mode::Stop:
		transaction_duty(0,0);
		break;
	case Mode::Straight:
		move(120,0);
		break;
	case Mode::Back:
		if(back_acc > 0) back_acc--;
		move(speed_back + back_acc, 2);
		//mode_linetrace(-speed_straight, -Kp, -Kd);
		break;
	case Mode::Linetrace:
		if(speed_straight + front_acc > 10 && -90 < front_acc && front_acc < 0) front_acc--;
		mode_linetrace(speed_straight + front_acc,Kp,Kd);
		break;
	case Mode::LinetraceTurn:
		mode_linetrace(0,0.5,0);
		break;
	case Mode::Turn:
		move(0, speed_straight);
		break;
	case Mode::Move:
		move(move_straight,move_turn);
		break;
	case Mode::tenp:
		move(0,40);
		break;
	case Mode::tenn:
		move(0,-40);
		break;
	default: break;
	}

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


