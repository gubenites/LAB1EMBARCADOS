/**
 * 5 semestre - Eng. da Computação - Insper
 * Rafael Corsi - rafael.corsi@insper.edu.br
 *
 * Projeto 0 para a placa SAME70-XPLD
 *
 * Objetivo :
 *  - Introduzir ASF e HAL
 *  - Configuracao de clock
 *  - Configuracao pino In/Out
 *
 * Material :
 *  - Kit: ATMEL SAME70-XPLD - ARM CORTEX M7
 */

/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include "asf.h"
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/

typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t seccond;
} calendar;

#define LED_PIO           PIOC                 // periferico que controla o LED
#define LED_PIO_ID        ID_PIOC              // ID do perif�rico PIOC (controla LED)
#define LED_PIO_IDX       8                    // ID do LED no PIO
#define LED_PIO_IDX_MASK  (1 << LED_PIO_IDX)   // Mascara para CONTROLARMOS o LED

#define BUT_PIO           PIOA                 // periferico que controla o BUTTON
#define BUT_PIO_ID        ID_PIOA              // ID do perif�rico PIOC (controla BUTTON)
#define BUT_PIO_IDX       11                   // ID do BUTTON no PIO
#define BUT_PIO_IDX_MASK  (1 << BUT_PIO_IDX)   // Mascara para CONTROLARMOS o LED

//LED 1

#define OLED_LED1_PIO              PIOA
#define OLED_LED1_PIO_ID           ID_PIOA
#define OLED_LED1_PIO_IDX          0
#define OLED_LED1_PIO_IDX_MASK     (1 << OLED_LED1_PIO_IDX)

//LED 2

#define OLED_LED2_PIO              PIOC
#define OLED_LED2_PIO_ID           ID_PIOC
#define OLED_LED2_PIO_IDX          30
#define OLED_LED2_PIO_IDX_MASK     (1 << OLED_LED2_PIO_IDX)

//LED 3

#define OLED_LED3_PIO              PIOB
#define OLED_LED3_PIO_ID           ID_PIOB
#define OLED_LED3_PIO_IDX          2
#define OLED_LED3_PIO_IDX_MASK     (1 << OLED_LED3_PIO_IDX)

//BUT 1

#define OLED_BUT1_PIO              PIOD
#define OLED_BUT1_PIO_ID           ID_PIOD
#define OLED_BUT1_PIO_IDX          28
#define OLED_BUT1_PIO_IDX_MASK     (1 << OLED_BUT1_PIO_IDX)

//BUT 2

#define OLED_BUT2_PIO              PIOC
#define OLED_BUT2_PIO_ID           ID_PIOC
#define OLED_BUT2_PIO_IDX          31
#define OLED_BUT2_PIO_IDX_MASK     (1 << OLED_BUT2_PIO_IDX)

//BUT 3

#define OLED_BUT3_PIO              PIOA
#define OLED_BUT3_PIO_ID           ID_PIOA
#define OLED_BUT3_PIO_IDX          19
#define OLED_BUT3_PIO_IDX_MASK     (1 << OLED_BUT3_PIO_IDX)


/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/
volatile int play;
volatile char flag_4hz = 0;
volatile char flag_5hz = 0;
volatile Bool f_rtt_alarme = 0;
volatile char flag_rtc = 0;


/************************************************************************/
/* prototypes                                                           */
/************************************************************************/
void nextButtonFunction(void);
void playButtonFunction(void);
void prevButtonFunction(void);
void writeLCD(void);
void TC1_Handler(void);
void TC2_Handler(void);
void TC_init(void);
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);
void io_init(void);
void pin_toggle(Pio *pio, uint32_t mask);
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);

void init(void);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

void writeLCD(){
	gfx_mono_draw_string(string, 0,16, &sysfont);
}


void TC1_Handler(void){
	volatile uint32_t ul_dummy;
	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	UNUSED(ul_dummy);

	flag_4hz = 1;
}

void TC2_Handler(void){
	volatile uint32_t ul_dummy;
	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 2);

	UNUSED(ul_dummy);

	flag_5hz = 1;
}
/************************************************************************/
/* funcoes                                                              */
/************************************************************************/
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	/* O TimerCounter é meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupçcão no TC canal 0 */
	/* Interrupção no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

static float get_time_rtt(){
	uint ul_previous_time = rtt_read_timer_value(RTT);
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	
	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}

void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
      flag_rtc = 1;
	}
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {  }

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		pin_toggle(OLED_LED2_PIO, OLED_LED2_PIO_IDX_MASK);    // BLINK Led
		f_rtt_alarme = true;                  // flag RTT alarme
	}
}

// Função de inicialização do uC
void init(void)
{	
	board_init();
	// Initialize the board clock
	sysclk_init();	
	
	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	play = 0;
	
	//Inicialização clock leds
	pmc_enable_periph_clk(OLED_LED1_PIO_ID);
	pmc_enable_periph_clk(OLED_LED2_PIO_ID);
	pmc_enable_periph_clk(OLED_LED3_PIO_ID);
	
	//Inicialização clock botoes
	pmc_enable_periph_clk(OLED_BUT1_PIO_ID);
	pmc_enable_periph_clk(OLED_BUT2_PIO_ID);
	pmc_enable_periph_clk(OLED_BUT3_PIO_ID);

	//Configura leds
	pio_configure(OLED_LED1_PIO,PIO_OUTPUT_1,OLED_LED1_PIO_IDX_MASK,PIO_DEFAULT);
	pio_configure(OLED_LED2_PIO,PIO_OUTPUT_1,OLED_LED2_PIO_IDX_MASK,PIO_DEFAULT);
	pio_configure(OLED_LED3_PIO,PIO_OUTPUT_1,OLED_LED3_PIO_IDX_MASK,PIO_DEFAULT);
	
	//Configura botoes
	pio_configure(OLED_BUT1_PIO,PIO_INPUT,OLED_BUT1_PIO_IDX_MASK,PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(OLED_BUT2_PIO,PIO_INPUT,OLED_BUT2_PIO_IDX_MASK,PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(OLED_BUT3_PIO,PIO_INPUT,OLED_BUT3_PIO_IDX_MASK,PIO_PULLUP | PIO_DEBOUNCE);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade i (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(OLED_BUT1_PIO_ID);
	NVIC_SetPriority(OLED_BUT1_PIO_ID, 4);
	NVIC_EnableIRQ(OLED_BUT2_PIO_ID);
	NVIC_SetPriority(OLED_BUT2_PIO_ID, 4);
	NVIC_EnableIRQ(OLED_BUT3_PIO_ID);
	NVIC_SetPriority(OLED_BUT3_PIO_ID, 4);
	
	TC_init(TC0, ID_TC1, 1, 4);
	TC_init(TC0, ID_TC2, 2, 5);
	
	flag_4hz = 0;
	flag_5hz = 0;
	
	delay_init();
	
	gfx_mono_ssd1306_init();

}

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.seccond);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 0);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/

// Funcao principal chamada na inicalizacao do uC.
int main(void)
{
  init();
	
  writeLCD();
  // super loop
  // aplicacoes embarcadas não devem sair do while(1).
  f_rtt_alarme = true;

  while (1)
  {
	if (flag_4hz)
	{
		pio_clear(OLED_LED1_PIO,OLED_LED1_PIO_IDX_MASK);
		delay_ms(1);
		pio_set(OLED_LED1_PIO,OLED_LED1_PIO_IDX_MASK);
		delay_ms(1);
		
		flag_4hz = 0;
	}
	if (flag_5hz)
	{
		pio_clear(OLED_LED3_PIO,OLED_LED3_PIO_IDX_MASK);
		delay_ms(1);
		pio_set(OLED_LED3_PIO,OLED_LED3_PIO_IDX_MASK);
		delay_ms(1);
	}
	 if (f_rtt_alarme){
      
      /*
       * IRQ apos 4s -> 8*0.5
       */
      uint16_t pllPreScale = (int) (((float) 32768) / 4.0);
      uint32_t irqRTTvalue = 8;
      
      // reinicia RTT para gerar um novo IRQ
      RTT_init(pllPreScale, irqRTTvalue);         
      
      f_rtt_alarme = false;
    }
	if(flag_rtc){
		pio_clear(OLED_LED3_PIO,OLED_LED3_PIO_IDX_MASK);
		delay_ms(200);
		pio_set(OLED_LED3_PIO,OLED_LED3_PIO_IDX_MASK);
		delay_ms(200);
		
		flag_rtc = 0;
  }
  return 0;
}
