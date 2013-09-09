#define USE_PPM 0
//#define USE_PPM 1



void setPWMpin(uint32_t pin) {
  PIO_Configure(g_APinDescription[pin].pPort,
                PIO_PERIPH_B, //hack Arduino does not allow high PWM by default
                g_APinDescription[pin].ulPin,
                g_APinDescription[pin].ulPinConfiguration);
}

void setupPWMs() {
  setPWMpin(34); //PWM L0 
  setPWMpin(36); //PWM L1 
  setPWMpin(38); //PWM L2 
  setPWMpin(40); //PWM L3 
  setPWMpin(9);  //PWM L4 
  setPWMpin(8);  //PWM L5 
  setPWMpin(7);  //PWM L6 
  setPWMpin(6);  //PWM L7 

  pmc_enable_periph_clk(ID_PWM);
  for (int ch=0; ch<8; ch++) {
    PWMC_DisableChannel(PWM, ch);
  }

  // set PWM clock A to 1MHz
  PWMC_ConfigureClocks(1000000,0,VARIANT_MCK);

  for (int ch=7; ch>=0; ch--) {
    PWMC_ConfigureChannel(PWM,ch,PWM_CMR_CPRE_CLKA,0,0);
    PWMC_SetPeriod(PWM,ch,2500); // 2.5ms -> 400Hz
    PWMC_SetDutyCycle(PWM,ch,1000); //1ms
    PWMC_EnableChannel(PWM,ch);
  }
}

void setPWM(uint8_t ch, uint16_t val) {
  PWMC_SetDutyCycle(PWM,ch,val);
}

void setTCpin(uint32_t pin) {
  PIO_Configure(g_APinDescription[pin].pPort,
                g_APinDescription[pin].ulPinType,
                g_APinDescription[pin].ulPin,
                g_APinDescription[pin].ulPinConfiguration);
}

// setup 6 PWM outputs on TC2: TIO[AB][6-8] 
void setupTCouts()
{
  pmc_enable_periph_clk(ID_TC6);
  pmc_enable_periph_clk(ID_TC7);
  pmc_enable_periph_clk(ID_TC8);
  TC_Configure(TC2, 0, TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET | TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_SET | TC_CMR_EEVT_XC0);
  TC_Configure(TC2, 1, TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET | TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_SET | TC_CMR_EEVT_XC0);
  TC_Configure(TC2, 2, TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET | TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_SET | TC_CMR_EEVT_XC0);
  TC_SetRC(TC2, 0, 840000); // 20ms
  TC_SetRA(TC2, 0, 42000);
  TC_SetRB(TC2, 0, 42000);
  TC_SetRC(TC2, 1, 840000); // 20ms
  TC_SetRA(TC2, 1, 42000);
  TC_SetRB(TC2, 1, 42000);
  TC_SetRC(TC2, 2, 840000); // 20ms
  TC_SetRA(TC2, 2, 42000);
  TC_SetRB(TC2, 2, 42000);
  setTCpin(5);
  setTCpin(4);
  setTCpin(3);
  setTCpin(10);
  setTCpin(11);
  setTCpin(12);
  TC_Start(TC2,0);
  TC_Start(TC2,1);
  TC_Start(TC2,2);
}

void setPWM2(uint8_t ch, uint16_t val) {
  if (ch < 6) {
    if (ch & 1) {
      TC_SetRB(TC2, ch >> 1, val * 42);
    } else {
      TC_SetRA(TC2, ch >> 1, val * 42);
    }
  }
}

volatile uint16_t PPMt[16]; // unvalidated input
volatile uint16_t PPM[16];
volatile uint8_t  PPMch = 255;
volatile uint32_t PPMlast=0;

void TC1_Handler() {
  if (TC0->TC_CHANNEL[1].TC_SR & TC_SR_LDRAS) {
    uint32_t out, now = TC0->TC_CHANNEL[1].TC_RA;
    out = (now - PPMlast) / 42;
    PPMlast = now;
    if ((out >= 750) && (out < 2250)) {
      // valid pulse...
      if (PPMch < 16) {
        PPMt[PPMch++] = out;
      }
    } else if (out > 2500) {
      // valid sync pulse
      if (PPMch <=16) {
        for (uint8_t i = 0; i < PPMch; i++) {
          PPM[i] = PPMt[i];
        }
      } 
      PPMch=0;
    } else {
      // glitch
      PPMch=255;
    }
  }
}

uint32_t pwmLast[8];

void pwmHandler(uint8_t ch, uint32_t pin) {
  uint32_t cv = TC0->TC_CHANNEL[1].TC_CV;
  if (digitalRead(pin)) {
    pwmLast[ch] = cv;
  } else {
    PPM[ch] = (cv - pwmLast[ch]) / 42;
  }
}

void ch1Handler() { pwmHandler(0, 44); }
void ch2Handler() { pwmHandler(1, 45); }
void ch3Handler() { pwmHandler(2, 46); }
void ch4Handler() { pwmHandler(3, 47); }
void ch5Handler() { pwmHandler(4, 48); }
void ch6Handler() { pwmHandler(5, 49); }
void ch7Handler() { pwmHandler(6, 50); }
void ch8Handler() { pwmHandler(7, 51); }

void setupInput(int isPPM)
{
  pmc_enable_periph_clk(ID_TC1);
  if (isPPM) {
    TC_Configure(TC0, 1, TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_LDRA_RISING | TC_CMR_LDRB_FALLING);
    setTCpin(A7);
    NVIC_EnableIRQ(TC1_IRQn);
    TC0->TC_CHANNEL[1].TC_IER = TC_IER_LDRAS;
    TC_Start(TC0,1);
  } else {
    // use timer just for timing reference
    TC_Configure(TC0, 1, TC_CMR_TCCLKS_TIMER_CLOCK1);
    TC_Start(TC0,1);
    attachInterrupt(44,ch1Handler,CHANGE);
    attachInterrupt(45,ch2Handler,CHANGE);
    attachInterrupt(46,ch3Handler,CHANGE);
    attachInterrupt(47,ch4Handler,CHANGE);
    attachInterrupt(48,ch5Handler,CHANGE);
    attachInterrupt(49,ch6Handler,CHANGE);
    attachInterrupt(50,ch7Handler,CHANGE);
    attachInterrupt(51,ch8Handler,CHANGE);
  }
}


void setup() {
  Serial.begin(115200);
  setupPWMs();
  setupTCouts();
  setupInput(USE_PPM);
}

uint32_t duty=1000;

void loop() {
  // put your main code here, to run repeatedly: 
  for (int i=0; i<16; i++) {
    Serial.print(PPM[i]);
    Serial.print(' ');
  }
  Serial.println();

  delay(100);

  duty+=50;
  if (duty>2000) {
    duty=1000;
  }

  for (uint32_t ch=0; ch<8; ch++) {
    setPWM(ch, duty);
  }
   for (uint32_t ch=0; ch<6; ch++) {
    setPWM2(ch, 3000-duty);
  }  

}
