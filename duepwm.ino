
void setPWMpin(uint32_t pin) {
  PIO_Configure(g_APinDescription[pin].pPort,
                PIO_PERIPH_B, //hack Arduino does not allow high PWM by default
                g_APinDescription[pin].ulPin,
                g_APinDescription[pin].ulPinConfiguration);
}

void setup() {
  
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

  for (int ch=0; ch<8; ch++) {
    PWMC_ConfigureChannel(PWM,ch,PWM_CMR_CPRE_CLKA,0,0);
    PWMC_SetPeriod(PWM,ch,20000); // 20ms
    PWMC_SetDutyCycle(PWM,ch,1000); //1ms
    PWMC_EnableChannel(PWM,ch);
  }
}

uint32_t duty=1000;

void loop() {
  // put your main code here, to run repeatedly: 
  delay(1000);
  if (duty==1000) {
    duty=2000;
  } else {
    duty=1000;
  }
  for (uint32_t ch=0; ch<8; ch++) {
    PWMC_SetDutyCycle(PWM,ch,duty);
  }  
}
