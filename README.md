# ESOE_Capstone

## Status
- have tested
  - 可以同時接收兩顆超音波的訊號
- planned but have not tested yet
  - 同時接收三顆超音波
- have not planned
  - 馬達PWM腳位
  - 驅動IC

## Settings
- 超音波
  - Trigger Signal(GPIO): PA1, PC0, PB0
  - Internal clock: TIM1
  - Echo(Input Capture): TIM2(PA0), TIM3(PA6), TIM4(PA11)

