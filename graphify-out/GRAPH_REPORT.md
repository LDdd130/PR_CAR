# Graph Report - PR_CAR  (2026-07-04)

## Corpus Check
- 168 files · ~870,960 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 3451 nodes · 8760 edges · 96 communities (92 shown, 4 thin omitted)
- Extraction: 92% EXTRACTED · 8% INFERRED · 0% AMBIGUOUS · INFERRED: 704 edges (avg confidence: 0.8)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `f0a74d20`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- [[_COMMUNITY_Community 0|Community 0]]
- [[_COMMUNITY_Community 1|Community 1]]
- [[_COMMUNITY_Community 2|Community 2]]
- [[_COMMUNITY_Community 3|Community 3]]
- [[_COMMUNITY_Community 4|Community 4]]
- [[_COMMUNITY_Community 5|Community 5]]
- [[_COMMUNITY_Community 6|Community 6]]
- [[_COMMUNITY_Community 7|Community 7]]
- [[_COMMUNITY_Community 8|Community 8]]
- [[_COMMUNITY_Community 9|Community 9]]
- [[_COMMUNITY_Community 10|Community 10]]
- [[_COMMUNITY_Community 11|Community 11]]
- [[_COMMUNITY_Community 12|Community 12]]
- [[_COMMUNITY_Community 13|Community 13]]
- [[_COMMUNITY_Community 14|Community 14]]
- [[_COMMUNITY_Community 15|Community 15]]
- [[_COMMUNITY_Community 16|Community 16]]
- [[_COMMUNITY_Community 17|Community 17]]
- [[_COMMUNITY_Community 18|Community 18]]
- [[_COMMUNITY_Community 19|Community 19]]
- [[_COMMUNITY_Community 20|Community 20]]
- [[_COMMUNITY_Community 21|Community 21]]
- [[_COMMUNITY_Community 22|Community 22]]
- [[_COMMUNITY_Community 23|Community 23]]
- [[_COMMUNITY_Community 24|Community 24]]
- [[_COMMUNITY_Community 25|Community 25]]
- [[_COMMUNITY_Community 26|Community 26]]
- [[_COMMUNITY_Community 27|Community 27]]
- [[_COMMUNITY_Community 28|Community 28]]
- [[_COMMUNITY_Community 29|Community 29]]
- [[_COMMUNITY_Community 30|Community 30]]
- [[_COMMUNITY_Community 31|Community 31]]
- [[_COMMUNITY_Community 32|Community 32]]
- [[_COMMUNITY_Community 33|Community 33]]
- [[_COMMUNITY_Community 34|Community 34]]
- [[_COMMUNITY_Community 35|Community 35]]
- [[_COMMUNITY_Community 36|Community 36]]
- [[_COMMUNITY_Community 37|Community 37]]
- [[_COMMUNITY_Community 38|Community 38]]
- [[_COMMUNITY_Community 39|Community 39]]
- [[_COMMUNITY_Community 40|Community 40]]
- [[_COMMUNITY_Community 41|Community 41]]
- [[_COMMUNITY_Community 42|Community 42]]
- [[_COMMUNITY_Community 43|Community 43]]
- [[_COMMUNITY_Community 44|Community 44]]
- [[_COMMUNITY_Community 45|Community 45]]
- [[_COMMUNITY_Community 46|Community 46]]
- [[_COMMUNITY_Community 47|Community 47]]
- [[_COMMUNITY_Community 48|Community 48]]
- [[_COMMUNITY_Community 49|Community 49]]
- [[_COMMUNITY_Community 50|Community 50]]
- [[_COMMUNITY_Community 51|Community 51]]
- [[_COMMUNITY_Community 52|Community 52]]
- [[_COMMUNITY_Community 53|Community 53]]
- [[_COMMUNITY_Community 54|Community 54]]
- [[_COMMUNITY_Community 55|Community 55]]
- [[_COMMUNITY_Community 56|Community 56]]
- [[_COMMUNITY_Community 57|Community 57]]
- [[_COMMUNITY_Community 58|Community 58]]
- [[_COMMUNITY_Community 59|Community 59]]
- [[_COMMUNITY_Community 60|Community 60]]
- [[_COMMUNITY_Community 61|Community 61]]
- [[_COMMUNITY_Community 62|Community 62]]
- [[_COMMUNITY_Community 63|Community 63]]
- [[_COMMUNITY_Community 64|Community 64]]
- [[_COMMUNITY_Community 65|Community 65]]
- [[_COMMUNITY_Community 66|Community 66]]
- [[_COMMUNITY_Community 67|Community 67]]
- [[_COMMUNITY_Community 68|Community 68]]
- [[_COMMUNITY_Community 69|Community 69]]
- [[_COMMUNITY_Community 70|Community 70]]
- [[_COMMUNITY_Community 71|Community 71]]
- [[_COMMUNITY_Community 72|Community 72]]
- [[_COMMUNITY_Community 73|Community 73]]
- [[_COMMUNITY_Community 74|Community 74]]
- [[_COMMUNITY_Community 75|Community 75]]
- [[_COMMUNITY_Community 76|Community 76]]
- [[_COMMUNITY_Community 77|Community 77]]
- [[_COMMUNITY_Community 78|Community 78]]
- [[_COMMUNITY_Community 79|Community 79]]
- [[_COMMUNITY_Community 80|Community 80]]
- [[_COMMUNITY_Community 81|Community 81]]
- [[_COMMUNITY_Community 84|Community 84]]
- [[_COMMUNITY_Community 85|Community 85]]
- [[_COMMUNITY_Community 86|Community 86]]
- [[_COMMUNITY_Community 87|Community 87]]
- [[_COMMUNITY_Community 88|Community 88]]
- [[_COMMUNITY_Community 89|Community 89]]
- [[_COMMUNITY_Community 91|Community 91]]
- [[_COMMUNITY_Community 92|Community 92]]
- [[_COMMUNITY_Community 94|Community 94]]

## God Nodes (most connected - your core abstractions)
1. `__DSB()` - 97 edges
2. `__ISB()` - 77 edges
3. `HAL_GetTick()` - 45 edges
4. `TIM_CCxChannelCmd()` - 40 edges
5. `uxListRemove()` - 28 edges
6. `xTaskResumeAll()` - 27 edges
7. `HAL_DMA_Start_IT()` - 26 edges
8. `HAL_DMA_Abort_IT()` - 23 edges
9. `vTaskSuspendAll()` - 23 edges
10. `TIM_CCxNChannelCmd()` - 18 edges

## Surprising Connections (you probably didn't know these)
- `StartTask03()` --calls--> `osMessageQueueGet()`  [INFERRED]
  Core/Src/freertos.c → Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
- `HAL_MspInit()` --calls--> `HAL_NVIC_SetPriority()`  [INFERRED]
  Core/Src/stm32f4xx_hal_msp.c → Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
- `TIM3_IRQHandler()` --calls--> `HAL_TIM_IRQHandler()`  [INFERRED]
  Core/Src/stm32f4xx_it.c → Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
- `USART1_IRQHandler()` --calls--> `HAL_UART_IRQHandler()`  [INFERRED]
  Core/Src/stm32f4xx_it.c → Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
- `bno_bus_recover()` --calls--> `HAL_GPIO_Init()`  [INFERRED]
  Core/Src/bno055.c → Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c

## Import Cycles
- None detected.

## Communities (96 total, 4 thin omitted)

### Community 0 - "Community 0"
Cohesion: 0.03
Nodes (180): __STATIC_INLINE, TIM_TypeDef, LL_TIM_CC_DisableChannel(), LL_TIM_CC_DisablePreload(), LL_TIM_CC_EnableChannel(), LL_TIM_CC_EnablePreload(), LL_TIM_CC_GetDMAReqTrigger(), LL_TIM_CC_IsEnabledChannel() (+172 more)

### Community 1 - "Community 1"
Cohesion: 0.04
Nodes (136): __STATIC_INLINE, LL_USART_ClearFlag_FE(), LL_USART_ClearFlag_IDLE(), LL_USART_ClearFlag_LBD(), LL_USART_ClearFlag_nCTS(), LL_USART_ClearFlag_NE(), LL_USART_ClearFlag_ORE(), LL_USART_ClearFlag_PE() (+128 more)

### Community 2 - "Community 2"
Cohesion: 0.03
Nodes (132): __CLREX(), __CLZ(), __cmsis_start(), __disable_fault_irq(), __disable_irq(), __enable_fault_irq(), __enable_irq(), __get_APSR() (+124 more)

### Community 3 - "Community 3"
Cohesion: 0.03
Nodes (115): __CLZ(), __disable_fault_irq(), __disable_irq(), __enable_fault_irq(), __enable_irq(), __get_APSR(), __get_BASEPRI(), __get_CONTROL() (+107 more)

### Community 4 - "Community 4"
Cohesion: 0.05
Nodes (111): __STATIC_INLINE, LL_I2C_AcknowledgeNextData(), LL_I2C_ClearFlag_ADDR(), LL_I2C_ClearFlag_AF(), LL_I2C_ClearFlag_ARLO(), LL_I2C_ClearFlag_BERR(), LL_I2C_ClearFlag_OVR(), LL_I2C_ClearFlag_STOP() (+103 more)

### Community 5 - "Community 5"
Cohesion: 0.04
Nodes (101): MX_FREERTOS_Init(), TIM_HandleTypeDef, HAL_TIM_IC_CaptureCallback(), Ultra_Measure(), MemPool_t, AllocBlock(), osThreadId_t, StackType_t (+93 more)

### Community 6 - "Community 6"
Cohesion: 0.12
Nodes (43): FlagStatus, HAL_StatusTypeDef, UART_HandleTypeDef, HAL_HalfDuplex_EnableReceiver(), HAL_HalfDuplex_EnableTransmitter(), HAL_LIN_SendBreak(), HAL_MultiProcessor_EnterMuteMode(), HAL_MultiProcessor_ExitMuteMode() (+35 more)

### Community 7 - "Community 7"
Cohesion: 0.06
Nodes (83): configSTACK_DEPTH_TYPE, eNotifyAction, eTaskState, MemoryRegion_t, UBaseType_t, uxListRemove(), vListInsertEnd(), BaseType_t (+75 more)

### Community 8 - "Community 8"
Cohesion: 0.02
Nodes (84): LL_RCC_ClearFlag_HSECSS(), LL_RCC_ClearFlag_HSIRDY(), LL_RCC_ClearFlag_PLLI2SRDY(), LL_RCC_ClearResetFlags(), LL_RCC_DisableIT_HSERDY(), LL_RCC_DisableIT_LSERDY(), LL_RCC_DisableIT_LSIRDY(), LL_RCC_DisableIT_PLLSAIRDY() (+76 more)

### Community 9 - "Community 9"
Cohesion: 0.02
Nodes (85): __STATIC_INLINE, LL_RCC_ClearFlag_HSERDY(), LL_RCC_ClearFlag_LSERDY(), LL_RCC_ClearFlag_LSIRDY(), LL_RCC_ClearFlag_PLLRDY(), LL_RCC_ClearFlag_PLLSAIRDY(), LL_RCC_ConfigMCO(), LL_RCC_DisableIT_HSIRDY() (+77 more)

### Community 10 - "Community 10"
Cohesion: 0.08
Nodes (76): HAL_DMA_Abort_IT(), HAL_StatusTypeDef, HAL_TIM_ChannelStateTypeDef, HAL_TIM_StateTypeDef, TIM_HandleTypeDef, HAL_TIM_Base_DeInit(), HAL_TIM_Base_GetState(), HAL_TIM_Base_MspDeInit() (+68 more)

### Community 11 - "Community 11"
Cohesion: 0.05
Nodes (69): __CLZ(), __disable_fault_irq(), __disable_irq(), __enable_fault_irq(), __enable_irq(), __get_APSR(), __get_BASEPRI(), __get_CONTROL() (+61 more)

### Community 12 - "Community 12"
Cohesion: 0.11
Nodes (64): osMessageQueueGetCount(), xCoRoutineRemoveFromEventList(), BaseType_t, TaskHandle_t, TickType_t, UBaseType_t, pcQueueGetName(), prvCopyDataFromQueue() (+56 more)

### Community 13 - "Community 13"
Cohesion: 0.10
Nodes (47): DCB_GetAuthCtrl(), DCB_SetAuthCtrl(), DIB_GetAuthStatus(), IRQn_Type, __STATIC_INLINE, ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar() (+39 more)

### Community 14 - "Community 14"
Cohesion: 0.06
Nodes (62): __STATIC_INLINE, LL_PWR_ClearFlag_SB(), LL_PWR_ClearFlag_UD(), LL_PWR_ClearFlag_WU(), LL_PWR_DisableBkUpAccess(), LL_PWR_DisableBkUpRegulator(), LL_PWR_DisableFLASHInterfaceSTOP(), LL_PWR_DisableFLASHMemorySTOP() (+54 more)

### Community 15 - "Community 15"
Cohesion: 0.08
Nodes (56): DCB_GetAuthCtrl(), DCB_SetAuthCtrl(), DIB_GetAuthStatus(), IRQn_Type, __STATIC_INLINE, ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar() (+48 more)

### Community 16 - "Community 16"
Cohesion: 0.07
Nodes (55): __CLZ(), __get_APSR(), __get_MSPLIM(), __get_PSPLIM(), __packed, __STATIC_FORCEINLINE, __STATIC_INLINE, __iar_u32() (+47 more)

### Community 17 - "Community 17"
Cohesion: 0.10
Nodes (52): DMA_HandleTypeDef, HAL_StatusTypeDef, HAL_TIM_ChannelStateTypeDef, HAL_TIM_StateTypeDef, TIM_HandleTypeDef, TIM_TypeDef, __weak, HAL_TIMEx_BreakCallback() (+44 more)

### Community 18 - "Community 18"
Cohesion: 0.04
Nodes (49): DMA_TypeDef, LL_DMA_ClearFlag_DME1(), LL_DMA_ClearFlag_DME2(), LL_DMA_ClearFlag_DME4(), LL_DMA_ClearFlag_DME7(), LL_DMA_ClearFlag_HT0(), LL_DMA_ClearFlag_HT3(), LL_DMA_ClearFlag_TC5() (+41 more)

### Community 19 - "Community 19"
Cohesion: 0.04
Nodes (48): LL_DMA_ClearFlag_DME3(), LL_DMA_ClearFlag_DME5(), LL_DMA_ClearFlag_FE0(), LL_DMA_ClearFlag_FE6(), LL_DMA_ClearFlag_FE7(), LL_DMA_ClearFlag_HT2(), LL_DMA_ClearFlag_TC2(), LL_DMA_ClearFlag_TC3() (+40 more)

### Community 20 - "Community 20"
Cohesion: 0.04
Nodes (49): __STATIC_INLINE, LL_DMA_ClearFlag_DME0(), LL_DMA_ClearFlag_DME6(), LL_DMA_ClearFlag_FE1(), LL_DMA_ClearFlag_FE2(), LL_DMA_ClearFlag_FE3(), LL_DMA_ClearFlag_FE4(), LL_DMA_ClearFlag_FE5() (+41 more)

### Community 21 - "Community 21"
Cohesion: 0.10
Nodes (45): HAL_StatusTypeDef, __weak, HAL_StatusTypeDef, FLASH_Erase_Sector(), FLASH_FlushCaches(), FLASH_MassErase(), FLASH_OB_BootConfig(), FLASH_OB_BOR_LevelConfig() (+37 more)

### Community 22 - "Community 22"
Cohesion: 0.10
Nodes (46): DCB_GetAuthCtrl(), DCB_SetAuthCtrl(), DIB_GetAuthStatus(), IRQn_Type, __STATIC_INLINE, ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar() (+38 more)

### Community 23 - "Community 23"
Cohesion: 0.10
Nodes (46): DCB_GetAuthCtrl(), DCB_SetAuthCtrl(), DIB_GetAuthStatus(), IRQn_Type, __STATIC_INLINE, ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar() (+38 more)

### Community 24 - "Community 24"
Cohesion: 0.11
Nodes (43): DCB_GetAuthCtrl(), DIB_GetAuthStatus(), IRQn_Type, __STATIC_INLINE, ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ() (+35 more)

### Community 25 - "Community 25"
Cohesion: 0.10
Nodes (47): DCB_GetAuthCtrl(), DCB_SetAuthCtrl(), DIB_GetAuthStatus(), IRQn_Type, __STATIC_INLINE, ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar() (+39 more)

### Community 26 - "Community 26"
Cohesion: 0.11
Nodes (43): DCB_GetAuthCtrl(), DIB_GetAuthStatus(), IRQn_Type, __STATIC_INLINE, ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ() (+35 more)

### Community 27 - "Community 27"
Cohesion: 0.12
Nodes (45): HAL_DMA_Start_IT(), HAL_StatusTypeDef, I2C_HandleTypeDef, HAL_I2C_AddrCallback(), HAL_I2C_DeInit(), HAL_I2C_DisableListen_IT(), HAL_I2C_EnableListen_IT(), HAL_I2C_GetError() (+37 more)

### Community 28 - "Community 28"
Cohesion: 0.05
Nodes (41): LL_DBGMCU_APB2_GRP1_FreezePeriph(), LL_DBGMCU_APB2_GRP1_UnFreezePeriph(), LL_DBGMCU_DisableDBGStandbyMode(), LL_DBGMCU_DisableDBGStopMode(), LL_DBGMCU_EnableDBGSleepMode(), LL_DBGMCU_EnableDBGStopMode(), LL_DBGMCU_GetRevisionID(), LL_DBGMCU_GetTracePinAssignment() (+33 more)

### Community 29 - "Community 29"
Cohesion: 0.05
Nodes (42): __STATIC_INLINE, LL_DBGMCU_APB1_GRP1_FreezePeriph(), LL_DBGMCU_APB1_GRP1_UnFreezePeriph(), LL_DBGMCU_DisableDBGSleepMode(), LL_DBGMCU_EnableDBGStandbyMode(), LL_DBGMCU_GetDeviceID(), LL_FLASH_DisableDataCacheReset(), LL_FLASH_DisableInstCache() (+34 more)

### Community 30 - "Community 30"
Cohesion: 0.12
Nodes (39): DCB_GetAuthCtrl(), DCB_SetAuthCtrl(), DIB_GetAuthStatus(), IRQn_Type, __STATIC_INLINE, __NVIC_ClearPendingIRQ(), NVIC_ClearTargetState(), NVIC_DecodePriority() (+31 more)

### Community 31 - "Community 31"
Cohesion: 0.14
Nodes (29): EventBits_t, EventGroupHandle_t, osEventFlagsClear(), osEventFlagsDelete(), osEventFlagsGet(), osEventFlagsNew(), osEventFlagsSet(), osEventFlagsWait() (+21 more)

### Community 32 - "Community 32"
Cohesion: 0.12
Nodes (39): DCB_GetAuthCtrl(), DCB_SetAuthCtrl(), DIB_GetAuthStatus(), IRQn_Type, __STATIC_INLINE, __NVIC_ClearPendingIRQ(), NVIC_ClearTargetState(), NVIC_DecodePriority() (+31 more)

### Community 33 - "Community 33"
Cohesion: 0.11
Nodes (36): __STATIC_INLINE, LL_AHB1_GRP1_DisableClock(), LL_AHB1_GRP1_DisableClockLowPower(), LL_AHB1_GRP1_EnableClock(), LL_AHB1_GRP1_EnableClockLowPower(), LL_AHB1_GRP1_ForceReset(), LL_AHB1_GRP1_IsEnabledClock(), LL_AHB1_GRP1_ReleaseReset() (+28 more)

### Community 34 - "Community 34"
Cohesion: 0.08
Nodes (17): SysTick_Handler(), HAL_StatusTypeDef, __weak, HAL_NVIC_SetPriorityGrouping(), HAL_SYSTICK_Config(), HAL_DeInit(), HAL_GetTickFreq(), HAL_IncTick() (+9 more)

### Community 35 - "Community 35"
Cohesion: 0.12
Nodes (27): HAL_GetTick(), FlagStatus, HAL_I2C_IsDeviceReady(), HAL_I2C_Master_Receive(), HAL_I2C_Master_Transmit(), HAL_I2C_Mem_Read(), HAL_I2C_Mem_Write(), HAL_I2C_Slave_Receive() (+19 more)

### Community 36 - "Community 36"
Cohesion: 0.15
Nodes (36): __STATIC_FORCEINLINE, SCB_CleanDCache(), SCB_CleanDCache_by_Addr(), SCB_CleanInvalidateDCache(), SCB_CleanInvalidateDCache_by_Addr(), SCB_DisableDCache(), SCB_DisableICache(), SCB_EnableDCache() (+28 more)

### Community 37 - "Community 37"
Cohesion: 0.15
Nodes (34): BaseType_t, TaskHandle_t, TickType_t, TimerHandle_t, UBaseType_t, pcTimerGetName(), prvCheckForValidListAndQueue(), prvGetNextExpireTime() (+26 more)

### Community 38 - "Community 38"
Cohesion: 0.15
Nodes (32): BaseType_t, TickType_t, UBaseType_t, prvBytesInBuffer(), prvInitialiseNewStreamBuffer(), prvReadBytesFromBuffer(), prvReadMessageFromBuffer(), prvWriteBytesToBuffer() (+24 more)

### Community 39 - "Community 39"
Cohesion: 0.11
Nodes (17): UART_HandleTypeDef, VL53L0X, HAL_UART_ErrorCallback(), HAL_UART_RxCpltCallback(), sensor_put_front_early(), sensor_record_front(), StartTask02(), StartTask03() (+9 more)

### Community 40 - "Community 40"
Cohesion: 0.18
Nodes (36): CenteringContext, brake_enter(), brake_run(), center_sanitize_cm(), Centering_Reset(), clampf(), corner_enter(), corner_run() (+28 more)

### Community 41 - "Community 41"
Cohesion: 0.14
Nodes (27): __STATIC_INLINE, LL_CPUID_GetConstant(), LL_CPUID_GetImplementer(), LL_CPUID_GetParNo(), LL_CPUID_GetRevision(), LL_CPUID_GetVariant(), LL_HANDLER_DisableFault(), LL_HANDLER_EnableFault() (+19 more)

### Community 42 - "Community 42"
Cohesion: 0.21
Nodes (21): DMA_HandleTypeDef, HAL_StatusTypeDef, DMA_CalcBaseAndBitshift(), DMA_CheckFifoParam(), DMA_SetConfig(), HAL_DMA_Abort(), HAL_DMA_DeInit(), HAL_DMA_GetError() (+13 more)

### Community 43 - "Community 43"
Cohesion: 0.19
Nodes (23): IRQn_Type, __STATIC_INLINE, ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ() (+15 more)

### Community 44 - "Community 44"
Cohesion: 0.06
Nodes (62): SystemInit(), SystemInit_ExtMemCtl(), IRQn_Type, __STATIC_INLINE, ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ() (+54 more)

### Community 45 - "Community 45"
Cohesion: 0.07
Nodes (36): MX_GPIO_Init(), EXTI0_IRQHandler(), TIM_HandleTypeDef, HAL_TIM_Base_MspDeInit(), HAL_TIM_Base_MspInit(), UART_HandleTypeDef, HAL_UART_MspDeInit(), HAL_UART_MspInit() (+28 more)

### Community 46 - "Community 46"
Cohesion: 0.10
Nodes (17): eSleepModeStatus, SysTick_Handler(), BaseType_t, StackType_t, TaskFunction_t, TickType_t, prvPortStartFirstTask(), prvTaskExitError() (+9 more)

### Community 47 - "Community 47"
Cohesion: 0.19
Nodes (23): IRQn_Type, __STATIC_INLINE, ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ() (+15 more)

### Community 48 - "Community 48"
Cohesion: 0.19
Nodes (23): IRQn_Type, __STATIC_INLINE, ITM_CheckChar(), ITM_ReceiveChar(), ITM_SendChar(), __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ() (+15 more)

### Community 49 - "Community 49"
Cohesion: 0.33
Nodes (19): Init_ToF_Sensors(), VL53L0X, get_spad_info(), rd16(), rd8(), rd_n(), ref_calibration(), tmo_decode() (+11 more)

### Community 50 - "Community 50"
Cohesion: 0.21
Nodes (25): GPIO_TypeDef, __STATIC_INLINE, LL_GPIO_GetAFPin_0_7(), LL_GPIO_GetAFPin_8_15(), LL_GPIO_GetPinMode(), LL_GPIO_GetPinOutputType(), LL_GPIO_GetPinPull(), LL_GPIO_GetPinSpeed() (+17 more)

### Community 51 - "Community 51"
Cohesion: 0.09
Nodes (7): clock_t, _exit(), _kill(), starm_getc(), starm_putc(), _times(), FILE

### Community 52 - "Community 52"
Cohesion: 0.25
Nodes (8): Car_Stop(), BusFault_Handler(), HardFault_Handler(), MemManage_Handler(), NMI_Handler(), TIM3_IRQHandler(), UsageFault_Handler(), USART1_IRQHandler()

### Community 53 - "Community 53"
Cohesion: 0.11
Nodes (28): BlockLink_t, HeapStats_t, osKernelGetState(), osKernelLock(), osKernelRestoreLock(), osKernelUnlock(), osMemoryPoolNew(), osThreadEnumerate() (+20 more)

### Community 54 - "Community 54"
Cohesion: 0.14
Nodes (22): __get_APSR(), __get_BASEPRI(), __get_CONTROL(), __get_FAULTMASK(), __get_FPSCR(), __get_IPSR(), __get_MSP(), __get_PRIMASK() (+14 more)

### Community 55 - "Community 55"
Cohesion: 0.13
Nodes (23): DMA_HandleTypeDef, __weak, HAL_TIM_Base_MspInit(), HAL_TIM_ErrorCallback(), HAL_TIM_IC_CaptureCallback(), HAL_TIM_IC_CaptureHalfCpltCallback(), HAL_TIM_IRQHandler(), HAL_TIM_OC_DelayElapsedCallback() (+15 more)

### Community 56 - "Community 56"
Cohesion: 0.16
Nodes (20): DMA_HandleTypeDef, __weak, HAL_UART_AbortCpltCallback(), HAL_UART_AbortReceiveCpltCallback(), HAL_UART_ErrorCallback(), HAL_UART_IRQHandler(), HAL_UART_RxCpltCallback(), HAL_UART_RxHalfCpltCallback() (+12 more)

### Community 57 - "Community 57"
Cohesion: 0.17
Nodes (21): DMA_HandleTypeDef, __weak, HAL_I2C_AbortCpltCallback(), HAL_I2C_ER_IRQHandler(), HAL_I2C_ErrorCallback(), HAL_I2C_ListenCpltCallback(), HAL_I2C_MasterRxCpltCallback(), HAL_I2C_MemRxCpltCallback() (+13 more)

### Community 58 - "Community 58"
Cohesion: 0.25
Nodes (17): IRQn_Type, __STATIC_INLINE, __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority(), __NVIC_GetEnableIRQ() (+9 more)

### Community 59 - "Community 59"
Cohesion: 0.23
Nodes (15): ARM_PMU_CNTR_Disable(), ARM_PMU_CNTR_Enable(), ARM_PMU_CNTR_Increment(), ARM_PMU_CYCCNT_Reset(), ARM_PMU_Disable(), ARM_PMU_Enable(), ARM_PMU_EVCNTR_ALL_Reset(), ARM_PMU_Get_CCNTR() (+7 more)

### Community 60 - "Community 60"
Cohesion: 0.13
Nodes (26): MX_I2C1_Init(), MX_IWDG_Init(), Error_Handler(), main(), SystemClock_Config(), Motor_Init(), HAL_TIM_MspPostInit(), MX_TIM11_Init() (+18 more)

### Community 61 - "Community 61"
Cohesion: 0.16
Nodes (16): HAL_StatusTypeDef, RCC_OscInitTypeDef, __weak, HAL_RCC_ClockConfig(), HAL_RCC_CSSCallback(), HAL_RCC_GetClockConfig(), HAL_RCC_GetHCLKFreq(), HAL_RCC_GetOscConfig() (+8 more)

### Community 62 - "Community 62"
Cohesion: 0.38
Nodes (6): test_delay(), HAL_StatusTypeDef, HAL_IWDG_Init(), HAL_IWDG_Refresh(), IWDG_HandleTypeDef, osDelay()

### Community 63 - "Community 63"
Cohesion: 0.23
Nodes (21): __DMB(), ARM_MPU_ClrRegion(), ARM_MPU_ClrRegion_NS(), ARM_MPU_ClrRegionEx(), ARM_MPU_Disable(), ARM_MPU_Disable_NS(), ARM_MPU_Enable(), ARM_MPU_Enable_NS() (+13 more)

### Community 64 - "Community 64"
Cohesion: 0.05
Nodes (18): HAL_StatusTypeDef, HAL_EXTI_ClearConfigLine(), HAL_EXTI_ClearPending(), HAL_EXTI_GenerateSWI(), HAL_EXTI_GetConfigLine(), HAL_EXTI_GetHandle(), HAL_EXTI_GetPending(), HAL_EXTI_IRQHandler() (+10 more)

### Community 65 - "Community 65"
Cohesion: 0.09
Nodes (16): crCOROUTINE_CODE, BaseType_t, List_t, TickType_t, UBaseType_t, prvCheckDelayedList(), prvCheckPendingReadyList(), prvInitialiseCoRoutineLists() (+8 more)

### Community 66 - "Community 66"
Cohesion: 0.46
Nodes (13): StartDefaultTask(), Car_ArcLeft(), Car_ArcRight(), Car_Backward(), Car_Brake(), Car_Forward(), Car_PivotLeft(), Car_PivotRight() (+5 more)

### Community 67 - "Community 67"
Cohesion: 0.17
Nodes (13): HAL_StatusTypeDef, RCC_OscInitTypeDef, HAL_RCC_GetOscConfig(), HAL_RCC_OscConfig(), HAL_RCCEx_DisablePLLI2S(), HAL_RCCEx_DisablePLLSAI(), HAL_RCCEx_EnablePLLI2S(), HAL_RCCEx_EnablePLLSAI() (+5 more)

### Community 68 - "Community 68"
Cohesion: 0.25
Nodes (17): IRQn_Type, __STATIC_INLINE, __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority(), __NVIC_GetEnableIRQ() (+9 more)

### Community 69 - "Community 69"
Cohesion: 0.13
Nodes (28): TIM_TypeDef, HAL_TIM_ConfigClockSource(), HAL_TIM_ConfigOCrefClear(), HAL_TIM_IC_ConfigChannel(), HAL_TIM_OC_ConfigChannel(), HAL_TIM_OnePulse_ConfigChannel(), HAL_TIM_PWM_ConfigChannel(), HAL_TIM_SlaveConfigSynchro() (+20 more)

### Community 70 - "Community 70"
Cohesion: 0.18
Nodes (9): void(), HAL_StatusTypeDef, I2C_HandleTypeDef, HAL_I2CEx_ConfigAnalogFilter(), HAL_I2CEx_ConfigDigitalFilter(), HAL_FLASHEx_DisableFlashSleepMode, HAL_FLASHEx_EnableFlashSleepMode, HAL_FLASHEx_StartFlashInterfaceClk (+1 more)

### Community 71 - "Community 71"
Cohesion: 0.57
Nodes (7): HAL_HalfDuplex_Init(), HAL_LIN_Init(), HAL_MultiProcessor_Init(), HAL_UART_Init(), HAL_UART_MspInit(), UART_InitCallbacksToDefault(), UART_SetConfig()

### Community 72 - "Community 72"
Cohesion: 0.67
Nodes (4): HAL_UART_DMAStop(), UART_DMAError(), UART_EndRxTransfer(), UART_EndTxTransfer()

### Community 73 - "Community 73"
Cohesion: 0.25
Nodes (17): IRQn_Type, __STATIC_INLINE, __NVIC_ClearPendingIRQ(), NVIC_DecodePriority(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), NVIC_EncodePriority(), __NVIC_GetEnableIRQ() (+9 more)

### Community 74 - "Community 74"
Cohesion: 0.28
Nodes (12): Atomic_Add_u32(), Atomic_AND_u32(), Atomic_CompareAndSwap_u32(), Atomic_CompareAndSwapPointers_p32(), Atomic_Decrement_u32(), Atomic_Increment_u32(), Atomic_NAND_u32(), Atomic_OR_u32() (+4 more)

### Community 75 - "Community 75"
Cohesion: 0.50
Nodes (7): DMA_HandleTypeDef, HAL_StatusTypeDef, DMA_MultiBufferSetConfig(), HAL_DMAEx_ChangeMemory(), HAL_DMAEx_MultiBufferStart(), HAL_DMAEx_MultiBufferStart_IT(), HAL_DMA_MemoryTypeDef

### Community 76 - "Community 76"
Cohesion: 0.23
Nodes (12): HAL_I2C_EV_IRQHandler(), HAL_I2C_MasterTxCpltCallback(), HAL_I2C_MemTxCpltCallback(), I2C_ConvertOtherXferOptions(), I2C_Master_ADD10(), I2C_Master_ADDR(), I2C_Master_SB(), I2C_MasterTransmit_BTF() (+4 more)

### Community 77 - "Community 77"
Cohesion: 0.26
Nodes (15): IRQn_Type, __STATIC_INLINE, __NVIC_ClearPendingIRQ(), __NVIC_DisableIRQ(), __NVIC_EnableIRQ(), __NVIC_GetEnableIRQ(), __NVIC_GetPendingIRQ(), __NVIC_GetPriority() (+7 more)

### Community 78 - "Community 78"
Cohesion: 0.36
Nodes (9): ARM_MPU_ClrRegion(), ARM_MPU_Disable(), ARM_MPU_Enable(), ARM_MPU_Load(), ARM_MPU_OrderedMemcpy(), ARM_MPU_SetRegion(), ARM_MPU_SetRegionEx(), ARM_MPU_Region_t (+1 more)

### Community 79 - "Community 79"
Cohesion: 0.29
Nodes (5): portFORCE_INLINE, ulPortRaiseBASEPRI(), vPortRaiseBASEPRI(), vPortSetBASEPRI(), xPortIsInsideInterrupt()

### Community 80 - "Community 80"
Cohesion: 0.12
Nodes (16): 0. 새 세션에서 먼저 볼 것, 10. 디버깅 포인트, 11. 최근 주행 이력 요약, 1. 프로젝트 목표, 2. 하드웨어와 센서, 3. 런타임 구조, 4. 코스 정보, 5. IMG_2985 관찰과 수정 결과 (+8 more)

### Community 81 - "Community 81"
Cohesion: 0.14
Nodes (11): BNO055_Euler, BNO055_HardReset(), BNO055_Init(), BNO055_ReadCalibStatus(), BNO055_ReadEuler(), bno_bus_recover(), bno_rd(), bno_wr8() (+3 more)

### Community 84 - "Community 84"
Cohesion: 0.40
Nodes (4): Auto-Clarity, Boundaries, Intensity, Rules

### Community 85 - "Community 85"
Cohesion: 0.40
Nodes (5): __PACKED_STRUCT, T_UINT16_READ(), T_UINT16_WRITE(), T_UINT32_READ(), T_UINT32_WRITE()

### Community 86 - "Community 86"
Cohesion: 0.40
Nodes (5): __PACKED_STRUCT, T_UINT16_READ(), T_UINT16_WRITE(), T_UINT32_READ(), T_UINT32_WRITE()

### Community 87 - "Community 87"
Cohesion: 0.40
Nodes (5): __ROR(), __SXTAB16(), __SXTAB16_RORn(), __SXTB16(), __SXTB16_RORn()

### Community 94 - "Community 94"
Cohesion: 0.36
Nodes (7): packed, __PACKED_STRUCT, T_UINT16_READ(), T_UINT16_WRITE(), T_UINT32(), T_UINT32_READ(), T_UINT32_WRITE()

## Knowledge Gaps
- **23 isolated node(s):** `v`, `v`, `v`, `Rules`, `Intensity` (+18 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **4 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `__ISB()` connect `Community 36` to `Community 2`, `Community 3`, `Community 11`, `Community 13`, `Community 15`, `Community 21`, `Community 22`, `Community 23`, `Community 24`, `Community 25`, `Community 26`, `Community 30`, `Community 32`, `Community 41`, `Community 43`, `Community 44`, `Community 47`, `Community 48`, `Community 54`, `Community 58`, `Community 63`, `Community 68`, `Community 73`, `Community 77`, `Community 78`?**
  _High betweenness centrality (0.079) - this node is a cross-community bridge._
- **Why does `HAL_MPU_Enable()` connect `Community 36` to `Community 45`?**
  _High betweenness centrality (0.055) - this node is a cross-community bridge._
- **Why does `__DSB()` connect `Community 36` to `Community 2`, `Community 13`, `Community 15`, `Community 22`, `Community 23`, `Community 24`, `Community 25`, `Community 26`, `Community 30`, `Community 32`, `Community 41`, `Community 43`, `Community 44`, `Community 47`, `Community 48`, `Community 58`, `Community 63`, `Community 68`, `Community 73`, `Community 77`, `Community 78`?**
  _High betweenness centrality (0.043) - this node is a cross-community bridge._
- **What connects `v`, `v`, `v` to the rest of the system?**
  _23 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Community 0` be split into smaller, more focused modules?**
  _Cohesion score 0.032780847145488026 - nodes in this community are weakly interconnected._
- **Should `Community 1` be split into smaller, more focused modules?**
  _Cohesion score 0.043151567196221555 - nodes in this community are weakly interconnected._
- **Should `Community 2` be split into smaller, more focused modules?**
  _Cohesion score 0.02984734563681932 - nodes in this community are weakly interconnected._