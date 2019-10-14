
HAL_ADCEx_Calibration_Start(&hadc3,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);
for(i=0;i<6;i++)
                  {
                                
                                HAL_ADC_Start(&hadc1);
                                HAL_ADC_PollForConversion(&hadc1,0xffff);
                                ADBUF[i]= HAL_ADC_GetValue(&hadc1);
                                HAL_ADC_Stop(&hadc1);
                                delay_ms(10);                                
                        }        
                
                        delay_ms(500);