HAL_TIM_Base_Start_IT(&htim3);						// 启动定时器3




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)		// 判断是由哪个定时器触发的中断
	{
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);
	}

}
————————————————
版权声明：本文为CSDN博主「落叶_小唱」的原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/ouening/article/details/79218971