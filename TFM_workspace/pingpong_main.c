void pingpong(void)
{
	rStatus = SX1276GetStatus();
	if ( isMaster )
	   	  {
	   		  BufferTx[0] = 'P';  //  P
	   		  BufferTx[1] = 'I';  //  O
	   		  BufferTx[2] = 'N';				// N
	   		  BufferTx[3] = 'G';				// G
	   		  if ( rStatus == RF_IDLE )
	   		  {
	   			if (rxDoneFlag)
	   			{
	   				rxDoneFlag = 0;
	   				if ( strncmp( ( const char* )Buffer, ( const char* )PongMsg, 4 ) == 0 )
	   				{
	   					HAL_GPIO_WritePin(GPIOD, LD4_Pin, SET);
	   					HAL_Delay(100);
	   					HAL_GPIO_WritePin(GPIOD, LD4_Pin, RESET);
	   				}
	   				HAL_Delay(100);
	   				Radio.Send( BufferTx, BufferSize );
	   			}
	   			else if (txDoneFlag)
	   			{
	   				txDoneFlag = 0;
	   				nt = 0;
	   				HAL_Delay(100);
	   				SX1276SetRx( RX_TIMEOUT_VALUE );
	   			}
	   			else	// ni recibo ni envio -> inicio del programa
	   			{
	   				Radio.Send( BufferTx, BufferSize );
	   			}
	   		  }
	   		  else if ( rStatus == RF_TX_RUNNING )
	   		  {
	   			  HAL_GPIO_WritePin(GPIOD, LD5_Pin, SET);
	   			  HAL_Delay(100);
	   			  HAL_GPIO_WritePin(GPIOD, LD5_Pin, RESET);
	   			  nt++;
	   			  if ( nt > MAX_TX )
	   			  {
	   				  nt = 0;
	   				  txDoneFlag=1;
	   				  SX1276SetStby();

	   			  }
	   		  }
	   		  else if ( rStatus == RF_RX_RUNNING )
	   		  {
	   			  HAL_GPIO_WritePin(GPIOD, LD3_Pin, SET);
	   			  HAL_Delay(3000);
	   			  SX1276SetStby();
	   			  HAL_GPIO_WritePin(GPIOD, LD3_Pin, RESET);
	   		  }
	   	  }
	   	  /* Codigo para NO master = PONG */ //######### SALVE ###########
	   	  else
	   	  {
	   		  BufferTx[0] = 'P';
	   		  BufferTx[1] = 'O';
	   		  BufferTx[2] = 'N';
	   		  BufferTx[3] = 'G';
	   		  if ( rStatus == RF_IDLE )
	   		  {
	   			if (rxDoneFlag)
	   			{
	   				rxDoneFlag = 0;
	   				if ( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
	   				{
	   					HAL_GPIO_WritePin(GPIOD, LD6_Pin, SET);
	   					HAL_Delay(100);
	   					HAL_GPIO_WritePin(GPIOD, LD6_Pin, RESET);
	   				}
	   				HAL_Delay(100);
	   				Radio.Send( BufferTx, BufferSize );
	   			}
	   			else if (txDoneFlag)
	   			{
	   				txDoneFlag = 0;
	   				nt = 0;
	   				HAL_Delay(100);
	   				SX1276SetRx( RX_TIMEOUT_VALUE );
	   			}
	   			else	// ni recibo ni envio -> inicio del programa
	   			{
	   				Radio.Send( BufferTx, BufferSize );
	   			}
	   		  }
	   		  else if ( rStatus == RF_TX_RUNNING )
	   		  {
	   			 HAL_GPIO_WritePin(GPIOD, LD5_Pin, SET);
	   			 HAL_Delay(100);
	   			 HAL_GPIO_WritePin(GPIOD, LD5_Pin, RESET);
	   			  nt++;
	   			  if ( nt > MAX_TX )
	   			  {
	   				  nt = 0;
	   				  txDoneFlag=1;
	   				  SX1276SetStby();
	   			  }
	   		  }
	   		  else if ( rStatus == RF_RX_RUNNING )
	   		  {
	   			HAL_GPIO_WritePin(GPIOD, LD3_Pin, SET);
	   			HAL_Delay(3000);
	   			SX1276SetStby();
	   			HAL_GPIO_WritePin(GPIOD, LD3_Pin, RESET);
	   		  }
	   	  }
}