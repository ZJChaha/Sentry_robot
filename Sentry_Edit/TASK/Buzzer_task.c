
#include "Buzzer_task.h"
#include "tim.h"
#include "cmsis_os.h"




uint8_t buzzer_flag;


void Buzzer_task(void const *pvParameters)
{
	
    vTaskDelay(200);
	
	while(1)

	{
		switch (buzzer_flag)
		{
			case 0:
				buzzer_off();
				break;
			case 1:
				Funky_town();
				break;
			case 2:
				trace();//一直叫
				break;
			case 3:
				warning();
				break;
			case 4:
				Windows_XP();
			    break;
			case 5:
				di();
			    break;
            case 6:
                DJI();
                break;


		}
        vTaskDelay(10);
        
	}

}



void buzzer_on(uint16_t arr)
{
  
   
    __HAL_TIM_SetAutoreload(&htim4,arr);
	 __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, arr/2);

}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
int caculate_arr(int set)
{
    int out;
	out=21000000/set;
return out;
}	

void di()
{

   buzzer_on(caculate_arr(988));
   vTaskDelay(500);
   buzzer_flag=0;
}

	
	


void Windows_XP(void)
{

	buzzer_on(caculate_arr(659));
	vTaskDelay(375);
 
	buzzer_on(caculate_arr(880));  
	vTaskDelay(450);
  
	buzzer_on(caculate_arr(988));  
	vTaskDelay(500);
 
	buzzer_on(caculate_arr(1115));  
	vTaskDelay(650);	  

     buzzer_off();
	vTaskDelay(230);
	
	buzzer_on(caculate_arr(659));  
	vTaskDelay(400); 
	  
	buzzer_on(caculate_arr(988));  
	vTaskDelay(400); 
	
	buzzer_on(caculate_arr(1109));  
	vTaskDelay(430); 
	  
	buzzer_on(caculate_arr(1318));  
	vTaskDelay(450);	  
	  
	buzzer_on(caculate_arr(1480));  
	vTaskDelay(680); 
	  
    buzzer_off();
	vTaskDelay(10);
	
	buzzer_on(caculate_arr(988));   
	vTaskDelay(300);	 
	
    buzzer_off();
	vTaskDelay(10);
	
	buzzer_on(caculate_arr(831));  
	vTaskDelay(300);	
	
    buzzer_off();

    buzzer_flag=0;
}


void part1(void)
{if(buzzer_flag==1)
	{buzzer_on(caculate_arr(1046));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(30); 	  
	buzzer_on(caculate_arr(1046)); 
	vTaskDelay(250);  
	buzzer_on(caculate_arr(932));
	vTaskDelay(250);
	buzzer_on(caculate_arr(1046));
	vTaskDelay(350); 
    buzzer_off();
	vTaskDelay(70);      	  
	buzzer_on(caculate_arr(784));
	vTaskDelay(320);
    buzzer_off();
	vTaskDelay(200);  
	buzzer_on(caculate_arr(784));
	vTaskDelay(250);
	buzzer_on(caculate_arr(1046));
	vTaskDelay(250);	  
	buzzer_on(caculate_arr(1397));
	vTaskDelay(250);	  
	buzzer_on(caculate_arr(1318));
	vTaskDelay(250);		  
	buzzer_on(caculate_arr(1046));
	vTaskDelay(250);		
    buzzer_off();
	vTaskDelay(1000); 
}
	else
		buzzer_off();

}
void part2(void)
{if(buzzer_flag==1)
	{buzzer_on(caculate_arr(1046));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);
	buzzer_on(caculate_arr(1046));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);
	buzzer_on(caculate_arr(1046));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);
	buzzer_on(caculate_arr(1318));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);
	buzzer_on(caculate_arr(1318));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);
	buzzer_on(caculate_arr(1318));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);
	buzzer_on(caculate_arr(1568));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);
	buzzer_on(caculate_arr(1568));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);
	buzzer_on(caculate_arr(1568));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);
	buzzer_on(caculate_arr(1318));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);	
	buzzer_on(caculate_arr(1318));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);	
	buzzer_on(caculate_arr(1175));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);	
	buzzer_on(caculate_arr(1046));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);	
	buzzer_off();
	vTaskDelay(1000);
}
	else
		buzzer_off();
}
void part3(void)
{
if(buzzer_flag==1)
{buzzer_on(caculate_arr(1175));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);
	buzzer_on(caculate_arr(1175));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);	
	buzzer_on(caculate_arr(1175));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);	
	buzzer_on(caculate_arr(1175));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);		
	buzzer_on(caculate_arr(1046));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);	
		buzzer_on(caculate_arr(1046));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);
		buzzer_on(caculate_arr(1046));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);
		buzzer_on(caculate_arr(1046));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);
	buzzer_on(caculate_arr(988));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);
	buzzer_on(caculate_arr(988));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);	
	buzzer_on(caculate_arr(988));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);	
	buzzer_on(caculate_arr(988));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);		
	buzzer_on(caculate_arr(880));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);
	buzzer_on(caculate_arr(880));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);
	buzzer_on(caculate_arr(880));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);
	buzzer_on(caculate_arr(784));
	vTaskDelay(250);
    buzzer_off();
	vTaskDelay(10);
}
else 
	buzzer_off();

}

void Funky_town(void)
{	
part1();
part1();
part2();
part1();
part2();	
part1();	
part3();
vTaskDelay(1000);
}

void trace()//一直叫
{

  buzzer_on(caculate_arr(988));
//  vTaskDelay(10);
//  buzzer_off();
//  vTaskDelay(600);

}

void warning()
{
buzzer_on(caculate_arr(1109));
vTaskDelay(150);
 buzzer_off();
vTaskDelay(30);	
buzzer_on(caculate_arr(1109));
vTaskDelay(150);
 buzzer_off();
vTaskDelay(30);
buzzer_on(caculate_arr(1109));
vTaskDelay(150);
 buzzer_off();
vTaskDelay(30);	
buzzer_flag=0;
}

void DJI(void)
{
    buzzer_on(caculate_arr(523));
    vTaskDelay(240);
    buzzer_off();
    vTaskDelay(3);
    buzzer_on(caculate_arr(587));
    vTaskDelay(250);
    buzzer_off();
    vTaskDelay(3);
    buzzer_on(caculate_arr(790));
    vTaskDelay(400);
    buzzer_off();
    buzzer_flag=0;
}
