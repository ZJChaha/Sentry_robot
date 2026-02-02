#include "bsp_can.h"
#include "main.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);



}
void HAL_CAN_FilterInit(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef HAL_CAN_FilterInitStructure;

    HAL_CAN_FilterInitStructure.FilterMode=CAN_FILTERMODE_IDMASK;
    HAL_CAN_FilterInitStructure.FilterScale=CAN_FILTERSCALE_32BIT;
    HAL_CAN_FilterInitStructure.FilterIdHigh=0x0000;
    HAL_CAN_FilterInitStructure.FilterIdLow=0x0000;
    HAL_CAN_FilterInitStructure.FilterMaskIdHigh=0x0000;
    HAL_CAN_FilterInitStructure.FilterMaskIdLow=0x0000;
    HAL_CAN_FilterInitStructure.FilterFIFOAssignment=CAN_FilterFIFO0;
    HAL_CAN_FilterInitStructure.FilterActivation=ENABLE;
    HAL_CAN_FilterInitStructure.SlaveStartFilterBank=14;

    if(hcan == &hcan1)
    {
        HAL_CAN_FilterInitStructure.FilterBank = 0;
    }
    if(hcan == &hcan2)
    {
        HAL_CAN_FilterInitStructure.FilterBank = 14;
    }
    if(HAL_CAN_ConfigFilter(hcan,&HAL_CAN_FilterInitStructure)!=HAL_OK)
    {
        Error_Handler();
    }
    if(HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)
    {
        Error_Handler();
    }
    if(HAL_CAN_ActivateNotification(hcan,CAN_IT_TX_MAILBOX_EMPTY)!=HAL_OK)
    {
        Error_Handler();
    }
    if(HAL_CAN_Start(hcan)!=HAL_OK)
    {
        Error_Handler();
    }

}

void can_Filter_and_it_Init(void)
{
    HAL_CAN_FilterInit(&hcan1);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    HAL_CAN_FilterInit(&hcan2);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
}