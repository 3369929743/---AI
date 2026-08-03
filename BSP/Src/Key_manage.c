#include "Key_manage.h"
#include "Key.h"
#include "Task_Ball_Contral.h"

static Key_t Key_User, Key1, Key2, Key3, Key4;
extern uint8_t i;

void Key_Global_Callback(Key_t *Key, Key_Event_e Event){
  if(Key == &Key_User){
    switch(Event){
      case KEY_EVENT_ONCE_PRESS:
        Task_Ball_Contral_Toggle();
        i++;
        break;
      case KEY_EVENT_DOUBLE_PRESS:
        Task_Ball_Contral_Pop_Restore();
        break;
      case KEY_EVENT_LONG_PRESS:
        Task_Ball_Start_5cm_Sequence();
        break;
      case KEY_EVENT_LONG_PRESS_REPEAT:
        break;
      default:
        break;
    }
  }
  if(Key == &Key1){
    switch (Event) {
      case KEY_EVENT_ONCE_PRESS:
        Task_Ball_Contral_Toggle();
        break;
      case KEY_EVENT_DOUBLE_PRESS:
        Task_Ball_Reset_Zero();
        break;
      case KEY_EVENT_LONG_PRESS:
        break;
      case KEY_EVENT_LONG_PRESS_REPEAT:
        break;
      default:
        break;
    }
  }
  if(Key == &Key2){
    switch (Event) {
      case KEY_EVENT_ONCE_PRESS:
        break;
      case KEY_EVENT_DOUBLE_PRESS:
        break;
      case KEY_EVENT_LONG_PRESS:
        break;
      case KEY_EVENT_LONG_PRESS_REPEAT:
        break;
      default:
        break;
    }
  }
  if(Key == &Key3){
    switch (Event) {
      case KEY_EVENT_ONCE_PRESS:
        break;
      case KEY_EVENT_DOUBLE_PRESS:
        break;
      case KEY_EVENT_LONG_PRESS:
        break;
      case KEY_EVENT_LONG_PRESS_REPEAT:
        break;
      default:
        break;
    }
  }
  if(Key == &Key4){
    switch (Event) {
      case KEY_EVENT_ONCE_PRESS:
        break;
      case KEY_EVENT_DOUBLE_PRESS:
        break;
      case KEY_EVENT_LONG_PRESS:
        break;
      case KEY_EVENT_LONG_PRESS_REPEAT:
        break;
      default:
        break;
    }
  }
}

void Key_Glabal_Init(void){
    Key_Init(&Key_User, KEY_LEVEL_HIGH);
    Key_Init(&Key1, KEY_LEVEL_LOW);
    Key_Init(&Key2, KEY_LEVEL_LOW);
    Key_Init(&Key3, KEY_LEVEL_LOW);
    Key_Init(&Key4, KEY_LEVEL_LOW);
    Key_SetEventCallback(&Key_User, Key_Global_Callback);
    Key_SetEventCallback(&Key1, Key_Global_Callback);
    Key_SetEventCallback(&Key2, Key_Global_Callback);
    Key_SetEventCallback(&Key3, Key_Global_Callback);
    Key_SetEventCallback(&Key4, Key_Global_Callback);
}

void Key_Global_Tick(void){
    Key_Tick(&Key_User, (uint8_t)HAL_GPIO_ReadPin(User_Key_GPIO_Port, User_Key_Pin));
    Key_Tick(&Key1, (uint8_t)HAL_GPIO_ReadPin(Key1_GPIO_Port, Key1_Pin));
    Key_Tick(&Key2, (uint8_t)HAL_GPIO_ReadPin(Key2_GPIO_Port, Key2_Pin));
    Key_Tick(&Key3, (uint8_t)HAL_GPIO_ReadPin(Key3_GPIO_Port, Key3_Pin));
    Key_Tick(&Key4, (uint8_t)HAL_GPIO_ReadPin(Key4_GPIO_Port, Key4_Pin));
}

void Key_Global_Trigger_Event(void){
    Key_Trigger_Event(&Key_User);
    Key_Trigger_Event(&Key1);
    Key_Trigger_Event(&Key2);
    Key_Trigger_Event(&Key3);
    Key_Trigger_Event(&Key4);
}
