#ifndef SimonSays_h
#define SimonSays_h

typedef struct{
  int *note;
  int *duration;
  int number;
}Melody;
typedef enum{
  MELODY_START,
  MELODY_CORRECT,
  MELODY_WRONG,
  MELODY_MAX,
} Melody_Enum;
typedef enum{
  STATE_START,//重新開始遊戲
  STATE_QUESTION,//閃爍LED，給問題
  STATE_ANSWER,//等待使用者按下開關，回答問題
  STATE_CORRECT,//正確，播放恭喜音效，重置
  STATE_WRONG,//錯誤，播方可惜的音效，重置
  STATE_MAX,
} State;

#endif
