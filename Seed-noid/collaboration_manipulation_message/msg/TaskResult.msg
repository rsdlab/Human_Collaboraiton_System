#作業結果
uint16 task_command_id   #作業指令ID：UnlimitedNatural
uint16 picked_number     #ピッキング番号：UnlimitedNatural
bool task_result         #作業結果：Boolean
float32 cycle_time       #サイクルタイム：Real[0..1]
ErrorInfomation[] errors #エラー情報：エラー情報[0..*]
