#ワーク認識結果
uint16 work_type_id           #ワーク種類ID：UnlimitedNatural
string timestamp              #計測時刻：String[0..1]
geometry_msgs/Pose pose       #位置・姿勢：センサ座標系位置・姿勢
float32 reliability           #信頼度：Real [0..1]
geometry_msgs/Pose grasp_pose #把持位置・姿勢：ワーク座標系位置・姿勢 [0..1]
float32 graspability          #把持容易性：Real [0..1]
