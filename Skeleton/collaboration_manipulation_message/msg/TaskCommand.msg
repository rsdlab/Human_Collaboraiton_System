#作業指令
uint16 task_command_id				#作業指令ID：UnlimitedNatural
uint16 work_type_id				#ワーク種類ID：UnlimitedNatural
uint16 picking_count				#ピッキング個数：UnlimitedNatural
TargetArea work_presence_area			#ワークが存在するエリア：3Dエリア
PlaceAreaCandidatesList place_area_candidates	#排出位置候補：排出位置候補エリアリスト
bool has_pause					#一時中断あり：Boolean[0..1]
float32[] standby_pose				#待機姿勢：Real[0..*] {ordered}
Property[] properties 				#追加情報：プロパティ[0..*]
