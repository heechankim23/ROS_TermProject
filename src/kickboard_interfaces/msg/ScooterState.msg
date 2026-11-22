# 킥보드 펌웨어가 주기적으로 내보내는 상태 (scooter_fleet_sim -> queue_manager, RViz)
string id
float64 lat
float64 lon
float32 tilt_deg
float32 battery_pct
bool in_parking_zone       # 캠퍼스 맵: 슬롯에 바르게 서 있음 (거치대 센서)
string fault               # 자가진단 고장 코드 (빈 문자열 = 정상): brake motor throttle lock display tire stem
bool in_slot_area          # 슬롯은 아니지만 거치대 가까이(6 m) 있음 -> 정렬 불량
builtin_interfaces/Time stamp
