string STATUS_WAITING=WAITING
string STATUS_IN_PROGRESS=IN_PROGRESS
string STATUS_DONE=DONE
string STATUS_FAILED=FAILED
string STATUS_NEEDS_HUMAN=NEEDS_HUMAN
string STATUS_CANCELLED=CANCELLED

string id
uint8 reason            # RequestCollection.Request.FALLEN / OUT_OF_ZONE / LOW_BATTERY
float32 wait_sec
float32 score
string status
float64 x               # map 좌표 (GPS 변환값, 카메라 보정 후 갱신)
float64 y
uint8 nav_failures
