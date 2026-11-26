# SCOOTOVER 로봇 충전소 (컨테이너 차고)

참고 사진(Patrover 경찰 순찰로봇 차고)의 구성을 따라 SCOOTOVER 로봇 크기에 맞게 만들었습니다.

## 파일 (`robot_station/`)
- `scootover_robot_station.blend`: 충전소 모델
- `screenshots/`: `robot_station_front.png`(셔터 닫힘), `robot_station_open.png`(셔터 열림, 로봇 주차), `robot_station_side.png`
- `scripts/build_robot_station.py`: 생성 스크립트. 혼자 실행되고, 로봇 미리보기에 `v19/scootover_v19.blend`를 씁니다.
  - 실행: `Blender -b --factory-startup --python robot_station/scripts/build_robot_station.py`

## 치수 (문 쪽이 −X, 로봇은 +X로 들어감)
- 바깥: 길이 2.6 × 폭 1.72 × 높이 2.0 m. 안쪽 바닥 높이 0.10 m, 앞에 0.45 m 체커판 경사
- 출입구: 폭 1.40, 셔터를 올렸을 때 실제 통과 높이 1.42 m
- 로봇(실린 킥보드 핸들 포함)이 차지하는 범위: 폭 ±0.55, 높이 1.45 m
  - 문 옆 여유 각 11.5 cm, 위 여유 6.7 cm

## 구성 (컬렉션)
- `Station`
  - `station_frame`: 민트 프레임(넓은 앞 테두리)
  - `station_shell`: 회색 골판 벽·뒷벽, 흰 지붕·천장, 실내 조명 띠, 짙은 바닥과 바퀴 안내 민트 선
  - `station_front`: 간판판(아래 민트 조명선), 오른쪽 위 제어함과 배관, 민트 경광등, 앞 경사
  - `station_sign_text`: SCOOTOVER 글자(짙은 민트)
  - `station_fittings`: 오른쪽 옆문과 손잡이, 전기함, 뒷벽 충전기(접점판, 표시등), 환기구, AR 마커판
- `Shutter`
  - `shutter_housing`: 셔터 말림통
  - `shutter_curtain`: 셔터. 셰이프 키 `open`이 0이면 닫힘, 1이면 말려 올라감
- `Robot_preview`: v19 로봇(수송 자세, 킥보드 2대)을 안에 주차해 둔 모습. 빈 오브젝트 `robot_preview`를 옮기거나 컬렉션을 숨기면 됩니다.
- `Stage`: 바닥, 카메라 3대, 조명

## 가정
- 충전은 로봇 앞면이 뒷벽 충전기 접점판(높이 0.30~0.36 m)에 닿는 방식으로 표현했습니다. 로봇 쪽 접점은 아직 모델에 없습니다.
- 경광등은 빨강 대신 민트로 했습니다(지정 색 조합).
