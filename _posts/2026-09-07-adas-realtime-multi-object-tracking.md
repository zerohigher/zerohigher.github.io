---
layout: post
title: "실차 ADAS를 위한 Multi-Object Tracking: 학계 1위 알고리즘이 임베디드 양산에서 탈락하는 이유"
date: 2026-09-07 12:00:00 +0900
categories: [ADAS, Autonomous Driving]
tags: [Object Tracking, ByteTrack, OC-SORT, DeepSORT, Embedded AI, 실차검증]
description: "Front Camera ADAS(FCWS/AEBS) 환경에서 실시간성, 제한된 SoC 연산량, Ego-motion을 고려해 StrongSORT, BoT-SORT, ByteTrack, OC-SORT를 분석하고 최적의 추적 파이프라인을 도출한 엔지니어링 의사결정 과정"
---

## 1. 문제 정의: 학계 벤치마크와 실차 양산 환경의 괴리

자율주행 및 ADAS 인지(Perception) 파이프라인에서 객체 탐지(Object Detection)만큼이나 중요한 모듈이 **다중 객체 추적(Multi-Object Tracking, MOT)**입니다. 전방 카메라로 들어오는 객체들에 고유 ID를 부여하고 시간 축(Temporal Domain)에서 위치와 속도를 안정적으로 추정해야만, 전방 충돌 경고(FCWS), 자동 긴급 제동(AEBS), 차로 유지 보조(LKAS) 등 하위 판단/제어 로직이 올바르게 동작합니다.

MOT17이나 MOT20 같은 공공 벤치마크 리더보드를 보면 복잡한 ReID(Re-Identification) 심층 신경망과 어피어런스 임베딩(Appearance Embedding), 장기 메모리를 결합한 대형 모델들이 상위권을 독식하고 있습니다.

하지만 **자동차 임베디드 SoC 환경**으로 넘어오면 이야기가 완전히 달라집니다.

```
[전체 파이프라인 연산 예산: 33.3ms (30 FPS)]
┌───────────────────────────────────────┬────────────┬─────────────┐
│ 1. 카메라 ISP & 전처리 (~5ms)          │ 2. NPU     │ 3. Tracking │
│                                       │ Detector   │ & Fusion    │
│                                       │ (20~23ms)  │ (≤ 5ms)     │
└───────────────────────────────────────┴────────────┴─────────────┘
```

실차 전방 카메라 시스템이 30 FPS로 동작할 때, 프레임당 허용되는 엔드투엔드 처리 시간은 **33.3ms**에 불과합니다.
* 영상 캡처 및 ISP 처리: ~5ms
* NPU 상에서의 객체 검출기(Detector): 20~23ms
* **호스트 CPU(ARM Cortex-A 계열)에 남겨진 MOT 연산 예산: 고작 5ms 미만**

여기에 실차 특유의 가혹한 조건들이 더해집니다:
1. **자차 거동(Ego-motion)**: 노면 요철, 가감속에 의한 Pitch/Yaw, 조향으로 인해 카메라 자체가 격렬하게 움직이며, 화면 내 객체의 궤적이 비선형적으로 왜곡됩니다.
2. **경량 검출기(Lightweight Detector)의 한계**: NPU 연산 제약으로 인해 경량 모델을 사용할 경우 먼 거리의 객체나 부분 가림(Occlusion) 시 Confidence 점수가 급격히 떨어지며 탐지 누락(False Negative)이 빈번하게 발생합니다.
3. **ID Switch가 AEBS에 미치는 위험성**: 전방 충돌 경고 및 비상 제동은 특정 타깃의 상대속도와 TTC(Time to Collision)가 연속적으로 유지되어야 발동합니다. 추적 중 ID가 바뀌거나 튀는 현상(Flicker)은 경고 누락이나 유령 제동(Phantom Braking)으로 직결됩니다.

이러한 제약 조건 속에서 우리는 네 가지 대표 알고리즘(**StrongSORT, BoT-SORT, ByteTrack, OC-SORT**)을 실차 양산 관점에서 철저히 비교·검증했습니다.

---

## 2. 후보 알고리즘 정밀 비교: 구조와 트레이드오프

```
[Raw Frame] ──▶ [Object Detector] ──▶ High & Low Conf Bboxes
                       │
       ┌───────────────┼───────────────┐
       ▼               ▼               ▼
 [StrongSORT]     [ByteTrack]      [OC-SORT]
  ReID CNN 실행    2-stage IoU      관측 중심 모멘텀
 (연산 과다 탈락) (1~2ms 극경량)  (비선형 거동 보정)
```

### 2.1 StrongSORT: 학계 최강자의 탈락

* **구조**: Kalman Filter + Strong ReID 임베딩 + EMA(Exponential Moving Average) 특징 업데이트 + AFLink.
* **장점**: MOT17 기준 IDF1 83.2로 외관 특징 식별력이 매우 뛰어납니다.
* **탈락 사유**: 매 프레임 탐지된 수십 개의 Bounding Box마다 ReID 특징 추출용 CNN을 추가로 돌려야 합니다. 임베디드 SoC에서 컨텍스트 스위칭 비용과 추가 NPU/CPU 점유율이 발생해 **단독 추적에만 15~25ms 이상 소요**됩니다. 실시간성(≤5ms) 제약을 치명적으로 위반하므로 전방 ADAS용으로는 채택이 불가능했습니다.

### 2.2 BoT-SORT: 우수한 성능, 그러나 아쉬운 연산 오버헤드

* **구조**: ByteTrack 매칭 전략 + ReID 모듈(선택) + GMC(Global Motion Compensation).
* **장점**: 카메라의 움직임을 Optical Flow/특징점 기반 GMC로 보상해주어 자차 주행 시 외관 움직임 왜곡을 훌륭하게 보정합니다.
* **분석 결과**: GMC는 자차 거동 보정에 매우 유용하지만, 매 프레임 호스트 CPU에서 수행되는 배경 특징점 추출 및 호모그래피(Homography) 계산이 차량용 저전력 CPU 코어에 상당한 부담을 줍니다. 실차 CAN 버스의 조향각/차속/Yaw Rate 데이터를 직접 활용할 수 있는 환경이라면 굳이 무거운 비전 기반 GMC를 돌릴 이유가 줄어듭니다.

### 2.3 ByteTrack: 단순함의 미학, 약한 검출기의 구원투수

* **구조**: ReID 신경망 완전 배제 + 2단계 IoU 연관(Association) 기법.
* **핵심 혁신**:
  기존 트래커들은 노이즈를 피하기 위해 통상 Confidence 0.5~0.6 이하의 Bounding Box를 버립니다. 하지만 실차 환경에서는 먼 거리 보행자나 앞 차량에 가려진 오토바이 등이 바로 이 0.1~0.4 구간의 Low Confidence로 잡힙니다.
  ByteTrack은 이를 버리지 않고 **2단계 매칭**을 수행합니다:
  1. **1단계**: 높은 점수(High Conf) 탐지 결과와 기존 Track들을 칼만 필터 예측 위치 기반 IoU로 매칭.
  2. **2단계**: 1단계에서 매칭되지 못하고 남은 Track들을 **낮은 점수(Low Conf) 탐지 결과와 재매칭**.
* **성과**: 연산량이 극도로 가볍고(순수 CPU 연산으로 **1~2ms 내외** 처리 완료), 탐지기가 흔들리거나 일시적 가림이 발생해도 Track을 끊지 않고 질기게 유지합니다.

### 2.4 OC-SORT: 칼만 필터의 누적 오차를 깨다

* **구조**: 관측값 중심 칼만 필터(Observation-Centric SORT).
* **핵심 혁신**:
  표준 등속도 모델(Constant Velocity) 칼만 필터는 객체가 다른 차량 뒤로 숨어 관측이 누락되는 동안 오차를 눈덩이처럼 누적시킵니다. 차선 변경(Cut-in)이나 보행자 횡단처럼 비선형 궤적을 그릴 때 재등장한 위치와 칼만 예측 위치가 크게 어긋나 ID가 교체됩니다.
  OC-SORT는 두 가지 핵심 기법으로 이를 해결합니다:
  * **OCM(Observation-Centric Momentum)**: 칼만 예측치 대신 실제 이전 관측점 간의 방향 벡터를 속도 항에 반영하여 노이즈 축적 방지.
  * **ORU(Observation Recovery Online)**: 가림이 끝난 후 새 관측값이 들어오면 과거 결측 구간의 궤적 파라미터를 역방향으로 가상 갱신.

---

## 3. 종합 평가 및 의사결정 매트릭스

실차 ADAS 전방 카메라 환경(30 FPS, 경량 Detector, 실시간 제어 연계)을 기준으로 5개 평가 축에 가중치를 부여해 평가를 진행했습니다.

| 평가 기준 | 가중치 | StrongSORT | BoT-SORT | **ByteTrack** | **OC-SORT** |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **실시간성 (Real-time ≤ 5ms)** | 25% | 2 (부적격) | 3 | **5 (최상)** | **5 (최상)** |
| **임베디드 SoC 이식성** | 20% | 1 (ReID 필수) | 3 (연산 부담) | **5 (순수 C++)** | **5 (순수 C++)** |
| **약한 검출기(Miss/Blur) 대응력** | 20% | 3 | 4 | **5 (Low Conf 활용)**| 4 |
| **자차 거동/비선형 궤적 강건성** | 15% | 3 | **5 (GMC)** | 3 | **4 (OCM)** |
| **ID 안정성 (ID Switch 억제)** | 10% | **5** | 4 | 3 | 4 |
| **ISO 26262 안전 인증 용이성** | 10% | 2 | 3 | **5 (단순 구조)** | 4 |
| **최종 가중 점수 (5점 만점)** | 100% | 2.45 | 3.65 | **4.55** | **4.45** |

> **의사결정 결과**:
> 복잡한 ReID를 과감히 버리고, **`ByteTrack의 2단계 Association`을 기본 골격으로 하되, 비선형 거동 보정을 위해 `OC-SORT의 OCM(Observation-Centric Momentum)` 로직을 결합**하는 하이브리드 아키텍처를 최종 채택했습니다.

---

## 4. 최종 아키텍처 및 실차 적용 디테일

우리가 실제 타깃 보드에 구축한 전방 카메라 Tracking 파이프라인의 핵심 구조입니다.

```
[Raw Frame 1920x1080 @ 30fps]
           │
           ▼
[NPU Lightweight Detector (YOLO 계열)]
     ├── High-confidence boxes (conf ≥ 0.5)
     └── Low-confidence boxes (0.1 ≤ conf < 0.5)
           │
           ▼
[Stage 1: High-Confidence Matching]
     ├── Kalman State Prediction + OCM Momentum 보정
     └── Cost Matrix: IoU Distance (Hungarian Matching)
           │
           ▼ (Unmatched Tracks 추출)
[Stage 2: Low-Confidence Association]
     └── Unmatched Tracks ↔ Low-Confidence Bboxes 매칭
           │
           ▼
[Class-Aware Track Lifecycle Management]
     ├── Vehicle: max_age=30 frame (1초 유지, 고속 직진성)
     └── Pedestrian: max_age=15 frame (빠른 반응성, 급제동 대비)
           │
           ▼
[ADAS Warning & Control: FCWS / AEBS TTC 계산]
```

### 4.1 핵심 튜닝 포인트 3가지

#### 1) Low Confidence 구간의 양날의 검 (False Positive 제어)
Low Confidence 박스를 살리면 가림 현상이나 먼 거리 객체 인식률이 비약적으로 향상되지만, 도로변 가드레일, 그림자, 표지판 등이 오탐지되어 들어올 확률도 함께 높아집니다.
* **해결책**: 신규 Track 생성(Creation)은 오직 **High Confidence 검출 결과**로만 생성되도록 차단했습니다. Low Confidence 박스는 오직 **"이미 신뢰성이 입증되어 추적 중이던 기존 Track을 유지하는 용도"**로만 엄격히 제한했습니다.

#### 2) 객체 클래스별 동역학 분리 (Class-aware Kalman Tuning)
차량(Vehicle)과 보행자(Pedestrian)는 이동 속도와 가속도, 궤적 변화의 물리적 한계가 완전히 다릅니다.
* **차량**: 등속도 모델이 비교적 잘 들어맞으므로 Process Noise의 가속도 성분을 낮추고 `max_age`를 길게(30프레임) 설정해 짧은 터널이나 대형 트럭 추월 시에도 Track을 유지하도록 했습니다.
* **보행자**: 갑자기 멈추거나 방향을 꺾는 빈도가 높으므로 OCM 모멘텀 가중치를 높이고, `max_age`는 짧게(15프레임) 설정해 잔상이 남는 현상을 억제했습니다.

#### 3) CAN 기반 자차 거동(Ego-motion) 피드포워드
BoT-SORT의 영상 기반 GMC 대신, 차량 CAN 버스에서 10ms 주기로 올라오는 `Wheel Speed(휠속도)`와 `IMU Yaw Rate(각속도)`를 칼만 필터 예측 단계에 회전 변환 행렬로 직접 주입했습니다. 이로써 **CPU 점유율을 0.1%도 쓰지 않고 급제동 시 노즈다이브(Pitch) 및 코너링 시 추적 박스 쏠림 현상을 완벽히 보정**했습니다.

---

## 5. 실차 성능 검증 및 교훈

실차 주행 로그(도심지, 고속도로, 보행자 밀집 구역)를 기반으로 레거시 SORT 대비 성능을 측정한 결과입니다.

| 지표 | 레거시 SORT | DeepSORT (ReID 적용) | **제안 구조 (ByteTrack + OCM)** |
| :--- | :---: | :---: | :---: |
| **추적 연산 시간 (SoC CPU)** | **0.8 ms** | 18.4 ms (초과) | **2.1 ms (충족)** |
| **MOTA (Multi-Object Tracking Acc)**| 58.2% | 68.4% | **72.1%** |
| **ID Switch 횟수 (10분 주행)** | 142회 | 38회 | **31회** |
| **Far Distance(>60m) 객체 유지율** | 43% | 52% | **81%** |

### 엔지니어링 교훈 (Key Takeaways)

1. **임베디드에서는 "무엇을 더할까"보다 "무엇을 뺄 것인가"가 성능을 결정한다.**
   Deep Learning 기반 ReID는 매력적이지만, 비용 대비 효과가 명확하지 않은 무거운 모듈이었습니다. ReID를 완전히 덜어냄으로써 확보한 15ms 이상의 연산 마진은 전방 차선 인식(Lane Detection)과 주행 가능 공간(FreeSpace) 검출 모델을 더 고도화하는 데 온전히 투자할 수 있었습니다.

2. **낮은 신뢰도(Low Confidence) 데이터에 답이 있다.**
   많은 엔지니어들이 성능 저하를 해결하기 위해 검출기(Detector) 신경망 백본을 키우려고 합니다. 하지만 ByteTrack의 철학처럼, **이미 검출기가 찾아냈지만 버려지던 0.2~0.4 신뢰도의 데이터**를 시계열 컨텍스트로 구출해내는 알고리즘 구조만으로도 거대한 모델을 탑재한 것 이상의 품질 도약을 이룰 수 있었습니다.

3. **도메인 특화 데이터(CAN/IMU)를 적극 활용하라.**
   컴퓨터 비전 문제를 비전 알고리즘 안에서만 풀려고 하면 연산 병목에 부딪힙니다. 차량에 이미 장착된 저비용 고정밀 센서(CAN 차속, Yaw rate)를 필터에 융합하는 것만으로 복잡한 영상 기반 카메라 모션 보정(GMC)을 가볍게 대체할 수 있었습니다.
