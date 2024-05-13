## Project Title - 로프구동 사족로봇의 3차원 외벽 장애물 극복을 위한 궤적 최적화
사족보행 로봇의 3차원 환경에 존재하는 여러 가지 장애물을 안정적으로 극복하기 위한 무게중심의 궤적 최적화를 위한 프로젝트 입니다. 

본 연구는 로프 시스템을 가진 사족 보행 로봇이 장애물 극복 시 흔들리는 무게중심의 변화량을 최소화하는 것을 목표로 합니다.

특히, 로봇 몸체의 무게중심의 z축 방향 병진과 pitch 방향 회전 변화량을 최소화하는 것에 집중합니다.

## Description

본 연구는 등강용 로봇(Ascender)와 장애물 극복 로봇(Wheel-leg)로 구성된 외벽 청소 로봇의 후속 연구입니다.

등강용 로봇에 해당하는 부분의 무게(mass)와 관성력(inertia)를 사족 보행 로봇의 무게중심 부분으로 집중 시키는 Single Rigid Body Dynamics(SRBD)를 이용하여 연구의 단순화를 진행했습니다. 
그리고 로프가 새롭게 추가되며 생기는 장력 또한 동역학 계산에 고려했습니다.

Fig. 1은 로프 시스템을 가진 사족 보행 로봇의 locomotion을 보여줍니다.
<p align="center">
  <img src="https://github.com/HyeonguDO/research_towr/blob/master/image01.png"/>
</p>
<p align="center">[Fig. 1] The locomotion of quadrupedal robot</p>

---

로프의 장력을 고려한 동역학 식인 수정된 Single Rigid Body Dynamics(SRBD)를 계산하기 위해 Fig. 2 ~ Fg. 5의 과정을 따릅니다.

<p align="center">
  <img src="https://github.com/HyeonguDO/research_towr/blob/master/dynamic_config.png"/>
</p>
<p align="center">[Fig. 2] The force configuration of quadruped robot with rope tension (for calculating dynamics)</p>

<p align="center">
  <img src="https://github.com/HyeonguDO/research_towr/blob/master/dynamic_formula.png"/>
</p>
<p align="center">[Fig. 3] The dynamic formula of quadruped robot with rope tension</p>

<p align="center">
  <img src="https://github.com/HyeonguDO/research_towr/blob/master/tension_config.png"/>
</p>
<p align="center">[Fig. 4] The force configuration of quadruped robot with rope tension (for calculating tension)</p>

<p align="center">
  <img src="https://github.com/HyeonguDO/research_towr/blob/master/tension_formula.png"/>
</p>
<p align="center">[Fig. 5] The tension formula of quadruped robot with rope tension</p>

---
TOWR를 이용해 로봇의 목표 지점까지 이동하기 위한 궤적 최적화는 크게 3가지 패키지(towr, ifopt, xpp)가 사용됩니다. 

+ towr: 사족 보행 로봇의 motion planning 및 이동 궤적 최적화를 위해 로봇의 정보(mass, inertia, dynamics, kinemaics etc.)와 구속조건(constraints)을 수학적으로 정의하는 부분
+ ifopt : towr 패키지에서 수학적으로 정의된 조건들을 바탕으로 최적 조건을 계산하기 위한 비선형 solver
+ xpp: towr와 ifopt를 이용해 계산된 결과를 시각적으로 보여주기 위한 tool

Fig. 6는 그래프의 모든 영역을 로봇이 이동할 수 있는 범위로 가정했을 때 로봇의 파라미터, 목표와 구속조건이 모두 포함되는 영역으로 그 범위를 제한 함으로써 궤적 최적화 방식을 나타내고 있습니다. 

<p align="center">
  <img src="https://github.com/HyeonguDO/research_towr/blob/master/about_towr.png" height="400px" width="650px"/>
</p>
<p align="center">[Fig. 6] The diagram for optimization method of ‘TOWR’</p>

TOWR에서 수학적으로 정의하는 로봇의 파라미터와 구속조건은 Fig. 7을 따르며 본 연구를 위해 수정된 부분은 빨간색으로 표시된 "dynamic model, rope tension, base motion constraint, terrain height" 입니다.
<p align="center">
  <img src="https://github.com/HyeonguDO/research_towr/blob/master/constraint_list.png"/>
</p>
<p align="center">[Fig. 7] Definition of variables and constraints for optimizing the legged locomotion</p>

본 연구에서 새롭게 추가된 조건인 base motion constraint는 외벽 청소 로봇이 외벽 존재하는 장애물 극복 시 로봇 몸체의 흔들림을 최소화하기 위해 추가됐고, z축 방향의 병진운동과 pitch 방향의 회전운동의 범위를 제한 함으로써 몸체의 흔들림을 제어하게 되며 
이를 표현하는 식은 Fig. 8과 같습니다. 
<p align="center">
  <img src="https://github.com/HyeonguDO/research_towr/blob/master/CoM_constraint.png"/>
</p>
<p align="center">[Fig. 8] The CoM constraint for optimizing the legged locomotion</p>

시뮬레이션은 roll, pitch 각도가 동시에 변하는 지형에 존재하는 4가지 형태의 장애물에 대하여 진행되었고 장애물의 모습은 Fig. 9와 같습니다.
<p align="center">
  <img src="https://github.com/HyeonguDO/research_towr/blob/master/terrain.png"/>
</p>
<p align="center">[Fig. 9] The definition of terrain with obstacles. (a) Impulse (b) Step (c) Small pit (d) Large pit</p>

## Simulation Result
+ Step 결과
![Step_opt (video-converter com)](https://github.com/HyeonguDO/research_towr/assets/134991454/cf7c844e-d9d0-44f8-940c-79195b32c990)

Fig. 10은 Step 장애물을 극복하는 동안의 무게중심의 병진 및 회전 변화량을 나타냅니다. 파란색 파선은 base motion constraint를 적용하기 전을, 빨간색 실선은 base motion constraint를 적용한 후를 나타냅니다.
<p align="center">
  <img src="https://github.com/HyeonguDO/research_towr/blob/master/step_data.png"/>
</p>
<p align="center">[Fig. 10] The trajectory optimization data during overcoming step obstacle</p>


Table. 1은 RMSE를 이용하여 최적화 정도를 계산한 것을 나타내며 클수록 성능개선 효과가 크다는 것을 의미합니다.  
|Step Case|Normal|Optimal|Difference [%]|
|:---:|:---:|:---:|:---:|
|z-CoM position|0.1068|0.0696|53.30|
|Pitch angle |0.0547|0.0555|-1.49|
</p>
<p align="center">[Table. 1] The trajectory optimization data during overcoming Step obstacle</p>

---

+ Impulse 결과  
![Impulse_opt (video-converter com)](https://github.com/HyeonguDO/research_towr/assets/134991454/b8472917-4cc7-4583-8ea3-d274f874139d)

Fig. 11는 Impulse 장애물을 극복하는 동안의 무게중심의 병진 및 회전 변화량을 나타냅니다. 파란색 파선은 base motion constraint를 적용하기 전을, 빨간색 실선은 base motion constraint를 적용한 후를 나타냅니다.
<p align="center">
  <img src="https://github.com/HyeonguDO/research_towr/blob/master/impulse_data.png"/>
</p>
<p align="center">[Fig. 11] The trajectory optimization data during overcoming Impulse obstacle</p>


Table. 2은 RMSE를 이용하여 최적화 정도를 계산한 것을 나타내며 클수록 성능개선 효과가 크다는 것을 의미합니다.  
|Step Case|Normal|Optimal|Difference [%]|
|:---:|:---:|:---:|:---:|
|z-CoM position|0.0793|0.0735|7.89|
|Pitch angle |0.0496|0.0441|12.60|
</p>
<p align="center">[Table. 2] The trajectory optimization data during overcoming Impulse obstacle</p>

---

+ Large pit 결과
![Large_pit_opt (video-converter com)](https://github.com/HyeonguDO/research_towr/assets/134991454/34a312f8-57d5-4e59-a51b-722fe3237f6a)

Fig. 12는 Large Pit 장애물을 극복하는 동안의 무게중심의 병진 및 회전 변화량을 나타냅니다. 파란색 파선은 base motion constraint를 적용하기 전을, 빨간색 실선은 base motion constraint를 적용한 후를 나타냅니다.
<p align="center">
  <img src="https://github.com/HyeonguDO/research_towr/blob/master/large_pit_data.png"/>
</p>
<p align="center">[Fig. 12] The trajectory optimization data during overcoming Large Pit obstacle</p>


Table. 3은 RMSE를 이용하여 최적화 정도를 계산한 것을 나타내며 클수록 성능개선 효과가 크다는 것을 의미합니다.  
|Step Case|Normal|Optimal|Difference [%]|
|:---:|:---:|:---:|:---:|
|z-CoM position|0.1429|0.1214|17.57|
|Pitch angle |0.1002|0.0915|9.45|
</p>
<p align="center">[Table. 3] The trajectory optimization data during overcoming Large Pit obstacle</p>

---

+ Small pit 결과
![Small_pit_opt (video-converter com)](https://github.com/HyeonguDO/research_towr/assets/134991454/f0a22152-3dcc-44f5-ab13-7e0ac32ce73f)

Fig. 13는 Small Pit 장애물을 극복하는 동안의 무게중심의 병진 및 회전 변화량을 나타냅니다. 파란색 파선은 base motion constraint를 적용하기 전을, 빨간색 실선은 base motion constraint를 적용한 후를 나타냅니다.
<p align="center">
  <img src="https://github.com/HyeonguDO/research_towr/blob/master/small_pit_data.png"/>
</p>
<p align="center">[Fig. 13] The trajectory optimization data during overcoming Small Pit obstacle</p>


Table. 4은 RMSE를 이용하여 최적화 정도를 계산한 것을 나타내며 클수록 성능개선 효과가 크다는 것을 의미합니다.  
|Step Case|Normal|Optimal|Difference [%]|
|:---:|:---:|:---:|:---:|
|z-CoM position|0.0530|0.0504|5.17|
|Pitch angle |0.0472|0.0311|51.96|
</p>
<p align="center">[Table. 4] The trajectory optimization data during overcoming Small Pit obstacle</p>

## Author

Hyeongu Do
