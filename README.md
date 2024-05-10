 24.05.10 미완성

## Project Title - 로프구동 사족로봇의 3차원 외벽 장애물 극복을 위한 궤적 최적화
사족보행 로봇의 3차원 환경에 존재하는 여러 가지 장애물을 안정적으로 극복하기 위한 무게중심의 궤적 최적화를 위한 프로젝트 입니다. 

본 연구는 로프 시스템을 가진 사족 보행 로봇이 장애물 극복 시 흔들리는 무게중심의 변화량을 최소화하는 것을 목표로 합니다.

특히, 로봇 몸체의 무게중심의 z축 방향 병진과 pitch 방향 회전 변화량을 최소화하는 것에 집중합니다.

## Description

사진1로 사용환경 제시
<p align="center">
  <img src="https://github.com/HyeonguDO/research_towr/blob/master/image01.png"/>
</p>

사진2로 로프를 포함한 동역학, 로프의 세기를 정역학해석으로 구했음을 표현 (사진이랑 수식 캡쳐뜨기)
<p align="center">
  <img src="https://github.com/HyeonguDO/research_towr/blob/master/dynamic_config.png"/>
</p>

<p align="center">
  <img src="https://github.com/HyeonguDO/research_towr/blob/master/dynamic_formula.png"/>
</p>

<p align="center">
  <img src="https://github.com/HyeonguDO/research_towr/blob/master/tension_config.png"/>
</p>

<p align="center">
  <img src="https://github.com/HyeonguDO/research_towr/blob/master/tension_formula.png"/>
</p>

사진3으로 로봇베이스 최적화를 위한 방식 설명(로봇베이스의 z높낮이와 pitch 각도 범위 조절로 인한.. 수식도 캡쳐뜨기)
<p align="center">
  <img src="https://github.com/HyeonguDO/research_towr/blob/master/about_towr.png" height="400px" width="650px"/>
</p>

<p align="center">
  <img src="https://github.com/HyeonguDO/research_towr/blob/master/constraint_list.png"/>
</p>

<p align="center">
  <img src="https://github.com/HyeonguDO/research_towr/blob/master/CoM_constraint.png"/>
</p>

<p align="center">
  <img src="https://github.com/HyeonguDO/research_towr/blob/master/terrain.png"/>
</p>

실제 움직이는 움짤로 사용하는 모습 보이기 (4가지 케이스)

데이터 결과는 어떻게?


## Authors

Contributors names and contact info

ex. Dominique Pizzie  
ex. [@DomPizzie](https://twitter.com/dompizzie)

## Version History

* 0.2
    * Various bug fixes and optimizations
    * See [commit change]() or See [release history]()
* 0.1
    * Initial Release

## License

This project is licensed under the [NAME HERE] License - see the LICENSE.md file for details

## Acknowledgments

Inspiration, code snippets, etc.
* [awesome-readme](https://github.com/matiassingers/awesome-readme)
* [PurpleBooth](https://gist.github.com/PurpleBooth/109311bb0361f32d87a2)
* [dbader](https://github.com/dbader/readme-template)
* [zenorocha](https://gist.github.com/zenorocha/4526327)
* [fvcproductions](https://gist.github.com/fvcproductions/1bfc2d4aecb01a834b46)
