# K1 Dance1 전신 모션 트래킹

WSL2 Ubuntu에서 기존 세인티 1일차 K1 모델과 3일차 uv 프로젝트를 사용한다. 상위 강의 배포본의 day1과 day3 폴더를 나란히 둔다. 현재 폴더에서 다음을 실행한다.

```bash
bash run.sh view reference
bash run.sh official
bash run.sh view official
bash run.sh pd
bash run.sh train --steps 16384 --seed 11 --checkpoint outputs/mine.zip
bash run.sh ppo --checkpoint outputs/mine.zip --out outputs/mine
bash run.sh video outputs/mine/ppo_roll0.npz
bash run.sh official --roll 40
```

- `view reference`는 기준 자세를 표시한다. `view official`과 `view ppo`는 물리 시뮬레이션으로 정책을 실행한다. GUI에는 WSLg가 필요하다. 영상 생성은 EGL과 ffmpeg를 사용한다.
- `official`은 assets의 공개 ONNX를 읽는다. `train`은 별도의 124→64→64→23 수업용 잔차 정책을 CPU로 학습한다. 보정 행동과 기준 동작을 더해 PD 관절 목표로 쓴다. 기본 평가 명령 `bash run.sh ppo`는 배포한 16,384스텝 체크포인트 outputs/ppo.zip을 읽는다.
- CLI 도움말은 각 명령 뒤에 `--help`를 붙인다. 상위 폴더에서는 `bash run.sh dance1 official`처럼 실행한다.
- 평가 CSV는 프레임별 관절 RMSE(rad), 골반 높이(m)를 담고 JSON은 실행 구간 전체의 RMSE와 완주 여부를 담는다. 시작 자세 0에서 마지막 자세 299까지 5.98초다. 종료 시간이 다르면 RMSE가 집계된 구간도 다르다. 같은 조건에서 여러 seed와 공통 구간을 비교한다.
- Dance1의 입력은 기준 동작과 로봇 몸의 상태다. 카메라나 언어 지시를 처리하는 입력은 없다. 이 실험이 다루는 범위는 한 안무의 시뮬레이션 추종이며 사람 인식, 작업 선택, 실물 안전은 별도로 평가해야 한다.
- 원본 안무와 가중치, 설정, 라이선스, 버전 및 체크섬은 assets/SOURCE.txt와 assets/LICENSE에 있다. 공식 대규모 훈련 절차는 [ROBOTIS Cyclo MJLab](https://docs.robotis.com/docs/systems/aisapiens/reinforcement_learning/cyclo_mjlab/mimic/)을 참조한다.
