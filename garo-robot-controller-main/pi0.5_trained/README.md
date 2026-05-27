# Pi0.5 학습된 모델 폴더

이 폴더에 Pi0.5 학습된 모델 체크포인트를 넣어야 합니다.

## 필요한 폴더 구조

```
pi0.5_trained/
├── v01/
│   └── 012000/
│       └── pretrained_model/
│           ├── model.safetensors          # 모델 가중치 (필수)
│           ├── config.json                # 모델 설정 (필수)
│           ├── train_config.json          # 학습 설정
│           ├── policy_preprocessor.json   # 입력 전처리 설정
│           ├── policy_postprocessor.json  # 출력 후처리 설정
│           └── *.safetensors              # 정규화 가중치
├── v02/
│   └── 012000/
│       └── pretrained_model/
│           └── (위와 동일한 구조)
├── v03/ ...
├── v04/ ...
├── v42/ ...
└── v43/ ...
```

## 설명

- 각 `vXX/` 폴더는 학습 버전을 나타냅니다
- 각 버전 안에 `012000/`, `018000/` 등의 체크포인트 폴더가 있습니다
- `pretrained_model/` 안에 실제 모델 파일이 들어갑니다
- `model.safetensors`와 `config.json`이 필수 파일입니다
- Inference GUI에서 이 폴더를 자동으로 스캔하여 사용 가능한 모델을 표시합니다

## 참고

- 모델 파일은 용량이 매우 크므로 (총 약 196GB) 별도로 관리됩니다
- 필요한 모델 버전만 선택적으로 복사해도 됩니다
