# BEHAVIOR-1K 설치 가이드 (Blackwell GPU 환경)

> 시스템 CUDA 13.x + NVIDIA Blackwell GPU (compute capability 10.x) 환경에서의 설치 과정을 정리한 문서입니다.

## 환경 정보

| 항목 | 버전 |
|------|------|
| OS | Ubuntu (Linux 6.14) |
| 시스템 CUDA | 13.0 / 13.1 |
| GPU | NVIDIA Blackwell (compute capability 10.3) |
| Python | 3.10 |
| PyTorch | 2.6.0 (CUDA 12.4) |
| conda | miniforge3 |

## 설치 순서

### 1단계: 기본 환경 생성 + 모듈 설치 (primitives 제외)

`--primitives` 플래그는 nvidia-curobo 빌드 문제로 별도 처리합니다.

```bash
cd ~/Documents/BEHAVIOR-1K

./setup.sh --new-env --bddl --omnigibson --joylo --eval \
  --accept-conda-tos --accept-nvidia-eula
```

이 단계에서 설치되는 것:
- conda 환경 `behavior` (Python 3.10)
- PyTorch 2.6.0 + CUDA 12.4
- BDDL (bddl3/)
- OmniGibson + Isaac Sim
- JoyLo (joylo/)
- eval 의존성 (lerobot, hydra-core 등)

### 2단계: conda 환경에 CUDA 12.4 toolkit 설치

시스템 CUDA(13.x)와 PyTorch의 CUDA(12.4)가 불일치하므로, conda 환경 내에 12.4를 설치하여 오버라이드합니다.

```bash
conda activate behavior

conda install -c conda-forge cuda-nvcc=12.4 cuda-cudart-dev=12.4 -y
```

설치 확인:
```bash
export CUDA_HOME=$CONDA_PREFIX
export PATH=$CONDA_PREFIX/bin:$PATH
nvcc --version
# 반드시 12.4가 출력되어야 합니다
```

> **참고**: `conda install -c nvidia/label/cuda-12.4.0 cuda-toolkit -y`도 가능하지만, conda-forge 채널의 `cuda-nvcc` + `cuda-cudart-dev` 조합이 더 안정적으로 설치됩니다.

### 3단계: CUDA arch 설정 (Blackwell GPU 대응)

PyTorch 2.6.0은 Blackwell 아키텍처(compute capability 10.x)를 인식하지 못합니다. PTX 포워드 호환 빌드로 우회합니다.

```bash
export TORCH_CUDA_ARCH_LIST="8.9+PTX"
```

| 값 | 의미 |
|----|------|
| `8.9` | Ada Lovelace — PyTorch 2.6.0이 지원하는 최신 아키텍처 |
| `+PTX` | PTX 중간 코드 포함 → Blackwell에서 런타임 JIT 컴파일로 동작 |

### 4단계: nvidia-curobo 설치

```bash
pip install ninja~=1.13.0

pip install "nvidia-curobo @ git+https://github.com/StanfordVL/curobo@cbaf7d32436160956dad190a9465360fad6aba73" \
  --no-build-isolation
```

> `--no-build-isolation`: 현재 conda 환경의 CUDA toolkit과 PyTorch를 직접 사용하도록 강제합니다. 이 플래그가 없으면 pip가 격리된 빌드 환경을 만들어 CUDA를 찾지 못할 수 있습니다.

### 5단계: OMPL 설치

```bash
pip install "ompl @ https://storage.googleapis.com/gibson_scenes/ompl-1.6.0-cp310-cp310-manylinux_2_28_x86_64.whl"
```

### 6단계: 데이터셋 다운로드

```bash
./setup.sh --bddl --omnigibson --joylo --eval --dataset \
  --accept-nvidia-eula --accept-dataset-tos --confirm-no-conda
```

> `--confirm-no-conda`는 이미 conda 환경이 활성화된 상태에서 `--new-env` 없이 실행할 때 프롬프트를 건너뜁니다.

## 전체 명령어 요약 (복사/붙여넣기용)

```bash
# === 1. 기본 설치 (primitives 제외) ===
cd ~/Documents/BEHAVIOR-1K
./setup.sh --new-env --bddl --omnigibson --joylo --eval \
  --accept-conda-tos --accept-nvidia-eula

# === 2. CUDA toolkit 맞추기 ===
conda activate behavior
conda install -c conda-forge cuda-nvcc=12.4 cuda-cudart-dev=12.4 -y

# === 3. 환경변수 설정 ===
export CUDA_HOME=$CONDA_PREFIX
export PATH=$CONDA_PREFIX/bin:$PATH
export TORCH_CUDA_ARCH_LIST="8.9+PTX"

# === 4. curobo + ompl 설치 ===
pip install ninja~=1.13.0
pip install "nvidia-curobo @ git+https://github.com/StanfordVL/curobo@cbaf7d32436160956dad190a9465360fad6aba73" \
  --no-build-isolation
pip install "ompl @ https://storage.googleapis.com/gibson_scenes/ompl-1.6.0-cp310-cp310-manylinux_2_28_x86_64.whl"

# === 5. 데이터셋 ===
./setup.sh --bddl --omnigibson --joylo --eval --dataset \
  --accept-nvidia-eula --accept-dataset-tos --confirm-no-conda
```

## 설치 검증

```bash
conda activate behavior

# OmniGibson
python -c "import omnigibson; print('OmniGibson OK')"

# BDDL
python -c "import bddl; print('BDDL OK')"

# curobo
python -c "from curobo.types.robot import RobotConfig; print('curobo OK')"

# ompl
python -c "from ompl import base; print('OMPL OK')"

# PyTorch + CUDA
python -c "import torch; print(f'PyTorch {torch.__version__}, CUDA {torch.version.cuda}, GPU: {torch.cuda.get_device_name(0)}')"
```

## 트러블슈팅

### CUDA 버전 불일치 (`RuntimeError: The detected CUDA version (13.x) mismatches...`)

**원인**: 시스템 CUDA(13.x)가 PyTorch의 CUDA(12.4)와 다름

**해결**:
```bash
conda install -c conda-forge cuda-nvcc=12.4 cuda-cudart-dev=12.4 -y
export CUDA_HOME=$CONDA_PREFIX
export PATH=$CONDA_PREFIX/bin:$PATH
```

### Unknown CUDA arch (`ValueError: Unknown CUDA arch (10.3)`)

**원인**: PyTorch 2.6.0이 Blackwell GPU의 compute capability 10.3을 인식 못함

**해결**:
```bash
export TORCH_CUDA_ARCH_LIST="8.9+PTX"
```

### Failed building wheel for nvidia-curobo

**원인**: 빌드 격리 환경에서 CUDA를 찾지 못함

**해결**: `--no-build-isolation` 플래그 추가

### 환경변수 영속화

매번 export하기 번거로우면 conda 환경에 영구 설정:
```bash
conda activate behavior

mkdir -p $CONDA_PREFIX/etc/conda/activate.d
cat > $CONDA_PREFIX/etc/conda/activate.d/cuda_env.sh << 'EOF'
export CUDA_HOME=$CONDA_PREFIX
export PATH=$CONDA_PREFIX/bin:$PATH
export TORCH_CUDA_ARCH_LIST="8.9+PTX"
EOF
```

이후 `conda activate behavior`만 하면 자동 적용됩니다.
