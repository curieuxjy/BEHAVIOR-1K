# BEHAVIOR-1K 설치 가이드 (Blackwell GPU 환경)

> 시스템 CUDA 13.x + NVIDIA Blackwell GPU (RTX 5090, sm_120) 환경에서의 설치 과정을 정리한 문서입니다.
>
> **핵심 이슈**: `setup.sh`가 설치하는 PyTorch 2.6.0은 sm_90(Hopper)까지만 지원하며,
> RTX 5090(sm_120)에서는 CUDA 연산이 불가능합니다.
> 반드시 PyTorch nightly(cu128)로 교체해야 합니다.

## 환경 정보

| 항목 | 버전 |
|------|------|
| OS | Ubuntu (Linux 6.14) |
| 시스템 CUDA | 13.0 / 13.1 |
| GPU | NVIDIA GeForce RTX 5090 (Blackwell, sm_120) |
| 드라이버 | 580.95 |
| Python | 3.10 |
| PyTorch | nightly (cu128) — **2.6.0은 사용 불가** |
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
- PyTorch 2.6.0 + CUDA 12.4 (이후 교체 예정)
- BDDL (bddl3/)
- OmniGibson + Isaac Sim
- JoyLo (joylo/)
- eval 의존성 (lerobot, hydra-core 등)

### 2단계: PyTorch nightly로 교체 (RTX 5090 필수)

**setup.sh가 설치하는 PyTorch 2.6.0(cu124)은 RTX 5090(sm_120)을 지원하지 않습니다.**
시뮬레이션 창은 열리지만 씬이 렌더링되지 않고, 아래 경고가 출력됩니다:

```
NVIDIA GeForce RTX 5090 with CUDA capability sm_120 is not compatible
with the current PyTorch installation.
The current PyTorch install supports CUDA capabilities sm_50 ... sm_90.
```

PyTorch nightly(CUDA 12.8, sm_120 지원)로 교체하고, 관련 패키지를 재정비합니다:

```bash
conda activate behavior

# 기존 PyTorch 제거
pip uninstall torch torchvision torchaudio -y

# PyTorch nightly (cu128) 설치
pip install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/cu128

# NumPy를 1.x로 고정 (Isaac Sim 4.5가 np.float_ 등 NumPy 1.x API를 사용)
pip install "numpy<2"

# torch_cluster 재빌드 (PyTorch 버전 변경 시 ABI 불일치)
pip uninstall torch-cluster -y
pip install torch-cluster --no-cache-dir

# 확인 — True + RTX 5090 + 경고 없음이어야 합니다
python -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0))"
```

### 3단계: conda 환경에 CUDA 12.4 toolkit 설치 (curobo 빌드용)

시스템 CUDA(13.x)와 curobo 빌드에 필요한 CUDA 버전을 맞추기 위해, conda 환경에 12.4를 설치합니다.

```bash
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

### 4단계: CUDA arch 설정 (curobo 빌드용)

curobo 빌드 시 PyTorch가 Blackwell arch를 인식하지 못하므로 PTX 포워드 호환으로 빌드합니다.

```bash
export TORCH_CUDA_ARCH_LIST="8.9+PTX"
```

| 값 | 의미 |
|----|------|
| `8.9` | Ada Lovelace — PyTorch가 지원하는 최신 아키텍처 |
| `+PTX` | PTX 중간 코드 포함 → Blackwell에서 런타임 JIT 컴파일로 동작 |

### 5단계: nvidia-curobo 설치

```bash
pip install ninja~=1.13.0

pip install "nvidia-curobo @ git+https://github.com/StanfordVL/curobo@cbaf7d32436160956dad190a9465360fad6aba73" \
  --no-build-isolation
```

> `--no-build-isolation`: 현재 conda 환경의 CUDA toolkit과 PyTorch를 직접 사용하도록 강제합니다. 이 플래그가 없으면 pip가 격리된 빌드 환경을 만들어 CUDA를 찾지 못할 수 있습니다.
>
> **주의**: PyTorch nightly로 교체한 후 curobo를 빌드해야 합니다. PyTorch 버전이 바뀌면 curobo 재빌드가 필요합니다.

### 6단계: OMPL 설치

```bash
pip install "ompl @ https://storage.googleapis.com/gibson_scenes/ompl-1.6.0-cp310-cp310-manylinux_2_28_x86_64.whl"
```

### 7단계: 데이터셋 다운로드

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

# === 2. PyTorch nightly로 교체 (RTX 5090 필수) ===
conda activate behavior
pip uninstall torch torchvision torchaudio -y
pip install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/cu128
pip install "numpy<2"
pip uninstall torch-cluster -y && pip install torch-cluster --no-cache-dir
python -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"

# === 3. CUDA toolkit 맞추기 (curobo 빌드용) ===
conda install -c conda-forge cuda-nvcc=12.4 cuda-cudart-dev=12.4 -y

# === 4. 환경변수 설정 ===
export CUDA_HOME=$CONDA_PREFIX
export PATH=$CONDA_PREFIX/bin:$PATH
export TORCH_CUDA_ARCH_LIST="8.9+PTX"

# === 5. curobo + ompl 설치 ===
pip install ninja~=1.13.0
pip install "nvidia-curobo @ git+https://github.com/StanfordVL/curobo@cbaf7d32436160956dad190a9465360fad6aba73" \
  --no-build-isolation
pip install "ompl @ https://storage.googleapis.com/gibson_scenes/ompl-1.6.0-cp310-cp310-manylinux_2_28_x86_64.whl"

# === 6. 데이터셋 ===
./setup.sh --bddl --omnigibson --joylo --eval --dataset \
  --accept-nvidia-eula --accept-dataset-tos --confirm-no-conda
```

## 설치 검증

```bash
conda activate behavior

# PyTorch + CUDA (가장 중요 — True + RTX 5090 + 경고 없음)
python -c "import torch; print(f'PyTorch {torch.__version__}, CUDA {torch.version.cuda}, GPU: {torch.cuda.get_device_name(0)}, Available: {torch.cuda.is_available()}')"

# OmniGibson
python -c "import omnigibson; print('OmniGibson OK')"

# BDDL
python -c "import bddl; print('BDDL OK')"

# curobo
python -c "from curobo.types.robot import RobotConfig; print('curobo OK')"

# ompl
python -c "from ompl import base; print('OMPL OK')"

# 시뮬레이션 실행 테스트
python -m omnigibson.examples.robots.robot_control_example --quickstart
```

## 트러블슈팅

### RTX 5090에서 시뮬레이션 창은 열리지만 씬이 안 보임

**증상**:
```
NVIDIA GeForce RTX 5090 with CUDA capability sm_120 is not compatible
with the current PyTorch installation.
```
렌더러 에러(`getResourcesDescriptorSet failed for pool 1`)와 함께 빈 화면이 나타남.

**원인**: PyTorch 2.6.0이 RTX 5090(sm_120)을 지원하지 않아 CUDA 연산 자체가 불가능

**해결**: PyTorch nightly(cu128)로 교체 (위 2단계 참조)
```bash
pip uninstall torch torchvision torchaudio -y
pip install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/cu128
```

### NumPy 2.x 비호환 (`np.float_ was removed in the NumPy 2.0 release`)

**원인**: PyTorch nightly가 NumPy 2.x를 끌고 오지만, Isaac Sim 4.5의 OmniGraph 노드들이 `np.float_` 등 NumPy 1.x API를 사용

**해결**:
```bash
pip install "numpy<2"
```

### torch_cluster ABI 불일치 (`undefined symbol: _ZN5torch3jit17parseSchemaOrName...`)

**원인**: PyTorch 2.6.0으로 빌드된 torch_cluster가 nightly와 호환되지 않음

**해결**:
```bash
pip uninstall torch-cluster -y
pip install torch-cluster --no-cache-dir
```

### 비전 센서 초기화 실패 (`TypeError: Unable to write from unknown dtype, kind=f, size=0`)

**원인**: Isaac Sim 4.5의 렌더링 파이프라인(omni.syntheticdata)이 RTX 5090에서 정상 동작하지 않음

**해결**: 비전 센서 없이 실행 (물리 시뮬레이션은 정상 동작)
```python
gm.RENDER_VIEWER_CAMERA = False
gm.HEADLESS = True
# robot config에서 obs_modalities=[] 사용
```

> Isaac Sim의 향후 Blackwell 지원 업데이트 전까지는 RGB/depth 센서 사용이 제한됩니다.

### CUDA 버전 불일치 (`RuntimeError: The detected CUDA version (13.x) mismatches...`)

**원인**: curobo 빌드 시 시스템 CUDA(13.x)가 PyTorch의 CUDA와 다름

**해결**:
```bash
conda install -c conda-forge cuda-nvcc=12.4 cuda-cudart-dev=12.4 -y
export CUDA_HOME=$CONDA_PREFIX
export PATH=$CONDA_PREFIX/bin:$PATH
```

### Unknown CUDA arch (`ValueError: Unknown CUDA arch (10.3)`)

**원인**: curobo 빌드 시 Blackwell GPU의 compute capability를 인식 못함

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

## 알려진 제약 사항

### Isaac Sim 5.x 업그레이드 불가

**현재 BEHAVIOR-1K는 Isaac Sim 4.5.0에 하드코딩되어 있어 Isaac Sim 5.x로 업그레이드할 수 없습니다.**

#### 근거

1. **setup.sh에 버전 고정**: `setup.sh`가 `isaacsim-4.5.0.0`, `isaacsim_kernel-4.5.0.0`, `omniverse_kit-106.5.0.162521` 등 모든 Isaac Sim 패키지를 4.5.0 버전으로 하드코딩하여 설치합니다 (setup.sh 306~314행).

2. **OmniGibson API 의존성**: OmniGibson 코드 전반에서 `isaacsim.core.api`, `isaacsim.core.utils.prims`, `isaacsim.core.utils.semantics`, `isaacsim.sensors.physics` 등 Isaac Sim 4.5.0의 모듈 경로를 사용합니다. Isaac Sim 5.x는 API 구조가 변경되었을 가능성이 높아 호환되지 않습니다.

3. **Kit 파일 호환성**: OmniGibson은 `omnigibson_4_5_0.kit` 파일로 Omniverse Kit 확장을 로드하며, 이 설정은 Kit 106.5.0 (Isaac Sim 4.5.0 내장)에 맞춰져 있습니다.

4. **VisionSensor / syntheticdata 파이프라인**: Isaac Sim 4.5.0의 `omni.syntheticdata` annotator는 Blackwell GPU(sm_120)에서 `TypeError: Unable to write from unknown dtype, kind=f, size=0` 에러로 크래시합니다. 이 문제는 Isaac Sim 5.x에서 Blackwell을 공식 지원하면 해결될 가능성이 높습니다.

#### 영향

| 기능 | 현재 상태 (Isaac Sim 4.5.0 + RTX 5090) |
|------|----------------------------------------|
| 물리 시뮬레이션 | 정상 동작 |
| 로봇 제어 (조인트, 베이스) | 정상 동작 |
| Viewport 시각화 | `RENDER_VIEWER_CAMERA=False` + 수동 카메라 설정 필요 |
| RGB/Depth 센서 (VisionSensor) | 사용 불가 (annotator 크래시) |
| BehaviorTask (태스크 실행) | 물리 기반 동작은 가능, 센서 기반 관찰은 제한적 |

#### 해결 전망

- **단기**: `gm.RENDER_VIEWER_CAMERA = False` + `obs_modalities = []` + USD API로 뷰포트 카메라 직접 설정하여 사용
- **중기**: NVIDIA가 Isaac Sim 5.x에서 Blackwell(sm_120)을 공식 지원 시, OmniGibson/BEHAVIOR-1K 업스트림에서 5.x 대응 업데이트 필요
- **장기**: BEHAVIOR-1K 레포지토리가 Isaac Sim 5.x를 지원하는 브랜치/릴리스를 게시하면 업그레이드 가능

#### Viewport에서 아무것도 안 보이는 문제

`RENDER_VIEWER_CAMERA=False`로 VisionSensor를 우회하면 OmniGibson이 뷰포트 카메라를 설정하지 않습니다. 해결 방법:

```python
# 환경 생성 후, USD Camera prim을 직접 생성하고 뷰포트에 연결
from pxr import UsdGeom, Gf
import omni.usd
import omni.kit.viewport.window

stage = omni.usd.get_context().get_stage()
cam_path = "/World/viewport_camera"
UsdGeom.Camera.Define(stage, cam_path)

prim = stage.GetPrimAtPath(cam_path)
xf = UsdGeom.Xformable(prim)
xf.ClearXformOpOrder()
xf.AddTranslateOp().Set(Gf.Vec3d(-0.2, -2.7, 1.1))
# 쿼터니언: USD는 (w,x,y,z) 순서
xf.AddOrientOp().Set(Gf.Quatf(0.73138017, 0.68196617, -0.00155408, -0.00166678))

UsdGeom.Camera(prim).GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10000.0))
UsdGeom.Camera(prim).GetFocalLengthAttr().Set(17.0)

vps = list(omni.kit.viewport.window.get_viewport_window_instances())
vps[0].viewport_api.set_active_camera(cam_path)
```

이 코드는 `custom/lab1_anatomy/inspect_r1pro.py`의 `setup_viewport_camera()` 함수로 구현되어 있습니다.
