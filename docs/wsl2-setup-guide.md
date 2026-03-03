# WSL2 + Ubuntu 24.04 설치 가이드

ROS 2는 Linux 환경에서 작동합니다. Windows에서 ROS 2를 사용하려면 **WSL2 (Windows Subsystem for Linux 2)**를 설치하는 것이 가장 편리합니다.

---

## 🎯 WSL2의 장점

✅ **Windows와 통합**: Windows 파일 시스템 접근 가능
✅ **GUI 지원**: WSLg로 RViz, Gazebo 등 GUI 앱 실행 가능
✅ **성능**: 거의 네이티브 Linux 수준의 성능
✅ **편의성**: 재부팅 없이 Windows와 Linux 동시 사용
✅ **VS Code 통합**: Remote-WSL extension으로 seamless 개발

---

## 📋 설치 단계

### Step 1: WSL2 설치

#### PowerShell (관리자 권한)에서 실행:

```powershell
# WSL 설치 (Ubuntu 24.04 자동 설치)
wsl --install -d Ubuntu-24.04

# 또는 WSL만 먼저 설치
wsl --install
```

**설치 후 재부팅 필요합니다.**

---

### Step 2: Ubuntu 24.04 설정

재부팅 후 Ubuntu가 자동으로 시작됩니다.

1. **사용자 계정 생성**
   - Username 입력 (소문자, 예: rosdev)
   - Password 입력 (두 번)

2. **초기 업데이트**
   ```bash
   sudo apt update
   sudo apt upgrade -y
   ```

---

### Step 3: WSL2 확인

Windows PowerShell에서:
```powershell
# WSL 버전 확인
wsl --list --verbose

# 출력 예시:
# NAME            STATE           VERSION
# Ubuntu-24.04    Running         2
```

**VERSION이 2인지 확인!** (1이면 업그레이드 필요)

#### WSL 1 → 2 업그레이드 (필요시):
```powershell
wsl --set-version Ubuntu-24.04 2
```

---

### Step 4: VS Code WSL 연동

1. **VS Code 설치** (Windows에)
   - https://code.visualstudio.com/

2. **WSL Extension 설치**
   - VS Code에서 `Remote - WSL` extension 설치

3. **WSL에서 VS Code 실행**
   ```bash
   # Ubuntu 터미널에서
   cd ~
   code .
   ```
   → Windows의 VS Code가 WSL 모드로 열립니다

---

### Step 5: ROS 2 Jazzy 설치

Ubuntu 24.04 터미널에서:

```bash
# ARGOS 프로젝트 디렉토리로 이동 (Windows 파일 접근)
cd /mnt/c/Users/USER/Desktop/ARGOS

# 설치 스크립트 실행
bash scripts/setup-ros2-jazzy.sh
```

**예상 소요 시간**: 20-30분

---

## 🛠️ WSL2 유용한 명령어

### Windows PowerShell에서

```powershell
# WSL Ubuntu 시작
wsl -d Ubuntu-24.04

# WSL 종료
wsl --shutdown

# WSL 상태 확인
wsl --list --verbose

# 특정 WSL 제거 (주의!)
wsl --unregister Ubuntu-24.04
```

### Ubuntu에서 Windows 파일 접근

```bash
# Windows C:\ 드라이브
cd /mnt/c/

# 사용자 폴더
cd /mnt/c/Users/USER/

# ARGOS 프로젝트
cd /mnt/c/Users/USER/Desktop/ARGOS/
```

### Windows에서 WSL 파일 접근

Windows 탐색기 주소창에:
```
\\wsl$\Ubuntu-24.04\home\yourusername\
```

---

## 🎨 WSL 개발 워크플로우

### 추천 개발 환경 구성

1. **Windows에서**:
   - VS Code 실행
   - 웹 브라우저 (문서 검색)
   - Git GUI 도구 (선택)

2. **WSL Ubuntu에서**:
   - ROS 2 노드 개발 및 실행
   - 빌드 (colcon build)
   - 시뮬레이션 (Gazebo, RViz)

3. **VS Code Remote-WSL**:
   - WSL 파일 시스템 직접 편집
   - WSL 터미널 통합
   - Debugging in WSL

---

## 🐛 일반적인 문제 해결

### 문제 1: "WSL 2 requires an update to its kernel component"

**해결**:
1. https://aka.ms/wsl2kernel 다운로드
2. WSL2 Linux kernel update package 설치
3. 재시작

### 문제 2: GUI 앱이 안 보임 (RViz, Gazebo)

**해결** (Windows 11):
- WSLg가 자동 설치됨
- `wsl --update` 실행

**해결** (Windows 10):
- VcXsrv 또는 X410 설치 필요
- `export DISPLAY=:0` 설정

### 문제 3: 네트워크 느림

**해결**:
```bash
# DNS 설정
sudo nano /etc/wsl.conf

# 추가:
[network]
generateResolvConf = false

# WSL 재시작
```

### 문제 4: 디스크 공간 부족

**해결** (PowerShell):
```powershell
# WSL 디스크 압축
wsl --shutdown
Optimize-VHD -Path C:\Users\USER\AppData\Local\Packages\CanonicalGroupLimited.Ubuntu24.04LTS_*\LocalState\ext4.vhdx -Mode Full
```

---

## 🎯 설치 완료 체크리스트

- [ ] WSL2 설치됨 (버전 2 확인)
- [ ] Ubuntu 24.04 실행됨
- [ ] VS Code WSL extension 설치됨
- [ ] WSL에서 `code .` 명령어 작동
- [ ] Windows ↔ WSL 파일 접근 가능
- [ ] ROS 2 Jazzy 설치 준비 완료

---

## 🔗 추가 리소스

- [Microsoft WSL 공식 문서](https://docs.microsoft.com/en-us/windows/wsl/)
- [WSL2 Best Practices](https://docs.microsoft.com/en-us/windows/wsl/setup/environment)
- [VS Code WSL 개발](https://code.visualstudio.com/docs/remote/wsl)

---

## 대안: 듀얼 부팅 (Native Ubuntu)

WSL2 대신 네이티브 Ubuntu를 선호한다면:

### 장점
- 최고 성능
- 하드웨어 직접 접근 (GPU, 카메라 등)
- 100% Linux 환경

### 단점
- Windows 재부팅 필요
- 파티션 설정 필요
- 초보자에게 복잡

### 설치 가이드
1. Ubuntu 24.04 ISO 다운로드
2. Rufus로 부팅 USB 생성
3. 파티션 최소 50GB 할당
4. 듀얼 부팅 설정

**권장**: 먼저 WSL2로 시작 → 나중에 필요하면 네이티브 Ubuntu 고려

---

**다음 단계**: WSL2 설치 후 → `setup-ros2-jazzy.sh` 실행
