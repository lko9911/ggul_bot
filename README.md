# 🧪 ggul_bot

### 변동 사항

1. CMakeLists.txt <br>
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME}) 주석 처리 (assistant 파일을 프로그램이 못 찾음)<br><br>

2. ggul_bot_v6_config <br>
UDRF 참조 경로 수정 (상대경로에서 절대경로로)<br><br>

### Strawberry_publisher_pkg 제작 <br>
- git clone 방법 동일, 이후 아래코드 실행해서 라이브러리 설치
<pre><code>cd /ros2_ws2/ggul_bot/strawberry_publisher_pkg/strawberry_publisher_pkg/
pip3 install -r requirements.txt </code></pre>
- pip 설치 코드
<pre><code>apt install python3-pip</code></pre>
- 패키지 사용
<pre><code>ros2 run strawberry_publisher_pkg strawberry_publisher</code></pre>
<br><br>
- 아직 IK랑 퍼블리쉬 안함 
- 코드 작동만 확인
- docker 환경에서 웹캠 사용 여부 확인 필요
