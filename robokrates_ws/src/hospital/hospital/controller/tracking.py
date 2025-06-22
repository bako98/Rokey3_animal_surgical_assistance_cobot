# controller.robot_control.py

# ROS 2 로깅 모듈 임포트
from rclpy.logging import get_logger
# 가비지 컬렉션 모듈 임포트
import gc

# 파일 시스템 접근을 위한 os 모듈
import os
# 시간 관련 함수 모듈
import time
# 시스템 관련 기능 모듈
import sys
# 3D 회전 계산을 위한 모듈
from scipy.spatial.transform import Rotation
# 수치 계산 라이브러리
import numpy as np
# ROS 2 Python 클라이언트 라이브러리
import rclpy
# ROS 2 노드 기본 클래스
from rclpy.node import Node
# DSR 로봇 초기화 모듈
import DR_init
# 사용자 정의 서비스 메시지
from od_msg.srv import SrvDepthPosition
# 표준 서비스 메시지 (트리거 타입)
from std_srvs.srv import Trigger
# ROS 패키지 경로 가져오기
from ament_index_python.packages import get_package_share_directory
# 그리퍼 제어 모듈
from hospital.controller.onrobot import RG
# ROS 표준 메시지 타입 (다중 배열)
from std_msgs.msg import Float32MultiArray, Float64MultiArray
# 불리언 설정 서비스 메시지
from std_srvs.srv import SetBool

# 'hospital' 패키지의 공유 디렉토리 경로 가져오기
package_path = get_package_share_directory("hospital")

# 단일 로봇 설정 ---------------------------------------------------
# 로봇 ID 설정
ROBOT_ID = "dsr01"
# 로봇 모델 지정
ROBOT_MODEL = "m0609"
# 기본 속도와 가속도 설정 (60%)
VELOCITY, ACC = 60, 60
# 그리퍼 모델 이름 지정
GRIPPER_NAME = "rg2"
# 툴 체인저 IP 주소
TOOLCHARGER_IP = "192.168.1.1"
# 툴 체인저 포트 번호
TOOLCHARGER_PORT = "502"
# 깊이 값 보정을 위한 오프셋
DEPTH_OFFSET = -5.0
# 최소 유효 깊이 값
MIN_DEPTH = 2.0

# 추적 대상 설정 ---------------------------------------------------
# 추적할 대상 물체 이름 (오타 있음: scarpel → scalpel)
TRACKING_TRAGET = "scarpel_tip"
# 추적 대상의 클래스 ID
TRACKING_CLASS_ID=5

# DSR 로봇 초기화 --------------------------------------------------
# 전역 변수에 로봇 ID 설정
DR_init.__dsr__id = ROBOT_ID
# 전역 변수에 로봇 모델 설정
DR_init.__dsr__model = ROBOT_MODEL

# ROS 2 초기화
rclpy.init()
# ROS 2 노드 생성 (네임스페이스에 로봇 ID 사용)
dsr_node = rclpy.create_node("generate_tracking_node", namespace=ROBOT_ID)
# DSR 모듈에서 사용할 노드 설정
DR_init.__dsr__node = dsr_node

# DSR 로봇 제어 모듈 동적 임포트 -------------------------------------
try:
    # DSR 로봇 제어 명령어 임포트
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, trans, DR_MV_MOD_REL, DR_MV_RA_OVERRIDE, amovel, wait
    # DSR 공통 함수 임포트
    from DR_common2 import posj,posx
except ImportError as e:
    # 임포트 실패 시 에러 출력 후 종료
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

########### Gripper Setup. Do not modify this area ############
# 그리퍼 객체 초기화 (모델명, IP, 포트)
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

########### Robot Controller ############

# 추적 제어 노드 클래스
class TargetTracking(Node):
    def __init__(self):
        # 부모 클래스(Node) 초기화
        super().__init__("tracking_node")
        # 추적 활성화 상태 플래그
        self.tracking_active = False
        # 추적 제어 서비스 서버 생성 (SetBool 타입, '외부에서 추적 기능을 ON/OFF 할 수 있도록 서비스 서버를 제공)
        self.create_service(SetBool, '/tracking_trigger', self.handle_tracking_trigger)
        # 0.5초 주기로 실행될 타이머 콜백 설정
        self.create_timer(0.5, self.tracking_loop)
        # 칼 끝 위치 저장 리스트 초기화
        self.scalpel_tip_pos=[]
        # 마지막 칼 끝 위치 저장 변수
        self.last_tip_pos = None

        # 객체 추적 정보 구독 설정 ---------------------------------
        # '/tracked_objects_3d' 토픽 구독 (Float32MultiArray 타입)
        self.subscription_object_camera = self.create_subscription(
            Float32MultiArray,
            '/tracked_objects_3d',
            self.tracked_object_callback,
            10
        )
        
        # 현재 로봇 위치 저장 변수
        self.current_posx_topic = None
        # 로봇 현재 위치 정보 구독 설정
        self.subscription_robot_pos = self.create_subscription(
            Float64MultiArray,
            '/dsr01/msg/current_posx',
            self.current_posx_topic_listener_callback,
            10
        )

        # 그리퍼→카메라 변환 행렬 캐시 변수
        self.gripper2cam_cache = None
    
    # 추적 활성화/비활성화 서비스 핸들러 ----------------------------
    def handle_tracking_trigger(self, request, response):
        # 서비스 요청 로깅
        self.get_logger().info("========================")
        self.get_logger().info("서비스 요청 받음")
        self.get_logger().info(f"받은 데이터 : {request.data}")
        # 요청값으로 추적 상태 업데이트
        self.tracking_active = request.data
        # 상태 변경 로깅
        self.get_logger().info(f"📡 트래킹 상태: {'ON' if self.tracking_active else 'OFF'}")
        # 응답 설정
        response.success = True
        response.message = f"Tracking {'started' if request.data else 'stopped'}"
        return response

    # 로봇 현재 위치 콜백 함수 --------------------------------------
    def current_posx_topic_listener_callback(self, msg):
        # 메시지 데이터 길이 확인 (6개 값: x,y,z,rx,ry,rz)
        if len(msg.data) == 6:
            # 현재 로봇 위치 저장
            self.current_posx_topic = msg.data
        else:
            # 잘못된 데이터 형식 경고
            self.get_logger().warn("Received data does not have 6 elements.")
    
    # 현재 로봇 위치 반환 함수 ---------------------------------------
    def get_current_posx_topic(self):
        return self.current_posx_topic
    
    # 추적 객체 정보 콜백 함수 --------------------------------------
    def tracked_object_callback(self, msg):
        # 메시지 데이터 추출
        data = msg.data
        # 데이터 형식 검증 (5의 배수여야 함: track_id, class_id, x, y, z)
        if len(data) % 5 != 0:
            self.get_logger().warn("Received malformed object data")
            return
        
        # 5개씩 묶어서 처리
        for i in range(0, len(data), 5):
            track_id, track_class_id, x, y, z = data[i:i+5]
            # 추적 대상 클래스 ID 확인
            if int(track_class_id) == TRACKING_CLASS_ID:
                # 칼 끝 위치 업데이트
                self.scalpel_tip_pos = [x, y, z]

        # 디버그 로깅
        self.get_logger().debug(f"콜백 데이터~~~~~~~: {data}")
        # 정보 로깅
        self.get_logger().info(f"Updated 칼 끝: {self.scalpel_tip_pos}")

    # 로봇 포즈 행렬 생성 함수 --------------------------------------
    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        # ZYZ 오일러 각도 → 회전 행렬 변환 (도 단위)
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        # 4x4 단위 행렬 생성
        T = np.eye(4)
        # 회전 부분 설정
        T[:3, :3] = R
        # 이동 부분 설정
        T[:3, 3] = [x, y, z]
        return T

    # 카메라 좌표 → 로봇 베이스 좌표 변환 함수 -----------------------
    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        # 캐시된 변환 행렬이 없으면 파일에서 로드
        if self.gripper2cam_cache is None:
            self.gripper2cam_cache = np.load(gripper2cam_path)
        
        # 동차 좌표로 변환 (마지막에 1 추가)
        coord = np.append(np.array(camera_coords), 1)

        # 로봇 위치 정보 분해
        x, y, z, rx, ry, rz = robot_pos
        # 로봇 베이스 → 그리퍼 변환 행렬 계산
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        # 최종 변환 행렬 계산: 베이스 → 그리퍼 → 카메라
        base2cam = base2gripper @ self.gripper2cam_cache
        # 좌표 변환 적용
        td_coord = np.dot(base2cam, coord)
        
        # 깊이 값 유효성 검사
        if td_coord[2] < MIN_DEPTH:
            self.get_logger().warn(f"Invalid depth value: {td_coord[2]}")
            return None
        # 깊이 오프셋 적용
        td_coord[2] += DEPTH_OFFSET

        # x,y,z 좌표만 반환
        return td_coord[:3]

    # 추적 루프 함수 (주기적 실행) ----------------------------------
    def tracking_loop(self):
        # 현재 상태 로깅
        self.get_logger().info(f"타이머 콜백 실행중 트래킹 액티브 : {self.tracking_active}")
        # 추적 비활성화 상태면 종료
        if not self.tracking_active:
            return
        
        # 현재 로봇 위치 가져오기
        current_robot_pos = self.get_current_posx_topic()
        # 위치 정보 없으면 종료
        if not current_robot_pos:
            return
        
        # 추적 대상 이름 가져오기
        tracking_target = TRACKING_TRAGET
        # 추적 대상 위치 계산
        tracking_target_pos = self.tracking_get_target_pos()

        # 위치 정보 없으면 종료
        if not tracking_target_pos:
            self.get_logger().info(f"빈값이므로 리턴 트래킹 타켓 포즈 : {tracking_target_pos}")
            return
            
        # 위치 변경 감지 시
        if self.last_tip_pos is None or self.is_position_changed(self.last_tip_pos, tracking_target_pos):
            # 변경 로깅
            self.get_logger().info(f"위치 갱신~ {tracking_target}: {tracking_target_pos}")
            # 로봇 이동 명령 실행
            self.tracking_move(tracking_target_pos)
            # 마지막 위치 업데이트
            self.last_tip_pos = tracking_target_pos

    # 위치 변경 감지 함수
    # x,y,z 좌표 중 하나라도 threshold 이상 차이나는지 검사
    def is_position_changed(self, pos1, pos2, threshold=10.0):
        # 두 위치 리스트 길이 비교
        if len(pos1) != len(pos2):
            self.get_logger().info("위치 변경 확인")
            return True
        # x,y,z 좌표 중 하나라도 threshold(기본 10mm) 이상 차이 나면 True 반환
        return any(abs(a - b) > threshold for a, b in zip(pos1[:3], pos2[:3]))

    # 추적 대상 위치 계산 함수 --------------------------------------
    def tracking_get_target_pos(self):
        # 현재 칼 끝 위치 가져오기
        xyz = self.scalpel_tip_pos

        # 데이터 형식 검증
        if not isinstance(xyz, list) or len(xyz) < 3:
            self.get_logger().warn(f"📉 scalpel_tip_pos 길이 부족 또는 잘못된 형식: {xyz}")
            return None

        # 최소 깊이 검사
        if xyz[2] <= MIN_DEPTH:
            self.get_logger().warn(f"깊이 값이 너무 얕음: {xyz[2]}")
            return None

        # 현재 로봇 위치 가져오기
        robot_posx = self.get_current_posx_topic()
        # 위치 정보 유효성 검사
        if not robot_posx or len(robot_posx) < 6:
            self.get_logger().warn("로봇 좌표 길이 이상 , 로봇 좌표 없음")
            return None
            
        # 그리퍼→카메라 변환 행렬 경로 설정
        gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2suction.npy") 
        # 좌표 변환 수행
        td_coord = self.transform_to_base(xyz, gripper2cam_path, robot_posx)
        
        # 변환 실패 시
        if td_coord is None:
            return None
        
        # 깊이 오프셋 적용
        td_coord[2] += DEPTH_OFFSET
        # 최소 깊이 보장
        td_coord[2] = max(td_coord[2], MIN_DEPTH)
        
        # 변환된 좌표 + 로봇 회전값 반환
        return list(td_coord[:3]) + list(robot_posx[3:6])
    
    # 추적 이동 명령 함수 -------------------------------------------
    def tracking_move(self, target_pos): 
        # 이동 명령 로깅
        print(f"트래킹 실행 좌표 : {target_pos}")
        
        if target_pos[2] < 80:
            target_pos[2] = 80
        
        # 로봇 이동 명령 (속도 20%, 가속도 20%, 오버라이드 모드)
        amovel(target_pos, vel=20, acc=20, mod=0, radius=10, ra=DR_MV_RA_OVERRIDE)
        # 0.8초 대기
        wait(0.8)

# 메인 함수 --------------------------------------------------------
def main(args=None):
    # 노드 객체 생성
    node = TargetTracking()
    # 노드 실행 (이벤트 루프 진입)
    rclpy.spin(node)
    # 종료 시 ROS 2 정리
    rclpy.shutdown()
    # 노드 제거
    node.destroy_node()

# 스크립트 직접 실행 시
if __name__ == "__main__":
    main()